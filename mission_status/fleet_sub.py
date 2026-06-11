import os
import re
import subprocess
import threading
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, Point
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from async_pac_gnn_interfaces.msg import RobotStatus, MissionControl
from async_pac_gnn_interfaces.srv import SystemInfo, WorldMap

from mission_status.drone_record import DroneStateStore, MissionStore


BATTERY_POLL_INTERVAL = 180  # seconds

_FENCE_KEYS = {'fence_x_buf_l', 'fence_x_buf_r', 'fence_y_buf_b', 'fence_y_buf_t'}


def _load_fence_params() -> dict:
    """Read fence buffer values from launch/starling/lpac_l2.yaml."""
    pac_ws = os.environ.get('PAC_WS', os.path.expanduser('~/pac_ws'))
    yaml_path = os.path.join(pac_ws, 'launch', 'starling', 'lpac_l2.yaml')
    result: dict = {}
    try:
        with open(yaml_path) as f:
            data = yaml.safe_load(f)

        def _search(obj):
            if isinstance(obj, dict):
                if obj.get('name') in _FENCE_KEYS:
                    result[obj['name']] = float(obj['value'])
                for v in obj.values():
                    _search(v)
            elif isinstance(obj, list):
                for item in obj:
                    _search(item)

        _search(data)
    except Exception:
        pass
    return result
ROBOT_IP_PREFIX = "192.168.0.1"  # r<N> -> 192.168.0.1<NN>
ROBOT_SSH_PASS = "oelinux123"


def _robot_ssh_host(namespace: str) -> str:
    """Convert robot namespace (e.g. 'r5') to its SSH IP (e.g. '192.168.0.105')."""
    m = re.match(r'^r(\d+)$', namespace)
    if m:
        return f"{ROBOT_IP_PREFIX}{int(m.group(1)):02d}"
    return namespace


def _poll_battery(namespace: str, store: DroneStateStore) -> None:
    host = _robot_ssh_host(namespace)
    try:
        result = subprocess.run(
            [
                "sshpass", "-p", ROBOT_SSH_PASS,
                "ssh", "-o", "StrictHostKeyChecking=no",
                f"root@{host}", "px4-listener battery_status -n 1",
            ],
            capture_output=True, text=True, timeout=15,
        )
        output = result.stdout + result.stderr
    except Exception as e:
        store.update(namespace, batt_status=f"Error: {e}")
        return

    if not output.strip():
        store.update(namespace, batt_status=f"Error: empty output (rc={result.returncode})")
        return

    volt = curr = remaining = None
    for line in output.splitlines():
        parts = line.split()           # awk-style: split on whitespace, $1=key $2=value
        if len(parts) < 2:
            continue
        key, val = parts[0], parts[1]
        if key == "voltage_v:":
            try:
                volt = float(val)
            except ValueError:
                pass
        elif key == "current_a:":
            try:
                curr = float(val)
            except ValueError:
                pass
        elif key == "remaining:":
            try:
                remaining = float(val)
            except ValueError:
                pass

    if remaining is not None:
        pct = round(remaining * 100)
        if remaining >= 0.80:
            status = "OK"
        elif remaining >= 0.40:
            status = "Low"
        elif remaining >= 0.15:
            status = "Critical"
        else:
            status = "EMPTY"
    else:
        pct = None
        status = "Unknown"

    store.update(namespace, batt_volt=volt, batt_pct=pct, batt_curr=curr, batt_status=status)


def _battery_poll_loop(namespace: str, store: DroneStateStore) -> None:
    while True:
        _poll_battery(namespace, store)
        time.sleep(BATTERY_POLL_INTERVAL)


class FleetSubscriber(Node):
    def __init__(self, store: DroneStateStore, mission_store: MissionStore, namespaces: list[str]):
        super().__init__("gcs_fleet_subscriber")
        self._store = store
        self._mission_store = mission_store
        self._subscribed: set[str] = set()
        self._battery_polled: set[str] = set()

        self.qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        for ns in namespaces:
            self._make_subs(ns)

        self.create_timer(2.0, self._discover)
        self._make_mission_subs()

        self._world_info_done = False
        self._world_info_waiting = False
        self._density_done = False
        self._density_waiting = False
        self._sys_info_client = self.create_client(SystemInfo, '/sim/get_system_info')
        self._world_map_client = self.create_client(WorldMap, '/sim/get_world_map')
        self.create_timer(3.0, self._fetch_world_info)

        fence_params = _load_fence_params()
        if fence_params:
            self._mission_store.update(**fence_params)

    def _make_mission_subs(self):
        def on_mission_origin(msg: Point):
            self._mission_store.update(
                origin_lon=msg.x,
                origin_lat=msg.y,
                origin_heading=msg.z,
            )

        def on_mission_control(msg: MissionControl):
            self._mission_store.update(
                hw_enable=msg.hw_enable,
                ob_enable=msg.ob_enable,
                ob_takeoff=msg.ob_takeoff,
                ob_land=msg.ob_land,
                geofence=msg.geofence,
                pac_offboard_only=msg.pac_offboard_only,
                pac_lpac_l1=msg.pac_lpac_l1,
                pac_lpac_l2=msg.pac_lpac_l2,
            )

        self.create_subscription(Point, "/mission_origin_gps", on_mission_origin, self.qos_profile)
        self.create_subscription(MissionControl, "/mission_control", on_mission_control, self.qos_profile)

    def _discover(self):
        for topic, _ in self.get_topic_names_and_types():
            parts = topic.strip("/").split("/")
            if len(parts) == 2 and parts[1] == "robot_status" and parts[0] not in self._subscribed:
                self._make_subs(parts[0])

    def _fetch_world_info(self):
        if not self._world_info_done and not self._world_info_waiting:
            if self._sys_info_client.service_is_ready():
                req = SystemInfo.Request()
                req.name = ''
                future = self._sys_info_client.call_async(req)
                future.add_done_callback(self._on_system_info)
                self._world_info_waiting = True

        if self._world_info_done and not self._density_done and not self._density_waiting:
            if self._world_map_client.service_is_ready():
                req = WorldMap.Request()
                req.map_size = 32
                future = self._world_map_client.call_async(req)
                future.add_done_callback(self._on_world_map)
                self._density_waiting = True

    def _on_system_info(self, future):
        self._world_info_waiting = False
        try:
            result = future.result()
            if result is None:
                return
            self._mission_store.update(
                world_size=result.world_size,
                env_scale_factor=result.env_scale_factor,
            )
            self._world_info_done = True
        except Exception:
            pass

    def _on_world_map(self, future):
        self._density_waiting = False
        try:
            result = future.result()
            if result is None or not result.success:
                return
            data = list(result.map.data)
            n = int(len(data) ** 0.5 + 0.5)
            self._mission_store.update(density_map=data, density_map_size=n)
            self._density_done = True
        except Exception:
            pass

    def _make_subs(self, ns: str):
        def on_robot_status(msg: RobotStatus):
            self._store.update(
                ns,
                state=msg.state,
                diagnostic=msg.diagnostic,
                breach=msg.breach,
                gps_lat=msg.gps_lat,
                gps_lon=msg.gps_lon,
                gps_alt=msg.gps_alt,
                gps_sats=msg.gps_sats,
                gps_heading=msg.gps_heading,
                local_pos_x=msg.local_pos_x,
                local_pos_y=msg.local_pos_y,
                local_pos_z=msg.local_pos_z,
                local_pos_heading=msg.local_pos_heading,
                ned_vel_x=msg.ned_vel_x,
                ned_vel_y=msg.ned_vel_y,
                ned_vel_z=msg.ned_vel_z,
                preflight_pass=msg.pre_flight_checks_pass,
                arming_state=msg.arming_state,
                disarming_reason=msg.disarming_reason,
                nav_state=msg.nav_state,
                gcs_conn_lost=msg.gcs_conn_lost,
                failure_detector_status=msg.failure_detector_status,
                safety_off=msg.safety_off,
            )

        def on_position(msg: PoseStamped):
            self._store.update(
                ns,
                position=(
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z
                )
            )
        self._subscribed.add(ns)
        self.create_subscription(RobotStatus, f"/{ns}/robot_status", on_robot_status, self.qos_profile)
        self.create_subscription(PoseStamped, f"/{ns}/pose", on_position, self.qos_profile)

        if ns not in self._battery_polled:
            self._battery_polled.add(ns)
            t = threading.Thread(
                target=_battery_poll_loop, args=(ns, self._store), daemon=True
            )
            t.start()
