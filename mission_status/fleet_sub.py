import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, Point
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from async_pac_gnn_interfaces.msg import RobotStatus, MissionControl

from mission_status.drone_record import DroneStateStore, MissionStore


class FleetSubscriber(Node):
    def __init__(self, store: DroneStateStore, mission_store: MissionStore, namespaces: list[str]):
        super().__init__("gcs_fleet_subscriber")
        self._store = store
        self._mission_store = mission_store
        self._subscribed: set[str] = set()

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
            # match /<ns>/pose — ignore anything nested deeper
            parts = topic.strip("/").split("/")
            if len(parts) == 2 and parts[1] == "pose" and parts[0] not in self._subscribed:
                self._make_subs(parts[0])

    def _make_subs(self, ns: str):
        def on_robot_status(msg: RobotStatus):
            self._store.update(
                ns,
                state=msg.state,
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
