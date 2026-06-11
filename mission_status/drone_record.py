import threading
from dataclasses import dataclass, field
import time


@dataclass
class MissionRecord:
    # /mission_origin_gps (geometry_msgs/Point: x=lon, y=lat, z=heading)
    origin_lon: float | None = None
    origin_lat: float | None = None
    origin_heading: float | None = None
    # /mission_control flags
    hw_enable: bool | None = None
    ob_enable: bool | None = None
    ob_takeoff: bool | None = None
    ob_land: bool | None = None
    geofence: bool | None = None
    pac_offboard_only: bool | None = None
    pac_lpac_l1: bool | None = None
    pac_lpac_l2: bool | None = None
    # world geometry (from /sim/get_system_info)
    world_size: float | None = None        # model units (e.g. 512)
    env_scale_factor: float | None = None  # divisor: phys_m = world_size / env_scale_factor
    # density map (from /sim/get_world_map, flat row-major float32 array)
    density_map: list | None = None
    density_map_size: int | None = None    # side length N; total = N*N
    # geofence buffer distances in physical metres (from lpac_l2.yaml)
    # fence spans [-buf_l, phys_extent+buf_r] x [-buf_b, phys_extent+buf_t]
    fence_x_buf_l: float = 5.0
    fence_x_buf_r: float = 5.0
    fence_y_buf_b: float = 5.0
    fence_y_buf_t: float = 5.0


class MissionStore:
    def __init__(self):
        self._lock = threading.Lock()
        self._record = MissionRecord()

    def update(self, **kwargs):
        with self._lock:
            for k, v in kwargs.items():
                setattr(self._record, k, v)

    def snapshot(self) -> MissionRecord:
        with self._lock:
            return MissionRecord(**self._record.__dict__)


@dataclass
class DroneRecord:
    namespace: str
    # pose subscription fields
    position: tuple | None = None
    # RobotStatus fields (updated on state change only)
    state: str | None = None
    diagnostic: str | None = None
    breach: bool | None = None
    gps_lat: float | None = None
    gps_lon: float | None = None
    gps_alt: float | None = None
    gps_sats: int | None = None
    gps_heading: float | None = None
    local_pos_x: float | None = None
    local_pos_y: float | None = None
    local_pos_z: float | None = None
    local_pos_heading: float | None = None
    ned_vel_x: float | None = None
    ned_vel_y: float | None = None
    ned_vel_z: float | None = None
    preflight_pass: bool | None = None
    arming_state: int | None = None
    disarming_reason: int | None = None
    nav_state: int | None = None
    gcs_conn_lost: bool | None = None
    failure_detector_status: int | None = None
    safety_off: bool | None = None
    batt_volt: float | None = None
    batt_pct: int | None = None
    batt_curr: float | None = None
    batt_status: str | None = None
    last_seen: float = field(default_factory=time.monotonic)
    stale: bool = False

class DroneStateStore:
    def __init__(self, stale_threshold_sec: float = 3.0):
        self._lock = threading.Lock()
        self._drones: dict[str, DroneRecord] = {}
        self._stale_threshold = stale_threshold_sec

    def update(self, namespace: str, **kwargs):
        with self._lock:
            if namespace not in self._drones:
                self._drones[namespace] = DroneRecord(namespace=namespace)
            rec = self._drones[namespace]
            for k, v in kwargs.items():
                setattr(rec, k, v)
            rec.last_seen = time.monotonic()
            rec.stale = False

    def snapshot(self) -> list[DroneRecord]:
        with self._lock:
            now = time.monotonic()
            for rec in self._drones.values():
                rec.stale = (now - rec.last_seen) > self._stale_threshold
            return list(self._drones.values())

    def inject(self, namespace: str, **kwargs):
        self.update(namespace, **kwargs)
