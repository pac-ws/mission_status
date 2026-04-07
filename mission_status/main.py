import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor

from mission_status.drone_record import DroneStateStore, MissionStore
from mission_status.fleet_sub import FleetSubscriber
from mission_status.ui import ui_main

# ── ROS thread ────────────────────────────────────────────────────────────────


def ros_thread_main(node):
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()          # blocks this thread only


def launch(namespaces: list[str]) -> tuple[DroneStateStore, MissionStore]:
    rclpy.init()
    store = DroneStateStore()
    mission_store = MissionStore()
    node = FleetSubscriber(store, mission_store, namespaces)

    t = threading.Thread(target=ros_thread_main, args=(node,), daemon=True)
    t.start()

    return store, mission_store


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    # 1. Start ROS thread — returns immediately, spins in background
    store, mission_store = launch([])

    # 2. Hand control to the rich UI — this blocks the main thread (intentional)
    ui_main(store, mission_store)


if __name__ == "__main__":
    main()
