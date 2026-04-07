# mission_status

Live terminal dashboard for the PAC fleet. Displays mission configuration and per-drone status in a continuously updated table.
New robots are discovered automatically by watching for `/<ns>/pose` topics.

## Run

```bash
ros2 run mission_status mission_status
```

## Dependencies

- `rclpy`, `std_msgs`, `geometry_msgs` — standard ROS 2
- `async_pac_gnn_interfaces` — `RobotStatus`, `MissionControl` message types
- `python3-rich` — terminal UI
