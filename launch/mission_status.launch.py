from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="mission_status",
            executable="mission_status",
            name="mission_status",
            namespace="gcs",
            output="screen",
        )
    ])
