import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lane_detection',  # Replace with the actual name of your ROS2 package
            executable='lane_detection_node',
            name='lane_detection_node',
        ),
    ])

