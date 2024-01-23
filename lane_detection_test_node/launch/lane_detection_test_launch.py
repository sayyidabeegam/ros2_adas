import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lane_detection_test_node',
            executable='lane_detection_test_node',
            name='lane_detection_test_node',
        ),
    ])

