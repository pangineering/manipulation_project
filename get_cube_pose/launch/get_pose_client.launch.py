# get_pose_client_launch.py
import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='simple_grasping',
            executable='basic_grasping_perception_node',
            name='basic_grasping_perception_node',
            output='screen',
            arguments=['--ros-args', '-p', 'debug_topics:=true']
        ),
        Node(
            package='get_cube_pose',
            executable='get_pose_client_node',
            name='get_pose_client_node',
            output='screen'
        ),

    ])
