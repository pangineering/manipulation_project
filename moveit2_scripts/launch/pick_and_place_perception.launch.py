import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Create MoveItConfigs using MoveItConfigsBuilder
    moveit_config = MoveItConfigsBuilder("name", package_name="my_moveit_config").to_moveit_configs()

    # Define nodes
    simple_grasping = Node(
        package='simple_grasping',
        executable='basic_grasping_perception_node',
        name='basic_grasping_perception_node',
        output='screen',
       # arguments=['--ros-args', '-p', 'debug_topics:=true']
    ),
    moveit_cpp_node = Node(
        package="moveit2_scripts",
        executable="pick_and_place_perception_node",
        name="pick_and_place_perception_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )



    # Return the LaunchDescription with both nodes
    return LaunchDescription([simple_grasping, moveit_cpp_node])