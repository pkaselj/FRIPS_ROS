"""
    This is a ROS2 launch script to start all nodes for FRIPS_ROS.
    Following nodes are started:

    - rover_controller\rover_controller_node as rover_controller_node
    - rover_interface\rover_interface_node as rover_interface_node
    - rover_pose_estimator\rover_pose_estimator_node as rover_pose_estimator_node
    - rover_protocol_hub\rover_protocol_hub_node as rover_protocol_hub_node
    - rover_forward_kinematics\rover_fwd_kinm_node as rover_fwd_kinm_node
    - rover_inverse_kinematics\rover_inv_kinm_node as rover_inv_kinm_node
    - marvelmind_ros2\marvelmind_ros2 as marvelmind_ros2_left_beacon
    - marvelmind_ros2\marvelmind_ros2 as marvelmind_ros2_right_beacon
    - marvelmind_position_fusion\marvelmind_position_fusion_node as marvelmind_position_fusion_node

    How to run:
    - Source ROS2 underlay
    - Source FRIPS_ROS overlay
    - ros2 launch rover_launcher rover.launch.py

    Author: Petar Kaselj
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def get_parameters(pkg_name, param_file_name = 'params.yaml'):
    params = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        param_file_name
    )
    return [params]

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_controller',
            executable='rover_controller_node',
            name='rover_controller_node',
            parameters=get_parameters('rover_launcher'),
            arguments=(
                '--ros-args',
                '--log-level', 'INFO'
            )
        ),
        Node(
            package='rover_interface',
            executable='rover_interface_node',
            name='rover_interface_node',
            parameters=get_parameters('rover_launcher'),
            arguments=(
                '--ros-args',
                '--log-level', 'INFO'
            )
        ),
        Node(
            package='rover_pose_estimator',
            executable='rover_pose_estimator_node',
            name='rover_pose_estimator_node',
            parameters=get_parameters('rover_launcher'),
            arguments=(
                '--ros-args',
                '--log-level', 'INFO'
            )
        ),
        Node(
            package='rover_protocol_hub',
            executable='rover_protocol_hub_node',
            name='rover_protocol_hub_node',
            parameters=get_parameters('rover_launcher'),
            arguments=(
                '--ros-args',
                '--log-level', 'INFO'
            )
        ),
        Node(
            package='rover_forward_kinematics',
            executable='rover_fwd_kinm_node',
            name='rover_fwd_kinm_node',
            parameters=get_parameters('rover_launcher'),
            arguments=(
                '--ros-args',
                '--log-level', 'INFO'
            )
        ),
        Node(
            package='rover_inverse_kinematics',
            executable='rover_inv_kinm_node',
            name='rover_inv_kinm_node',
            parameters=get_parameters('rover_launcher'),
            arguments=(
                '--ros-args',
                '--log-level', 'INFO'
            )
        ),
        Node(
            package='marvelmind_ros2',
            executable='marvelmind_ros2',
            name='marvelmind_ros2_left_beacon',
            parameters=get_parameters('rover_launcher'),
            arguments=(
                '--ros-args',
                '--log-level', 'INFO'
            )
        ),
        Node(
            package='marvelmind_ros2',
            executable='marvelmind_ros2',
            name='marvelmind_ros2_right_beacon',
            parameters=get_parameters('rover_launcher'),
            arguments=(
                '--ros-args',
                '--log-level', 'INFO'
            )
        ),
        Node(
            package='marvelmind_position_fusion',
            executable='marvelmind_position_fusion_node',
            name='marvelmind_position_fusion_node',
            parameters=get_parameters('rover_launcher'),
            arguments=(
                '--ros-args',
                '--log-level', 'INFO'
            )
        )
    ])

