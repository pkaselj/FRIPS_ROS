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
            parameters=get_parameters('rover_controller'),
            arguments=(
                '--ros-args',
                '--log-level', 'INFO'
            )
        ),
        Node(
            package='rover_interface',
            executable='rover_interface_node',
            name='rover_interface_node',
            parameters=get_parameters('rover_interface'),
            arguments=(
                '--ros-args',
                '--log-level', 'INFO'
            )
        ),
        Node(
            package='rover_pose_estimator',
            executable='rover_pose_estimator_node',
            name='rover_pose_estimator_node',
            parameters=get_parameters('rover_pose_estimator'),
            arguments=(
                '--ros-args',
                '--log-level', 'INFO'
            )
        ),
        Node(
            package='rover_protocol_hub',
            executable='rover_protocol_hub_node',
            name='rover_protocol_hub_node',
            parameters=get_parameters('rover_protocol_hub'),
            arguments=(
                '--ros-args',
                '--log-level', 'DEBUG'
            )
        )
    ])