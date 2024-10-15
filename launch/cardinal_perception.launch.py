import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('cardinal_perception')
    param_file = os.path.join(pkg_path, 'config', 'params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    perception_node = Node(
        name = 'cardinal_perception',
        package = 'cardinal_perception',
        executable = 'perception_node',
        output = 'screen',
        parameters = [param_file, {'use_sim_time': use_sim_time}],
        remappings = [
            ('debug_img', '/cardinal_perception/debug_img'),
            ('filtered_scan', '/cardinal_perception/filtered_scan'),
            ('path', '/cardinal_perception/optimized_path')
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        perception_node
    ])
