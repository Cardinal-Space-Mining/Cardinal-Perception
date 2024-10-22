import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('cardinal_perception')
    localization_config = os.path.join(pkg_path, 'config', 'localization.yaml')
    tag_detection_config = os.path.join(pkg_path, 'config', 'tag_detection.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    localization_node = Node(
        name = 'cardinal_perception_localization',
        package = 'cardinal_perception',
        executable = 'localization_node',
        output = 'screen',
        parameters = [localization_config, {'use_sim_time': use_sim_time}],
        remappings = [
            ('filtered_scan', '/cardinal_perception/filtered_scan'),
            ('path', '/cardinal_perception/optimized_path')
        ]
    )
    tag_detection_node = Node(
        name = 'cardinal_perception_tag_detection',
        package = 'cardinal_perception',
        executable = 'tag_detection_node',
        output = 'screen',
        parameters = [tag_detection_config, {'use_sim_time': use_sim_time}],
        remappings = []
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # localization_node,
        tag_detection_node
    ])
