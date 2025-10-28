import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import OpaqueFunction

try:
    sys.path.append(os.path.join(get_package_share_directory('launch_utils'), 'src'))
    from launch_utils.preprocess import preprocess_launch_json
    from launch_utils.actions import get_util_actions
    from launch_utils.common import try_load_json_from_args, parse_launch_args
    HAVE_LAUNCH_UTILS = True
except Exception as e:
    HAVE_LAUNCH_UTILS = False

from perception_launch_utils import get_perception_actions


PKG_PATH = get_package_share_directory('cardinal_perception')
DEFAULT_JSON_PATH = os.path.join(PKG_PATH, 'config', 'perception.json')

def launch(context, *args, **kwargs):
    actions = []

    if HAVE_LAUNCH_UTILS:
        launch_args = parse_launch_args(context.argv)
        json_data = try_load_json_from_args(launch_args, DEFAULT_JSON_PATH)
        config = preprocess_launch_json(json_data, launch_args)
        if config is not json_data:
            actions.extend(get_util_actions(config, launch_args))
        actions.extend(get_perception_actions(config))
    else:
        print("The 'launch_utils' package is needed to launch Cardinal Perception using JSON action configs.")

    return actions


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch),
    ])
