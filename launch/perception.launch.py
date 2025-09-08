import os
import sys
import json
import yaml
import math
import collections
from datetime import datetime

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

try:
    sys.path.append(os.path.join(get_package_share_directory('launch_utils'), 'src'))
    from launch_utils import utilities as lu
    print('SUCCESSFULLY LOADED LAUNCH UTILS')
    have_lu = True
except Exception as e:
    print('FAILED TO LOAD LAUNCH UTILS')
    have_lu = False


PKG_PATH = get_package_share_directory('cardinal_perception')
DEFAULT_JSON_PATH = os.path.join(PKG_PATH, 'config', 'perception.json')


def try_load_json(json_path, default_json_path = ''):
    if not json_path:
        json_path = default_json_path
    try:
        with open(json_path, 'r') as f: json_data = f.read()
    except Exception as e:
        raise RuntimeError(f"JSON file '{json_path}' does not exist or could not be read : {e}")
    try:
        return json.loads(json_data)
    except Exception as e:
        raise RuntimeError(f"Failed to load json data from file '{json_path}' : {e}")

def expand_exclusion_zones(config):
    if 'exclusion_zones' in config:
        expanded_zones = {}
        idx = 0
        for zone in config['exclusion_zones']:
            expanded_zones[f'zone{idx}.frame_id'] = zone['frame_id']
            expanded_zones[f'zone{idx}.min'] = zone['min']
            expanded_zones[f'zone{idx}.max'] = zone['max']
            idx += 1
        expanded_zones['num_zones'] = idx
        config['exclusion_zones'] = expanded_zones
    else:
        config['exclusion_zones'] = { 'num_zones' : 0 }

def flatten_dict(d, parent_key='', sep='.'):
    items = {}
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.update(flatten_dict(v, new_key, sep=sep))
        else:
            items[new_key] = v
    return items


def launch(context, *args, **kwargs):
    json_path = str(LaunchConfiguration('json_path').perform(context))
    json_data = str(LaunchConfiguration('json_data').perform(context))
    record_mode = str(LaunchConfiguration('record').perform(context))
    bag = str(LaunchConfiguration('bag').perform(context))

    if not json_data:
        json_data = try_load_json(json_path, DEFAULT_JSON_PATH)

    actions = []

    if 'perception' in json_data:
        config = json_data['perception']
        if have_lu:
            actions.extend(lu.get_util_actions(json_data))
    else:
        config = json_data

    expand_exclusion_zones(config)
    actions.append(
        Node(
            name = 'perception',
            package = 'cardinal_perception',
            executable = 'perception_node',
            output = 'screen',
            parameters = [flatten_dict(config)]
        )
    )

    return actions



def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('json_path', default_value=''),
        DeclareLaunchArgument('json_data', default_value=''),
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('bag', default_value=''),
        OpaqueFunction(function=launch)
    ])
