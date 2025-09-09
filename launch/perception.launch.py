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

try:
    sys.path.append(os.path.join(get_package_share_directory('launch_utils'), 'src'))
    from launch_utils.preprocess import preprocess_launch_json
    from launch_utils.common import try_load_json, parse_launch_args
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

def flatten_exclusion_zones(config):
    if 'exclusion_zones' in config:
        flattened_zones = {}
        idx = 0
        for zone in config['exclusion_zones']:
            flattened_zones[f'zone{idx}.frame_id'] = zone['frame_id']
            flattened_zones[f'zone{idx}.min'] = zone['min']
            flattened_zones[f'zone{idx}.max'] = zone['max']
            idx += 1
        flattened_zones['num_zones'] = idx
        config['exclusion_zones'] = flattened_zones
    else:
        config['exclusion_zones'] = { 'num_zones' : 0 }

def flatten_streams(config):
    if 'streams' in config:
        flattened_streams = {}
        idx = 0
        for stream in config['streams']:
            flattened_streams[f'stream{idx}'] = stream['topics']
            flattened_streams[f'stream{idx}_offset'] = stream['offset']
            idx += 1
        flattened_streams['num_streams'] = idx
        config['streams'] = flattened_streams
    else:
        config['streams'] = { 'num_streams' : 0 }

def flatten_tags(config):
    if 'tags' in config:
        flattened_tags = {}
        flattened_tags['ids'] = []
        for tag in config['tags']:
            id = tag['id']
            flattened_tags['ids'].append(id)
            flattened_tags[f'tag{id}_static'] = tag['static']
            flattened_tags[f'tag{id}_frames'] = tag['frames']
            flattened_tags[f'tag{id}_corners'] = tag['corners']
    else:
        config['tags'] = {'ids':[]}

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

    if not json_data:
        json_data = try_load_json(json_path, DEFAULT_JSON_PATH)

    actions = []

    if have_lu:
        launch_args = parse_launch_args(context.argv)
        config = preprocess_launch_json(json_data, launch_args)
        for k, _ in config.items():
            print(f"Loaded action config for '{k}'")
        # if a non-preproc scope was passed (and no preprocessing occurred), we DO NOT want to do this in case there are accidentally matching tags!
        actions.extend(lu.get_util_actions(config))
        percept_config = config['perception'] if 'perception' in config else {}
    else:
        if 'perception' in json_data:
            for k, v in json_data['perception'].items():
                if isinstance(v, dict):
                    percept_config = v
                    break
        else:
            percept_config = json_data

    flatten_exclusion_zones(percept_config)
    actions.append(
        Node(
            name = 'perception',
            package = 'cardinal_perception',
            executable = 'perception_node',
            output = 'screen',
            parameters = [flatten_dict(percept_config)]
        )
    )

    return actions



def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('json_path', default_value=''),
        DeclareLaunchArgument('json_data', default_value=''),
        OpaqueFunction(function=launch),
    ])
