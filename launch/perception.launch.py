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
    from launch_utils.actions import get_util_actions
    from launch_utils.common import try_load_json_from_args, parse_launch_args, flatten_dict
    print('SUCCESSFULLY LOADED LAUNCH UTILS')
    HAVE_LAUNCH_UTILS = True
except Exception as e:
    print('FAILED TO LOAD LAUNCH UTILS')
    HAVE_LAUNCH_UTILS = False


PKG_PATH = get_package_share_directory('cardinal_perception')
DEFAULT_JSON_PATH = os.path.join(PKG_PATH, 'config', 'perception.json')


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
    if 'aruco' in config and 'tags' in config['aruco']:
        flattened_tags = {}
        flattened_tags['ids'] = []
        for tag in config['aruco']['tags']:
            id = tag['id']
            flattened_tags['ids'].append(id)
            flattened_tags[f'tag{id}_static'] = tag['static']
            flattened_tags[f'tag{id}_frames'] = tag['frames']
            flattened_tags[f'tag{id}_corners'] = tag['corners']
        config['aruco']['tags'] = flattened_tags
    else:
        if 'aruco' not in config:
            config['aruco'] = {}
        config['aruco']['tags'] = {'ids':[]}

def preproc_perception_config(config):
    flatten_exclusion_zones(config)

def preproc_tag_detector_config(config):
    flatten_streams(config)
    flatten_tags(config)

def get_perception_actions(config):
    actions = []
    if 'perception' in config:
        percept_config = config['perception']
        preproc_perception_config(percept_config)
        actions.append(
            Node(
                name = 'perception',
                package = 'cardinal_perception',
                executable = 'perception_node',
                output = 'screen',
                parameters = [flatten_dict(percept_config)],
                remappings = [('/trace_notifications', '/cardinal_perception/trace_notifications')]
            ) )
        actions.append(
            Node(
                name = 'perception_profiling_manager',
                package = 'csm_metrics',
                executable = 'profiling_manager.py',
                output = 'screen',
                parameters = [{'notification_topic': '/cardinal_perception/trace_notifications'}]
            ) )
    if 'tag_detection' in config:
        tag_det_config = config['tag_detection']
        preproc_tag_detector_config(tag_det_config)
        actions.append(
            Node(
                name = 'tags_detector',
                package = 'cardinal_perception',
                executable = 'tag_detection_node',
                output = 'screen',
                parameters = [flatten_dict(tag_det_config)]
            )
        )
    return actions


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
        print("To properly launch Cardinal Perception the 'launch_utils' package is needed!")

    return actions


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch),
    ])
