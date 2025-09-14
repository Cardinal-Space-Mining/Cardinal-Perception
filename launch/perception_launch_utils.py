import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

sys.path.append(os.path.join(get_package_share_directory('launch_utils'), 'src'))
from launch_utils.actions import NodeAction


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
        node_action = NodeAction(percept_config)
        node_action.remappings['/trace_notifications'] = '/cardinal_perception/trace_notifications'
        actions.append(
            node_action.format_node(
                package = 'cardinal_perception',
                executable = 'perception_node',
                output = 'screen'
            )
        )
        actions.append(
            Node(
                exec_name = 'perception_profiling_manager',
                package = 'csm_metrics',
                executable = 'profiling_manager.py',
                output = 'screen',
                parameters = [{'notification_topic': '/cardinal_perception/trace_notifications'}]
            ) )
    if 'tag_detection' in config:
        tag_det_config = config['tag_detection']
        preproc_tag_detector_config(tag_det_config)
        actions.append(
            NodeAction(tag_det_config).format_node(
                package = 'cardinal_perception',
                executable = 'tag_detection_node',
                output = 'screen'
            )
        )
    if 'pplan_client' in config:
        pplan_client_config = config['pplan_client']
        actions.append(
            NodeAction(pplan_client_config).format_node(
                package = 'cardinal_perception',
                executable = 'pplan_client_node',
                output = 'screen'
            )
        )
    return actions
