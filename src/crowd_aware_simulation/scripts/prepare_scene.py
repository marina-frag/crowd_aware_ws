"""Offline launch helper: derive simulator geometry and configs from the real URDF.
No ROS publishers or planner implementation live here.
"""
from pathlib import Path
import copy
import math
import random
import re
import struct
import xml.etree.ElementTree as ET
import numpy as np
import xacro
import yaml
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from setup_config import load_configuration, load_scenario_defaults


class RosParamDumper(yaml.SafeDumper):
    # rcl_yaml_param_parser rejects aliases even though they are valid YAML.
    def ignore_aliases(self, data):
        return True


def dump_yaml(data):
    return yaml.dump(data, Dumper=RosParamDumper)


SCENARIO_NAME = re.compile(r'^[a-z][a-z0-9_]*$')


def _finite_numbers(value, length, label):
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f'{label} must be a list of {length} numbers')
    try:
        result = [float(item) for item in value]
    except (TypeError, ValueError) as error:
        raise ValueError(f'{label} must contain only numbers') from error
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f'{label} must contain only finite numbers')
    return result


def _asset_path(root, value, label):
    if not isinstance(value, str) or not value:
        raise ValueError(f'scenario.{label} must be a non-empty filename')
    relative = Path(value)
    if relative.is_absolute() or len(relative.parts) != 1 or relative.name != value:
        raise ValueError(f'scenario.{label} must be a filename within {root.name}/')
    path = (root/value).resolve()
    if path.parent != root.resolve() or not path.is_file():
        raise ValueError(f'scenario.{label} does not exist: {path}')
    return path


def _load_scenario(package_root, scenario, scenario_config=None, experiment=None):
    package_root = Path(package_root).resolve()
    if scenario_config:
        config_path = Path(scenario_config).resolve()
        requested_name = None
    elif isinstance(scenario, (str, Path)) and (Path(str(scenario)).suffix == '.yaml' or
                                                len(Path(str(scenario)).parts) > 1):
        config_path = Path(scenario).resolve()
        requested_name = None
    else:
        requested_name = str(scenario)
        if not SCENARIO_NAME.fullmatch(requested_name):
            raise ValueError(f'Invalid scenario name {requested_name!r}')
        config_path = package_root/'config/scenarios'/f'{requested_name}.yaml'
    if not config_path.is_file():
        available = sorted(path.stem for path in (package_root/'config/scenarios').glob('*.yaml'))
        raise ValueError(f'Unknown scenario {scenario!r}; available scenarios: {available}')
    document = yaml.safe_load(config_path.read_text())
    if not isinstance(document, dict) or not isinstance(document.get('scenario'), dict):
        raise ValueError(f'{config_path}: missing scenario mapping')
    config = load_scenario_defaults(package_root, document['scenario'], experiment)
    name = config.get('name')
    if not isinstance(name, str) or not SCENARIO_NAME.fullmatch(name):
        raise ValueError(f'{config_path}: scenario.name is invalid')
    if requested_name and name != requested_name:
        raise ValueError(f'{config_path}: scenario.name must be {requested_name!r}')
    config['_config_path'] = config_path
    config['_world_path'] = _asset_path(package_root/'worlds', config.get('world'), 'world')
    config['_agents_path'] = _asset_path(package_root/'scenarios', config.get('agents'), 'agents')

    robot = config.get('robot')
    if not isinstance(robot, dict):
        raise ValueError(f'{config_path}: scenario.robot must be a mapping')
    robot['initial_pose'] = _finite_numbers(
        robot.get('initial_pose'), 3, f'{config_path}: scenario.robot.initial_pose')
    path = robot.get('path_xy')
    if not isinstance(path, list) or len(path) < 2:
        raise ValueError(f'{config_path}: scenario.robot.path_xy needs at least two points')
    robot['path_xy'] = [
        _finite_numbers(point, 2, f'{config_path}: scenario.robot.path_xy[{index}]')
        for index, point in enumerate(path)]
    if robot['path_xy'][0] != robot['initial_pose'][:2]:
        raise ValueError(f'{config_path}: path_xy must start at robot.initial_pose x,y')

    intended = config.get('intended_user')
    if not isinstance(intended, dict) or not isinstance(intended.get('enabled'), bool):
        raise ValueError(f'{config_path}: scenario.intended_user.enabled must be boolean')
    if not isinstance(intended.get('agent_name'), str):
        raise ValueError(f'{config_path}: scenario.intended_user.agent_name must be a string')
    if intended['enabled'] != bool(intended['agent_name']):
        raise ValueError(
            f'{config_path}: enabled intended_user requires exactly one non-empty agent_name')

    evaluation = config.get('evaluation')
    if not isinstance(evaluation, dict):
        raise ValueError(f'{config_path}: scenario.evaluation must be a mapping')
    for key in ('timeout', 'goal_tolerance', 'personal_space'):
        try:
            evaluation[key] = float(evaluation[key])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f'{config_path}: evaluation.{key} must be a number') from error
        if not math.isfinite(evaluation[key]) or evaluation[key] <= 0.0:
            raise ValueError(f'{config_path}: evaluation.{key} must be positive and finite')
    return config


def _load_agents(path, intended_user):
    document = yaml.safe_load(Path(path).read_text())
    try:
        parameters = document['hunav_loader']['ros__parameters']
        names = parameters['agents']
    except (KeyError, TypeError) as error:
        raise ValueError(f'{path}: expected hunav_loader.ros__parameters.agents') from error
    if not isinstance(names, list) or not all(isinstance(name, str) and name for name in names):
        raise ValueError(f'{path}: agents must be a list of non-empty names')
    if len(names) != len(set(names)):
        raise ValueError(f'{path}: agent names must be unique')
    ids = []
    for name in names:
        agent = parameters.get(name)
        if not isinstance(agent, dict):
            raise ValueError(f'{path}: missing definition for agent {name!r}')
        identifier = agent.get('id')
        if isinstance(identifier, bool) or not isinstance(identifier, int):
            raise ValueError(f'{path}: {name}.id must be an integer')
        ids.append(identifier)
        goals = agent.get('goals')
        if not isinstance(goals, list) or not goals or len(goals) != len(set(goals)):
            raise ValueError(f'{path}: {name}.goals must be a non-empty list of unique names')
        init_pose = agent.get('init_pose')
        if not isinstance(init_pose, dict):
            raise ValueError(f'{path}: {name}.init_pose must be a mapping')
        _finite_numbers([init_pose.get(key) for key in ('x', 'y', 'z', 'h')], 4,
                        f'{path}: {name}.init_pose')
        for goal_name in goals:
            if not isinstance(goal_name, str) or not isinstance(agent.get(goal_name), dict):
                raise ValueError(f'{path}: {name} references missing goal {goal_name!r}')
            _finite_numbers([agent[goal_name].get(key) for key in ('x', 'y', 'h')], 3,
                            f'{path}: {name}.{goal_name}')
        try:
            configuration = int(agent['behavior']['configuration'])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f'{path}: {name}.behavior.configuration is missing') from error
        if configuration not in (0, 1, 2, 3):
            raise ValueError(f'{path}: {name}.behavior.configuration must be 0, 1, 2, or 3')
    if len(ids) != len(set(ids)):
        raise ValueError(f'{path}: agent IDs must be unique')
    intended_name = intended_user['agent_name']
    if intended_user['enabled'] and intended_name not in names:
        raise ValueError(f'{path}: intended user {intended_name!r} is not in agents')
    return document


def _materialize_random_agents(document, seed, cfg):
    """Replace HuNav's random_device modes with reproducible custom parameters."""
    result = copy.deepcopy(document)
    parameters = result['hunav_loader']['ros__parameters']
    rng = random.Random(seed)
    for name in parameters['agents']:
        behavior = parameters[name]['behavior']
        configuration = int(behavior['configuration'])
        if configuration not in (2, 3):
            continue
        behavior_type = int(behavior['type'])
        if configuration == 2:
            behavior['goal_force_factor'] = max(cfg['simulation']['random_agents']['min_force'], rng.gauss(*cfg['simulation']['random_agents']['normal_goal_force']))
            behavior['obstacle_force_factor'] = max(cfg['simulation']['random_agents']['min_force'], rng.gauss(*cfg['simulation']['random_agents']['normal_obstacle_force']))
            behavior['social_force_factor'] = max(cfg['simulation']['random_agents']['min_social_force'], rng.gauss(*cfg['simulation']['random_agents']['normal_social_force']))
            duration = rng.gauss(*cfg['simulation']['random_agents']['normal_duration'])
            velocity = max(cfg['simulation']['random_agents']['min_velocity'], rng.gauss(*cfg['simulation']['random_agents']['normal_velocity']))
            detection_distance = max(cfg['simulation']['random_agents']['min_detection_distance'], rng.gauss(*cfg['simulation']['random_agents']['normal_detection_distance']))
            if behavior_type in (3, 4, 5, 6):
                behavior['duration'] = duration
            if behavior_type in (4, 5):
                behavior['vel'] = velocity
            if behavior_type == 5:
                behavior['dist'] = rng.gauss(*cfg['simulation']['random_agents']['normal_type5_distance'])
            elif behavior_type in (3, 4):
                behavior['dist'] = detection_distance
            elif behavior_type == 6:
                behavior['dist'] = rng.gauss(*cfg['simulation']['random_agents']['normal_type6_distance'])
            if behavior_type == 4:
                behavior['other_force_factor'] = rng.gauss(*cfg['simulation']['random_agents']['normal_other_force'])
        else:
            behavior['goal_force_factor'] = rng.uniform(*cfg['simulation']['random_agents']['uniform_goal_force'])
            behavior['obstacle_force_factor'] = rng.uniform(*cfg['simulation']['random_agents']['uniform_obstacle_force'])
            behavior['social_force_factor'] = rng.uniform(*cfg['simulation']['random_agents']['uniform_social_force'])
            duration = rng.uniform(*cfg['simulation']['random_agents']['uniform_duration'])
            velocity = rng.uniform(*cfg['simulation']['random_agents']['uniform_velocity'])
            detection_distance = rng.uniform(*cfg['simulation']['random_agents']['uniform_detection_distance'])
            if behavior_type in (3, 4, 5, 6):
                behavior['duration'] = duration
            if behavior_type in (4, 5):
                behavior['vel'] = velocity
            if behavior_type == 5:
                behavior['dist'] = rng.uniform(*cfg['simulation']['random_agents']['uniform_type5_distance'])
            elif behavior_type in (3, 4):
                behavior['dist'] = detection_distance
            elif behavior_type == 6:
                behavior['dist'] = rng.uniform(*cfg['simulation']['random_agents']['uniform_type6_distance'])
            if behavior_type == 4:
                behavior['other_force_factor'] = rng.uniform(*cfg['simulation']['random_agents']['uniform_other_force'])
        behavior['configuration'] = 1
    return result


def _validate_obvious_static_obstacles(world, points, source):
    """Reject points inside explicit, unrotated static SDF boxes; skip model:// includes."""
    for model in world.findall('model'):
        if (model.findtext('static') or '').strip().lower() not in ('1', 'true'):
            continue
        model_pose = [float(value) for value in (model.findtext('pose') or '0 0 0 0 0 0').split()]
        if any(abs(value) > 1e-9 for value in model_pose[3:]):
            continue
        for link in model.findall('link'):
            link_pose = [float(value) for value in (link.findtext('pose') or '0 0 0 0 0 0').split()]
            for collision in link.findall('collision'):
                box = collision.find('geometry/box/size')
                if box is None or not box.text:
                    continue
                collision_pose = [float(value) for value in
                                  (collision.findtext('pose') or '0 0 0 0 0 0').split()]
                if any(abs(value) > 1e-9 for value in link_pose[3:] + collision_pose[3:]):
                    continue
                center = [model_pose[0] + link_pose[0] + collision_pose[0],
                          model_pose[1] + link_pose[1] + collision_pose[1]]
                size = [float(value) for value in box.text.split()]
                for label, point in points:
                    if (abs(point[0] - center[0]) < size[0]/2 and
                            abs(point[1] - center[1]) < size[1]/2):
                        raise ValueError(
                            f'{source}: robot {label} {point} lies inside static model '
                            f'{model.get("name")!r}')


def transform(origin):
    if origin is None:
        return np.eye(4)
    xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
    r, p, y = [float(v) for v in origin.get('rpy', '0 0 0').split()]
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    t = np.eye(4)
    t[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                 [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    t[:3, 3] = xyz
    return t


def pose(t):
    p = math.atan2(-t[2, 0], math.hypot(t[0, 0], t[1, 0]))
    r, y = math.atan2(t[2, 1], t[2, 2]), math.atan2(t[1, 0], t[0, 0])
    return ' '.join(f'{v:.10g}' for v in [*t[:3, 3], r, p, y])


def add(parent, tag, text=None, **attrs):
    e = ET.SubElement(parent, tag, attrs)
    if text is not None:
        e.text = str(text)
    return e


def mesh_file(uri, description):
    prefix = 'package://iwalk_description/'
    if not uri.startswith(prefix):
        raise ValueError(f'Unexpected mesh URI: {uri}')
    return description / uri[len(prefix):]


def geometry_points(g, description):
    if g.tag == 'mesh':
        data = mesh_file(g.get('filename'), description).read_bytes()
        n = struct.unpack_from('<I', data, 80)[0]
        if len(data) != 84 + 50*n:
            raise ValueError('Expected binary STL; refusing to guess footprint')
        pts = [struct.unpack_from('<fff', data, 84+50*i+12+12*j) for i in range(n) for j in range(3)]
        return np.array(pts) * np.array([float(v) for v in g.get('scale', '1 1 1').split()])
    if g.tag == 'box':
        half = np.array([float(v) for v in g.get('size').split()])/2
    elif g.tag == 'cylinder':
        half = np.array([float(g.get('radius')), float(g.get('radius')), float(g.get('length'))/2])
    else:
        raise ValueError(f'Unsupported geometry: {g.tag}')
    return np.array([[x,y,z] for x in [-half[0],half[0]] for y in [-half[1],half[1]] for z in [-half[2],half[2]]])


def sdf_geometry(parent, g, description):
    out = add(add(parent, 'geometry'), g.tag)
    if g.tag == 'mesh':
        add(out, 'uri', mesh_file(g.get('filename'), description).as_uri())
        add(out, 'scale', g.get('scale', '1 1 1'))
    else:
        for k,v in g.attrib.items():
            add(out, k, v)


def ros_params(name, parameters):
    return {name: {'ros__parameters': parameters}}


def obstacle_layer(scan_topic=None, *, cfg):
    scan_topic = cfg['experiment']['topics']['scan'] if scan_topic is None else scan_topic
    return {
        'plugin': cfg['costmap_obstacles']['plugin'], 'enabled': cfg['costmap_obstacles']['enabled'],
        'observation_sources': cfg['costmap_obstacles']['observation_sources'],
        'scan': {'topic': scan_topic, 'data_type': cfg['costmap_obstacles']['scan']['data_type'], 'clearing': cfg['costmap_obstacles']['scan']['clearing'],
                 'marking': cfg['costmap_obstacles']['scan']['marking'], 'max_obstacle_height': cfg['costmap_obstacles']['scan']['max_obstacle_height'],
                 'raytrace_max_range': cfg['costmap_obstacles']['scan']['raytrace_max_range'], 'obstacle_max_range': cfg['costmap_obstacles']['scan']['obstacle_max_range'],
                 'inf_is_valid': cfg['costmap_obstacles']['scan']['inf_is_valid']}}


def costmap_parameters(footprint, semantic=False, track_unknown=False, *, cfg):
    plugins = ['obstacle_layer']
    if semantic:
        plugins += ['human_static_layer', 'human_visibility_layer']
    plugins += ['inflation_layer']
    result = {
        'use_sim_time': cfg['runtime']['use_sim_time'], 'global_frame': cfg['experiment']['frames']['global'], 'robot_base_frame': cfg['experiment']['frames']['base'],
        # Humble's Costmap2DROS declares width/height as integer parameters.
        'rolling_window': cfg['costmap']['rolling_window'], 'width': cfg['costmap']['width'], 'height': cfg['costmap']['height'], 'resolution': cfg['costmap']['resolution'],
        'update_frequency': cfg['costmap']['update_frequency'], 'publish_frequency': cfg['costmap']['publish_frequency'],
        'always_send_full_costmap': cfg['costmap']['always_send_full_costmap'], 'track_unknown_space': track_unknown,
        'transform_tolerance': cfg['costmap']['transform_tolerance'], 'footprint': str(footprint),
        'footprint_padding': cfg['experiment']['geometry']['footprint_padding'], 'plugins': plugins,
        'obstacle_layer': obstacle_layer(cfg=cfg),
        'inflation_layer': {'plugin': cfg['costmap']['inflation_layer']['plugin'],
                            'inflation_radius': cfg['costmap']['inflation_layer']['inflation_radius'], 'cost_scaling_factor': cfg['costmap']['inflation_layer']['cost_scaling_factor']}}
    if semantic:
        result.update({
            'tracked_agents_topic': cfg['experiment']['topics']['tracked_agents_controller'],
            'agents_states_topic': cfg['experiment']['topics']['agents_info'],
            'human_static_layer': {'plugin': cfg['costmap']['human_static_layer']['plugin'],
                                   'agent_radius': cfg['experiment']['human_model']['radius'], 'amplitude': cfg['costmap']['human_static_layer']['amplitude']},
            'human_visibility_layer': {'plugin': cfg['costmap']['human_visibility_layer']['plugin'],
                                       'agent_radius': cfg['experiment']['human_model']['radius'], 'amplitude': cfg['costmap']['human_visibility_layer']['amplitude']}})
    return result


def common_controller(limits, goal_tolerance, *, cfg):
    return {
        'use_sim_time': cfg['runtime']['use_sim_time'], 'controller_frequency': cfg['controller_common']['controller_frequency'],
        'min_x_velocity_threshold': cfg['controller_common']['min_x_velocity_threshold'], 'min_y_velocity_threshold': cfg['controller_common']['min_y_velocity_threshold'],
        'min_theta_velocity_threshold': cfg['controller_common']['min_theta_velocity_threshold'], 'failure_tolerance': cfg['controller_common']['failure_tolerance'],
        'progress_checker_plugin': cfg['controller_common']['progress_checker_plugin'],
        'goal_checker_plugins': cfg['controller_common']['goal_checker_plugins'],
        'controller_plugins': cfg['controller_common']['controller_plugins'],
        'progress_checker': {'plugin': cfg['controller_common']['progress_checker']['plugin'],
                             'required_movement_radius': cfg['controller_common']['progress_checker']['required_movement_radius'],
                             'movement_time_allowance': cfg['controller_common']['progress_checker']['movement_time_allowance']},
        'general_goal_checker': {'plugin': cfg['controller_common']['general_goal_checker']['plugin'],
                                 'stateful': cfg['controller_common']['general_goal_checker']['stateful'], 'xy_goal_tolerance': goal_tolerance,
                                 'yaw_goal_tolerance': cfg['experiment']['evaluation_defaults']['yaw_goal_tolerance']},
        'odom_topic': cfg['experiment']['topics']['odom']}


def dwb_plugin(limits, goal_tolerance, *, cfg):
    return {
        'plugin': cfg['dwb']['plugin'], 'debug_trajectory_details': cfg['dwb']['debug_trajectory_details'],
        'min_vel_x': cfg['dwb']['min_vel_x'], 'min_vel_y': cfg['dwb']['min_vel_y'],
        'max_vel_x': limits['max_linear'], 'max_vel_y': cfg['dwb']['max_vel_y'],
        'max_vel_theta': limits['max_angular'], 'min_speed_xy': cfg['dwb']['min_speed_xy'],
        'max_speed_xy': limits['max_linear'], 'min_speed_theta': cfg['dwb']['min_speed_theta'],
        'acc_lim_x': limits['acceleration'], 'acc_lim_y': cfg['dwb']['acc_lim_y'],
        'acc_lim_theta': limits['angular_acceleration'],
        'decel_lim_x': -limits['deceleration'], 'decel_lim_y': cfg['dwb']['decel_lim_y'],
        'decel_lim_theta': -limits['angular_deceleration'],
        'vx_samples': cfg['dwb']['vx_samples'], 'vy_samples': cfg['dwb']['vy_samples'], 'vtheta_samples': cfg['dwb']['vtheta_samples'],
        'sim_time': cfg['dwb']['sim_time'], 'linear_granularity': cfg['dwb']['linear_granularity'],
        'angular_granularity': cfg['dwb']['angular_granularity'], 'transform_tolerance': cfg['dwb']['transform_tolerance'],
        'xy_goal_tolerance': goal_tolerance, 'trans_stopped_velocity': cfg['dwb']['trans_stopped_velocity'],
        'short_circuit_trajectory_evaluation': cfg['dwb']['short_circuit_trajectory_evaluation'], 'stateful': cfg['dwb']['stateful'],
        'critics': cfg['dwb']['critics'],
        'BaseObstacle.scale': cfg['dwb']['BaseObstacle.scale'], 'PathAlign.scale': cfg['dwb']['PathAlign.scale'],
        'PathAlign.forward_point_distance': cfg['dwb']['PathAlign.forward_point_distance'], 'GoalAlign.scale': cfg['dwb']['GoalAlign.scale'],
        'GoalAlign.forward_point_distance': cfg['dwb']['GoalAlign.forward_point_distance'], 'PathDist.scale': cfg['dwb']['PathDist.scale'],
        'GoalDist.scale': cfg['dwb']['GoalDist.scale'], 'RotateToGoal.scale': cfg['dwb']['RotateToGoal.scale'],
        'RotateToGoal.slowing_factor': cfg['dwb']['RotateToGoal.slowing_factor'], 'RotateToGoal.lookahead_time': cfg['dwb']['RotateToGoal.lookahead_time']}


def hateb_plugin(limits, footprint, semantic, goal_tolerance, personal_space, *, cfg):
    return {
        'plugin': cfg['hateb']['plugin'],
        'predict_srv_name': cfg['hateb']['predict_srv_name'],
        'reset_prediction_srv_name': cfg['hateb']['reset_prediction_srv_name'],
        'pose_prediction_reset_time': cfg['hateb']['pose_prediction_reset_time'], 'odom_topic': cfg['experiment']['topics']['odom'],
        'map_frame': cfg['experiment']['frames']['global'], 'global_frame': cfg['experiment']['frames']['global'], 'base_frame': cfg['experiment']['frames']['base'],
        'footprint_frame': cfg['experiment']['frames']['base'], 'planning_mode': cfg['hateb']['planning_mode']['when_true'] if semantic else cfg['hateb']['planning_mode']['when_false'],
        'footprint_model': {'type': cfg['hateb']['footprint_model']['type'], 'vertices': str(footprint)},
        'robot': {'max_vel_y': cfg['hateb']['robot']['max_vel_y'], 'acc_lim_y': cfg['hateb']['robot']['acc_lim_y'],
                  'max_vel_x': limits['max_linear'], 'min_vel_x': cfg['hateb']['robot']['min_vel_x'],
                  'max_vel_x_backwards': cfg['hateb']['robot']['max_vel_x_backwards'], 'max_vel_theta': limits['max_angular'],
                  'min_vel_theta': cfg['hateb']['robot']['min_vel_theta'], 'acc_lim_x': limits['acceleration'],
                  'acc_lim_theta': limits['angular_acceleration'],
                  'min_turning_radius': cfg['hateb']['robot']['min_turning_radius']},
        'agent': {'agent_radius': cfg['experiment']['human_model']['radius'], 'max_agent_vel_x': cfg['hateb']['agent']['max_agent_vel_x'],
                  'max_agent_vel_y': cfg['hateb']['agent']['max_agent_vel_y'], 'max_agent_vel_x_backwards': cfg['hateb']['agent']['max_agent_vel_x_backwards'],
                  'max_agent_vel_theta': cfg['hateb']['agent']['max_agent_vel_theta'], 'agent_acc_lim_x': cfg['hateb']['agent']['agent_acc_lim_x'],
                  'agent_acc_lim_y': cfg['hateb']['agent']['agent_acc_lim_y'], 'agent_acc_lim_theta': cfg['hateb']['agent']['agent_acc_lim_theta']},
        'trajectory': {'teb_autosize': cfg['hateb']['trajectory']['teb_autosize'], 'dt_ref': cfg['hateb']['trajectory']['dt_ref'], 'dt_hysteresis': cfg['hateb']['trajectory']['dt_hysteresis'],
                       'global_plan_overwrite_orientation': cfg['hateb']['trajectory']['global_plan_overwrite_orientation'],
                       'allow_init_with_backwards_motion': cfg['hateb']['trajectory']['allow_init_with_backwards_motion'],
                       'max_global_plan_lookahead_dist': cfg['hateb']['trajectory']['max_global_plan_lookahead_dist'],
                       'feasibility_check_no_poses': cfg['hateb']['trajectory']['feasibility_check_no_poses'],
                       'global_plan_viapoint_sep': cfg['hateb']['trajectory']['global_plan_viapoint_sep'], 'shrink_horizon_backup': cfg['hateb']['trajectory']['shrink_horizon_backup']},
        'hateb': {'use_agent_agent_safety_c': semantic,
                  'use_agent_robot_safety_c': semantic,
                  'use_agent_robot_rel_vel_c': semantic,
                  'use_agent_robot_visi_c': semantic,
                  'add_invisible_humans': cfg['hateb']['hateb']['add_invisible_humans'],
                  'min_agent_agent_dist': cfg['hateb']['hateb']['min_agent_agent_dist'], 'min_agent_robot_dist': personal_space,
                  'rel_vel_cost_threshold': cfg['hateb']['hateb']['rel_vel_cost_threshold'], 'visibility_cost_threshold': cfg['hateb']['hateb']['visibility_cost_threshold'],
                  'invisible_human_threshold': cfg['hateb']['hateb']['invisible_human_threshold'], 'prediction_time_horizon': cfg['hateb']['hateb']['prediction_time_horizon']},
        'goal': {'xy_goal_tolerance': goal_tolerance, 'yaw_goal_tolerance': cfg['experiment']['evaluation_defaults']['yaw_goal_tolerance'],
                 'free_goal_vel': cfg['hateb']['goal']['free_goal_vel']},
        'obstacles': {'min_obstacle_dist': cfg['hateb']['obstacles']['min_obstacle_dist'], 'include_costmap_obstacles': cfg['hateb']['obstacles']['include_costmap_obstacles'],
                      'costmap_obstacles_behind_robot_dist': cfg['hateb']['obstacles']['costmap_obstacles_behind_robot_dist'],
                      'obstacle_poses_affected': cfg['hateb']['obstacles']['obstacle_poses_affected'], 'costmap_converter_plugin': cfg['hateb']['obstacles']['costmap_converter_plugin'],
                      'costmap_converter_spin_thread': cfg['hateb']['obstacles']['costmap_converter_spin_thread'],
                      'costmap_converter_rate': cfg['hateb']['obstacles']['costmap_converter_rate'], 'obstacle_cost_mult': cfg['hateb']['obstacles']['obstacle_cost_mult'],
                      'use_nonlinear_obstacle_penalty': cfg['hateb']['obstacles']['use_nonlinear_obstacle_penalty']},
        'optim': {'no_inner_iterations': cfg['hateb']['optim']['no_inner_iterations'], 'no_outer_iterations': cfg['hateb']['optim']['no_outer_iterations'],
                  'optimization_activate': cfg['hateb']['optim']['optimization_activate'], 'optimization_verbose': cfg['hateb']['optim']['optimization_verbose'],
                  'penalty_epsilon': cfg['hateb']['optim']['penalty_epsilon'], 'weight_max_vel_x': cfg['hateb']['optim']['weight_max_vel_x'],
                  'weight_max_vel_y': cfg['hateb']['optim']['weight_max_vel_y'], 'weight_max_agent_vel_x': cfg['hateb']['optim']['weight_max_agent_vel_x'],
                  'weight_max_agent_vel_y': cfg['hateb']['optim']['weight_max_agent_vel_y'], 'weight_nominal_agent_vel_x': cfg['hateb']['optim']['weight_nominal_agent_vel_x'],
                  'weight_max_vel_theta': cfg['hateb']['optim']['weight_max_vel_theta'], 'weight_max_agent_vel_theta': cfg['hateb']['optim']['weight_max_agent_vel_theta'],
                  'weight_acc_lim_x': cfg['hateb']['optim']['weight_acc_lim_x'], 'weight_acc_lim_y': cfg['hateb']['optim']['weight_acc_lim_y'],
                  'weight_agent_acc_lim_x': cfg['hateb']['optim']['weight_agent_acc_lim_x'], 'weight_agent_acc_lim_y': cfg['hateb']['optim']['weight_agent_acc_lim_y'],
                  'weight_acc_lim_theta': cfg['hateb']['optim']['weight_acc_lim_theta'], 'weight_agent_acc_lim_theta': cfg['hateb']['optim']['weight_agent_acc_lim_theta'],
                  'weight_kinematics_nh': cfg['hateb']['optim']['weight_kinematics_nh'],
                  'weight_kinematics_forward_drive': cfg['hateb']['optim']['weight_kinematics_forward_drive'],
                  'weight_kinematics_turning_radius': cfg['hateb']['optim']['weight_kinematics_turning_radius'],
                  'weight_optimaltime': cfg['hateb']['optim']['weight_optimaltime'], 'weight_agent_optimaltime': cfg['hateb']['optim']['weight_agent_optimaltime'],
                  'weight_obstacle': cfg['hateb']['optim']['weight_obstacle'], 'weight_dynamic_obstacle': cfg['hateb']['optim']['weight_dynamic_obstacle'],
                  'weight_agent_viapoint': cfg['hateb']['optim']['weight_agent_viapoint'], 'weight_viapoint': cfg['hateb']['optim']['weight_viapoint'],
                  'weight_shortest_path': cfg['hateb']['optim']['weight_shortest_path'],
                  'selection_alternative_time_cost': cfg['hateb']['optim']['selection_alternative_time_cost'],
                  'cap_optimaltime_penalty': cfg['hateb']['optim']['cap_optimaltime_penalty'],
                  'weight_agent_robot_safety': cfg['hateb']['optim']['weight_agent_robot_safety']['when_true'] if semantic else cfg['hateb']['optim']['weight_agent_robot_safety']['when_false'],
                  'weight_agent_agent_safety': cfg['hateb']['optim']['weight_agent_agent_safety']['when_true'] if semantic else cfg['hateb']['optim']['weight_agent_agent_safety']['when_false'],
                  'weight_agent_robot_rel_vel': cfg['hateb']['optim']['weight_agent_robot_rel_vel']['when_true'] if semantic else cfg['hateb']['optim']['weight_agent_robot_rel_vel']['when_false'],
                  'weight_agent_robot_visibility': cfg['hateb']['optim']['weight_agent_robot_visibility']['when_true'] if semantic else cfg['hateb']['optim']['weight_agent_robot_visibility']['when_false'],
                  'weight_invisible_human': cfg['hateb']['optim']['weight_invisible_human'], 'disable_warm_start': cfg['hateb']['optim']['disable_warm_start']},
        'visualization': {'publish_agents_global_plans': semantic,
                          'publish_agents_local_plan_fp_poses': semantic,
                          'publish_agents_local_plan_poses': semantic,
                          'publish_agents_local_plans': semantic,
                          'publish_robot_global_plan': cfg['hateb']['visualization']['publish_robot_global_plan'],
                          'publish_robot_local_plan': cfg['hateb']['visualization']['publish_robot_local_plan'],
                          'publish_robot_local_plan_fp_poses': cfg['hateb']['visualization']['publish_robot_local_plan_fp_poses'],
                          'publish_robot_local_plan_poses': cfg['hateb']['visualization']['publish_robot_local_plan_poses']}}


def robot_frames(robot, root):
    """Resolve the description without changing its links or joints."""
    names = {link.get('name') for link in robot.findall('link')}
    children = {joint.find('child').get('link') for joint in robot.findall('joint')}
    if names - children != {root}:
        raise ValueError(f'Expected a single URDF root {root!r}')
    frames = {root: np.eye(4)}
    pending = list(robot.findall('joint'))
    while pending:
        ready = [j for j in pending if j.find('parent').get('link') in frames]
        if not ready:
            raise ValueError('URDF contains a cycle or disconnected joint')
        for joint in ready:
            child = joint.find('child').get('link')
            if child in frames or child not in names:
                raise ValueError(f'Invalid or duplicate joint child {child!r}')
            frames[child] = frames[joint.find('parent').get('link')] @ transform(joint.find('origin'))
            pending.remove(joint)
    if set(frames) != names:
        raise ValueError('URDF contains disconnected links')
    return frames


def append_description_geometry(link, robot, frames, description, laser_frame):
    """Copy physical visual/collision shapes into the rigid simulator chassis."""
    bounds = []
    for urdf_link in robot.findall('link'):
        name = urdf_link.get('name')
        for index, element in enumerate([*urdf_link.findall('visual'), *urdf_link.findall('collision')]):
            tf = frames[name] @ transform(element.find('origin'))
            geometry = list(element.find('geometry'))[0]
            if name != laser_frame:
                points = geometry_points(geometry, description)
                bounds.extend((points @ tf[:3, :3].T + tf[:3, 3]).tolist())
            copied = add(link, element.tag, name=f'{name}_{element.tag}_{index}')
            add(copied, 'pose', pose(tf))
            sdf_geometry(copied, geometry, description)
            if element.tag == 'visual':
                material = add(copied, 'material')
                add(material, 'ambient', '0.4 0.4 0.45 1')
                add(material, 'diffuse', '0.4 0.4 0.45 1')
    return np.asarray(bounds)


def prepare(description, wrapper, target, experiment_config=None, headless=None,
            scenario=None, scenario_config=None, seed=None, controller=None,
            require_intended_user_support=False):
    description, wrapper, target = Path(description), Path(wrapper), Path(target)
    config_path = (Path(experiment_config) if experiment_config else
                   Path(__file__).resolve().parents[1]/'config/experiment.yaml')
    cfg = load_configuration(config_path)
    runtime = cfg['runtime']
    scenario = runtime['scenario'] if scenario is None else scenario
    seed = runtime['seed'] if seed is None else seed
    controller = runtime['controller'] if controller is None else controller
    headless = runtime['headless'] if headless is None else headless
    try:
        seed = int(seed)
    except (TypeError, ValueError) as error:
        raise ValueError('seed must be an integer') from error
    if seed < 0:
        raise ValueError('seed must be non-negative')
    package_root = config_path.resolve().parent.parent
    scenario_data = _load_scenario(package_root, scenario, scenario_config, cfg['experiment'])
    agents_document = _load_agents(
        scenario_data['_agents_path'], scenario_data['intended_user'])
    if require_intended_user_support and scenario_data['intended_user']['enabled']:
        raise ValueError(
            f'Scenario {scenario_data["name"]!r} requires intended-user tracking/front-following; '
            'this project currently provides local-planner evaluation only')
    runtime_agents = _materialize_random_agents(agents_document, seed, cfg)
    evaluation = scenario_data['evaluation']
    exp = cfg['experiment']
    navigation_envelope = exp['dwal']
    target.mkdir(parents=True, exist_ok=True)
    robot = ET.fromstring(xacro.process_file(str(description/'urdf/iwalk.urdf.xacro')).toxml())
    frames = robot_frames(robot, exp['frames']['base'])
    laser_tf = frames[exp['frames']['laser']]
    laser_x, laser_y, laser_z, laser_roll, laser_pitch, laser_yaw = map(float, pose(laser_tf).split())
    if abs(laser_roll) > 1e-9 or abs(laser_pitch) > 1e-9:
        raise ValueError('The 2D navigation stack requires a horizontal LiDAR')
    height = float(frames[exp['frames']['robot_description_base']][2, 3])
    ET.ElementTree(robot).write(target/'robot.urdf', encoding='unicode')
    tree = ET.parse(scenario_data['_world_path'])
    world = tree.getroot().find('world')
    if world is None:
        raise ValueError(f'{scenario_data["_world_path"]}: missing SDF world element')
    points = [('start', scenario_data['robot']['initial_pose'][:2])]
    points.extend((f'path point {index}', point)
                  for index, point in enumerate(scenario_data['robot']['path_xy']))
    _validate_obvious_static_obstacles(world, points, scenario_data['_world_path'])
    if world.find("model[@name=%r]" % cfg['simulation']['robot_name']) is not None:
        raise ValueError(f'{scenario_data["_world_path"]}: model name iwalk is reserved')
    model = add(world, 'model', name=cfg['simulation']['robot_name'])
    initial_x, initial_y, initial_yaw = scenario_data['robot']['initial_pose']
    add(model, 'pose', f'{initial_x} {initial_y} 0 0 0 {initial_yaw}')
    link = add(model, 'link', name='chassis')
    add(link, 'gravity', 'false')
    inertial = add(link, 'inertial'); add(inertial, 'mass', cfg['simulation']['inertial']['mass'])
    inertia = add(inertial, 'inertia')
    for k,v in {k: v for k, v in cfg['simulation']['inertial'].items() if k != 'mass'}.items(): add(inertia,k,v)
    pts = append_description_geometry(link, robot, frames, description, exp['frames']['laser'])
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    xmin,ymin = np.floor(lo[:2]*1000)/1000
    xmax,ymax = np.ceil(hi[:2]*1000)/1000
    # Preserve the common conservative navigation envelope used by the fixed baseline.
    xmin = min(xmin, -float(navigation_envelope['rear_extent']))
    xmax = max(xmax, float(navigation_envelope['front_extent']))
    footprint = [[float(xmax),float(ymax)],[float(xmax),float(ymin)], [float(xmin),float(ymin)],[float(xmin),float(ymax)]]
    # The conservative footprint is planner data, never a physical collision box.
    sensor = add(link, 'sensor', name='lidar', type='ray' if headless else 'gpu_ray')
    add(sensor, 'pose', pose(laser_tf)); add(sensor,'always_on','true'); add(sensor,'update_rate',cfg['simulation']['lidar']['rate'])
    ray=add(sensor,'ray'); scan=add(ray,'scan'); h=add(scan,'horizontal')
    for k,v in {k: cfg['simulation']['lidar'][k] for k in ('samples', 'resolution', 'min_angle', 'max_angle')}.items(): add(h,k,v)
    ran=add(ray,'range')
    for k,v in {'min':cfg['simulation']['lidar']['range_min'], 'max':cfg['simulation']['lidar']['range_max'], 'resolution':cfg['simulation']['lidar']['range_resolution']}.items(): add(ran,k,v)
    sp=add(sensor,'plugin',name='lidar_ros',filename='libgazebo_ros_ray_sensor.so')
    ros=add(sp,'ros'); add(ros,'remapping','~/out:=' + exp['topics']['scan_raw'])
    add(sp,'output_type','sensor_msgs/LaserScan'); add(sp,'frame_name',exp['frames']['laser'])
    plugin=add(model,'plugin',name='ideal_planar_base',filename='libgazebo_ros_planar_move.so')
    for k,v in {'update_rate':cfg['simulation']['planar_base']['update_rate'],'publish_rate':cfg['simulation']['planar_base']['publish_rate'],'odometry_frame':exp['frames']['global'],'robot_base_frame':exp['frames']['base'], 'publish_odom':cfg['simulation']['planar_base']['publish_odom'],'publish_odom_tf':cfg['simulation']['planar_base']['publish_odom_tf']}.items(): add(plugin,k,v)
    tree.write(target/'scenario.world', encoding='unicode')
    limits, timeouts = exp['limits'], exp['timeouts']
    dwal_config, dynamic_config = exp['dwal'], exp['dynamic_dwal']
    # The configured conservative envelope also expands the URDF-derived bounds.
    if abs(float(dwal_config['front_extent']) - xmax) > 0.03 or abs(float(dwal_config['rear_extent']) + xmin) > 0.03:
        raise ValueError(f'experiment.yaml iWalk extents disagree: configured front={dwal_config["front_extent"]}, rear={dwal_config["rear_extent"]}; derived front={xmax}, rear={-xmin}')
    levels = [float(v) for v in dwal_config['fixed_levels']]
    controller_name = controller
    variable_dwal = controller == 'dynamic_dwal'
    if variable_dwal:
        # One reference level / search ceiling; each Path reports its actual extent.
        levels = [float(dynamic_config['max_search_radius'])]
    if not levels or any(not math.isfinite(r) or r<=0 for r in levels):
        raise ValueError('DWAL levels must be positive and finite')
    common={'common/levels':levels,'common/odom_frame':'odom','use_sim_time':True}
    generator={**common,
        'dwal_generator/odometryTopic':cfg['experiment']['topics']['odom'], 'dwal_generator/occ_topic':cfg['experiment']['topics']['costmap'],
        'dwal_generator/base_frame':cfg['experiment']['frames']['base'],
        'dwal_generator/footprint':[v for point in footprint for v in point],
        'dwal_generator/footprint_padding':cfg['experiment']['geometry']['footprint_padding'],
        'dwal_generator/input_timeout':timeouts['odom'],
        'dwal_generator/acc_lim_x':limits['acceleration'],
        'dwal_generator/acc_lim_th':limits['angular_acceleration'],
        'dwal_generator/max_trans_vel':limits['max_linear'],
        'dwal_generator/min_trans_vel':cfg['generator']['dwal_generator/min_trans_vel'], 'dwal_generator/max_vel_theta':limits['max_angular'],
        'dwal_generator/sim_period':cfg['generator']['dwal_generator/sim_period'],
        'dwal_generator/DS':float(dynamic_config['sweep_resolution']) if variable_dwal else cfg['generator']['dwal_generator/DS']['when_false'],
        'dwal_generator/Kmax':cfg['experiment']['dwal']['max_curvature'], 'dwal_generator/alpha':cfg['generator']['dwal_generator/alpha'], 'dwal_generator/Hz':cfg['generator']['dwal_generator/Hz']}
    clustering={**common,
        'dwal_clustering/variable_length':variable_dwal,
        'dwal_clustering/postfix':cfg['clustering']['dwal_clustering/postfix']['when_true'] if variable_dwal else cfg['clustering']['dwal_clustering/postfix']['when_false'],
        'dwal_clustering/spin':cfg['clustering']['dwal_clustering/spin']['when_true'] if variable_dwal else cfg['clustering']['dwal_clustering/spin']['when_false'],
        'dwal_clustering/min_cluster_span':float(dwal_config['min_cluster_span']),
        'dwal_clustering/cluster_separation':int(dwal_config['cluster_separation']),
        'dwal_clustering/subsample_step':cfg['clustering']['dwal_clustering/subsample_step']}
    if variable_dwal:
        clustering.update({
            'odom_topic': cfg['experiment']['topics']['odom'],
            'odom_timeout': float(timeouts['odom']),
            'a_brake': float(limits['deceleration']),
            'reaction_time': float(dynamic_config['reaction_time']),
            'stop_margin': float(dynamic_config['longitudinal_margin']),
    })
    (target/'dwal.yaml').write_text(dump_yaml({
        '/dwal_planner/dwal_generator':{'ros__parameters':generator},
        '/dwal_planner/dwal_clustering':{'ros__parameters':clustering}}))
    costmap=costmap_parameters(footprint, False, cfg=cfg)
    (target/'costmap.yaml').write_text(dump_yaml({'/costmap/costmap':{'ros__parameters':costmap}}))
    for family in ('dwb', 'hateb'):
        for semantic in (False, True):
            controller = common_controller(limits, evaluation['goal_tolerance'], cfg=cfg)
            if family == 'hateb':
                # HATEB calls setGoalControl() on its concrete goal checker.
                # The pinned implementation does not safely handle Nav2's
                # SimpleGoalChecker here, so use the plugin it ships with.
                controller['general_goal_checker']['plugin'] = \
                    'hateb_local_planner::HATEBGoalChecker'
            controller['FollowPath'] = (
                dwb_plugin(limits, evaluation['goal_tolerance'], cfg=cfg) if family == 'dwb'
                else hateb_plugin(limits, footprint, semantic,
                                  evaluation['goal_tolerance'],
                                  evaluation['personal_space'], cfg=cfg))
            params = {}
            params.update(ros_params('/controller_server', controller))
            params.update(ros_params('/local_costmap/local_costmap',
                                     costmap_parameters(footprint, semantic and family == 'dwb', cfg=cfg)))
            (target/f'{family}_{"on" if semantic else "off"}.yaml').write_text(dump_yaml(params))

    reference = common_controller(limits, evaluation['goal_tolerance'], cfg=cfg)
    reference['FollowPath'] = {
        'plugin': cfg['reference']['plugin'],
        'desired_linear_vel': limits['max_linear'], 'lookahead_dist': cfg['reference']['lookahead_dist'],
        'min_lookahead_dist': cfg['reference']['min_lookahead_dist'], 'max_lookahead_dist': cfg['reference']['max_lookahead_dist'],
        'lookahead_time': cfg['reference']['lookahead_time'], 'rotate_to_heading_angular_vel': cfg['reference']['rotate_to_heading_angular_vel'],
        'transform_tolerance': cfg['reference']['transform_tolerance'], 'use_velocity_scaled_lookahead_dist': cfg['reference']['use_velocity_scaled_lookahead_dist'],
        'min_approach_linear_velocity': cfg['reference']['min_approach_linear_velocity'], 'approach_velocity_scaling_dist': cfg['reference']['approach_velocity_scaling_dist'],
        'use_collision_detection': cfg['reference']['use_collision_detection'], 'max_allowed_time_to_collision_up_to_carrot': cfg['reference']['max_allowed_time_to_collision_up_to_carrot'],
        'use_rotate_to_heading': cfg['reference']['use_rotate_to_heading'], 'allow_reversing': cfg['reference']['allow_reversing'],
        'max_angular_accel': limits['angular_acceleration'], 'rotate_to_heading_min_angle': cfg['reference']['rotate_to_heading_min_angle']}
    reference_params = {}
    reference_params.update(ros_params('/reference/controller_server', reference))
    reference_params.update(ros_params('/reference/local_costmap/local_costmap', costmap_parameters(footprint, False, cfg=cfg)))
    (target/'reference.yaml').write_text(dump_yaml(reference_params))

    smoother = {'use_sim_time': cfg['runtime']['use_sim_time'], 'smoothing_frequency': cfg['smoother']['smoothing_frequency'],
                'scale_velocities': cfg['smoother']['scale_velocities'], 'feedback': cfg['smoother']['feedback'],
                'max_velocity': [limits['max_linear'], 0.0, limits['max_angular']],
                'min_velocity': [0.0, 0.0, -limits['max_angular']],
                'max_accel': [limits['acceleration'], 0.0, limits['angular_acceleration']],
                'max_decel': [-limits['deceleration'], 0.0, -limits['angular_deceleration']],
                'odom_topic': cfg['experiment']['topics']['odom'], 'odom_duration': cfg['smoother']['odom_duration'],
                'deadband_velocity': cfg['smoother']['deadband_velocity'], 'velocity_timeout': cfg['smoother']['velocity_timeout']}
    collision = {'use_sim_time': cfg['runtime']['use_sim_time'], 'base_frame_id': cfg['experiment']['frames']['base'], 'odom_frame_id': cfg['experiment']['frames']['global'],
                 'cmd_vel_in_topic': cfg['experiment']['topics']['smoothed_cmd'], 'cmd_vel_out_topic': cfg['experiment']['topics']['collision_checked_cmd'],
                 'transform_tolerance': cfg['collision']['transform_tolerance'], 'source_timeout': cfg['collision']['source_timeout'],
                 'base_shift_correction': cfg['collision']['base_shift_correction'], 'stop_pub_timeout': cfg['collision']['stop_pub_timeout'],
                 'polygons': cfg['collision']['polygons'],
                 'FootprintApproach': {'type': cfg['collision']['FootprintApproach']['type'], 'action_type': cfg['collision']['FootprintApproach']['action_type'],
                                       'footprint_topic': cfg['experiment']['topics']['published_footprint'],
                                       'time_before_collision': cfg['collision']['FootprintApproach']['time_before_collision'],
                                       'simulation_time_step': cfg['collision']['FootprintApproach']['simulation_time_step'], 'max_points': cfg['collision']['FootprintApproach']['max_points'],
                                       'visualize': cfg['collision']['FootprintApproach']['visualize'], 'enabled': cfg['collision']['FootprintApproach']['enabled']},
                 'observation_sources': cfg['collision']['observation_sources'],
                 'scan': {'type': cfg['collision']['scan']['type'], 'topic': cfg['experiment']['topics']['scan'], 'enabled': cfg['collision']['scan']['enabled']}}
    pipeline = {}
    pipeline.update(ros_params('/velocity_smoother', smoother))
    pipeline.update(ros_params('/collision_monitor', collision))
    pipeline.update(ros_params('/command_guard', {
        'use_sim_time': cfg['runtime']['use_sim_time'], 'input_timeout': timeouts['selected_command'],
        'speed_limit_timeout': timeouts['speed_limit'], 'output_rate': cfg['command_guard']['output_rate'],
        'max_linear': limits['max_linear'], 'max_angular': limits['max_angular'],
        'allow_reverse': cfg['experiment']['limits']['allow_reverse']}))
    pipeline.update(ros_params('/final_cmd_watchdog', {
        'use_sim_time': cfg['runtime']['use_sim_time'], 'input_timeout': timeouts['final_input'], 'output_rate': cfg['final_cmd_watchdog']['output_rate']}))
    (target/'command_pipeline.yaml').write_text(dump_yaml(pipeline))

    context = dict(exp['context'])
    context.update({'use_sim_time': cfg['runtime']['use_sim_time'], 'tracks_topic': cfg['experiment']['topics']['tracked_agents_logging'],
                    'costmap_topic': cfg['experiment']['topics']['costmap'], 'odom_topic': cfg['experiment']['topics']['odom'],
                    'output_topic': cfg['experiment']['topics']['context'], 'output_rate': context.pop('rate')})
    (target/'context.yaml').write_text(dump_yaml(ros_params('/context_estimator', context)))

    context_speed = {
        'use_sim_time': cfg['runtime']['use_sim_time'], 'context_timeout': timeouts['context'],
        'max_speed': limits['max_linear']}
    for profile, values in exp['profiles'].items():
        context_speed[f'profiles.{profile}.max_speed'] = values['max_speed']
    (target/'context_speed_limit.yaml').write_text(
        dump_yaml(ros_params('/context_speed_limit', context_speed)))

    shared_common = {'use_sim_time': cfg['runtime']['use_sim_time'], 'odom_topic': cfg['experiment']['topics']['odom'],
                     'near_cluster_topic': cfg['experiment']['topics']['clusters_near'],
                     'far_cluster_topic': cfg['experiment']['topics']['clusters_far'],
                     'user_cmd_topic': cfg['experiment']['topics']['reference_cmd'], 'output_cmd_topic': cfg['experiment']['topics']['selected_cmd'],
                     'use_stamped_twist_input': cfg['shared_common']['use_stamped_twist_input'], 'use_stamped_twist_output': cfg['shared_common']['use_stamped_twist_output'],
                     'tracked_agents_topic': cfg['experiment']['topics']['tracked_agents_controller'],
                     'context_profile_topic': cfg['experiment']['topics']['context_profile'],
                     'odom_timeout': timeouts['odom'], 'cluster_timeout': timeouts['clusters'],
                     'reference_timeout': timeouts['reference'],
                     'tracks_timeout': timeouts['tracks'],
                     'allow_reverse': cfg['experiment']['limits']['allow_reverse'], 'allow_no_cluster_passthrough': cfg['shared_common']['allow_no_cluster_passthrough'],
                     'personal_space': evaluation['personal_space'], 'social_weight': cfg['shared_common']['social_weight'],
                     'c_max': cfg['shared_common']['c_max'], 'alpha': cfg['shared_common']['alpha'], 's_min': cfg['shared_common']['s_min'],
                     'Kphi': cfg['shared_common']['Kphi'], 'Kmax': cfg['experiment']['dwal']['max_curvature'], 'v_low': cfg['shared_common']['v_low'], 'v_high': cfg['shared_common']['v_high'],
                     'intent.alpha': cfg['shared_common']['intent.alpha'], 'intent.beta': cfg['shared_common']['intent.beta'],
                     'intent.eta': cfg['shared_common']['intent.eta'], 'intent.sigma_min': cfg['shared_common']['intent.sigma_min'],
                     'intent.min_confidence': cfg['shared_common']['intent.min_confidence']}

    shared_common.update({
        'variable_length': variable_dwal,
        'inplace_via_collision_monitor': cfg['shared_common']['inplace_via_collision_monitor'],
        'intent_reference_radius': levels[-1] if variable_dwal else cfg['shared_common']['intent_reference_radius']['when_false'],
        'a_brake': limits['deceleration'],
        'reaction_time': dynamic_config['reaction_time'],
        'stop_margin': dynamic_config['longitudinal_margin'],
        'max_linear': limits['max_linear'], 'max_angular': limits['max_angular'],
        'prediction_horizon': dynamic_config['prediction_horizon'],
        'human_radius': exp['human_model']['radius'],
        'human_uncertainty': dynamic_config['human_base_uncertainty'],
        'human_uncertainty_growth': dynamic_config['human_uncertainty_growth'],
        'footprint': [v for point in footprint for v in point]})
    if variable_dwal:
        shared_common['near_cluster_topic'] = exp['topics']['clusters_far']
    for semantic in (False, True):
        shared = dict(shared_common)
        shared['semantic_mode'] = semantic
        (target/f'shared_control_{"on" if semantic else "off"}.yaml').write_text(
            dump_yaml(ros_params('/shared_controller', shared)))

    path_xy = [value for point in scenario_data['robot']['path_xy'] for value in point]
    task = {'use_sim_time': cfg['runtime']['use_sim_time'], 'path_xy': path_xy,
            'timeout': evaluation['timeout'],
            'odom_topic': cfg['experiment']['topics']['odom'], 'path_topic': cfg['experiment']['topics']['path']}
    (target/'task.yaml').write_text(dump_yaml(ros_params('/local_task', task)))

    (target/'agents.yaml').write_text(dump_yaml(runtime_agents))
    agent_parameters = runtime_agents['hunav_loader']['ros__parameters']
    prediction_goals = []
    for agent_name in agent_parameters['agents']:
        agent = agent_parameters[agent_name]
        for goal_name in agent['goals']:
            goal = agent[goal_name]
            prediction_goals.append({
                'name': f'{agent_name}_{goal_name}',
                'goal': [float(goal['x']), float(goal['y'])]})
    goals_file = target/'agent_goals.yaml'
    goals_file.write_text(dump_yaml({'window_size': cfg['predictor']['window_size'], 'goals': prediction_goals}))
    predictor = {'use_sim_time': cfg['runtime']['use_sim_time'], 'tracked_agents_sub_topic': cfg['experiment']['topics']['tracked_agents_controller'],
                 'robot_frame_id': cfg['experiment']['frames']['base'], 'map_frame_id': cfg['experiment']['frames']['global'],
                 'goals_file': str(goals_file), 'publish_markers': cfg['predictor']['publish_markers']}
    (target/'agent_prediction.yaml').write_text(dump_yaml(ros_params('/agent_path_prediction', predictor)))
    self_filter_padding = cfg['scan_filter']['padding']
    scan_filter = {
        'use_sim_time': cfg['runtime']['use_sim_time'],
        'filter1': {
            'name': cfg['scan_filter']['filter1']['name'],
            'type': cfg['scan_filter']['filter1']['type'],
            'params': {
                'box_frame': cfg['experiment']['frames']['base'],
                'min_x': float(xmin - self_filter_padding),
                'max_x': float(xmax + self_filter_padding),
                'min_y': float(ymin - self_filter_padding),
                'max_y': float(ymax + self_filter_padding),
                'min_z': cfg['scan_filter']['filter1']['params']['min_z'], 'max_z': cfg['scan_filter']['filter1']['params']['max_z'], 'invert': cfg['scan_filter']['filter1']['params']['invert']}}}
    (target/'scan_filter.yaml').write_text(dump_yaml(ros_params('/scan_self_filter', scan_filter)))
    (target/'geometry.yaml').write_text(dump_yaml({
        'footprint':footprint, 'front_extent':float(xmax), 'rear_extent':float(-xmin),
        'laser_xyz':[laser_x,laser_y,laser_z], 'base_link_height':height,
        'description':'URDF-derived conservative visual/collision envelope; shared by all conditions.'}))
    (target/'scenario_manifest.yaml').write_text(dump_yaml({
        'scenario': scenario_data['name'],
        'scenario_config': str(scenario_data['_config_path']),
        'world': scenario_data['world'],
        'agents': scenario_data['agents'],
        'seed': seed,
        'robot': scenario_data['robot'],
        'intended_user': scenario_data['intended_user'],
        'evaluation': evaluation,
        'runtime_world': str(target/'scenario.world'),
        'runtime_agents': str(target/'agents.yaml')}))
    (target/'resolved_config.yaml').write_text(dump_yaml({**cfg, 'selection': {'controller': 'dynamic_dwal' if variable_dwal else controller_name, 'scenario': scenario_data['name'], 'seed': seed, 'headless': headless}}))
    return target


if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('description')
    p.add_argument('wrapper')
    p.add_argument('output')
    p.add_argument('--experiment-config')
    p.add_argument('--scenario')
    p.add_argument('--scenario-config')
    p.add_argument('--seed', type=int)
    p.add_argument('--headless', action=argparse.BooleanOptionalAction, default=None)
    p.add_argument('--controller')
    p.add_argument('--require-intended-user-support', action='store_true')
    a = p.parse_args()
    prepare(a.description, a.wrapper, a.output, a.experiment_config, a.headless,
            a.scenario, a.scenario_config, a.seed, a.controller,
            a.require_intended_user_support)
