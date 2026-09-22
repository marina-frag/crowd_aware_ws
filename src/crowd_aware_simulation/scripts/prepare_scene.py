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


def _load_scenario(package_root, scenario='cafe', scenario_config=None):
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
    config = copy.deepcopy(document['scenario'])
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


def _materialize_random_agents(document, seed):
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
            behavior['goal_force_factor'] = max(0.5, rng.gauss(2.0, 1.5))
            behavior['obstacle_force_factor'] = max(0.5, rng.gauss(10.0, 4.0))
            behavior['social_force_factor'] = max(3.0, rng.gauss(4.0, 3.5))
            duration = rng.gauss(40.0, 15.0)
            velocity = max(0.4, rng.gauss(0.8, 0.35))
            detection_distance = max(1.5, rng.gauss(4.5, 2.5))
            if behavior_type in (3, 4, 5, 6):
                behavior['duration'] = duration
            if behavior_type in (4, 5):
                behavior['vel'] = velocity
            if behavior_type == 5:
                behavior['dist'] = rng.gauss(1.5, 0.3)
            elif behavior_type in (3, 4):
                behavior['dist'] = detection_distance
            elif behavior_type == 6:
                behavior['dist'] = rng.gauss(1.4, 0.3)
            if behavior_type == 4:
                behavior['other_force_factor'] = rng.gauss(20.0, 6.0)
        else:
            behavior['goal_force_factor'] = rng.uniform(2.0, 5.0)
            behavior['obstacle_force_factor'] = rng.uniform(2.0, 50.0)
            behavior['social_force_factor'] = rng.uniform(4.0, 20.0)
            duration = rng.uniform(25.0, 60.0)
            velocity = rng.uniform(0.6, 1.2)
            detection_distance = rng.uniform(2.0, 6.0)
            if behavior_type in (3, 4, 5, 6):
                behavior['duration'] = duration
            if behavior_type in (4, 5):
                behavior['vel'] = velocity
            if behavior_type == 5:
                behavior['dist'] = rng.uniform(1.0, 2.5)
            elif behavior_type in (3, 4):
                behavior['dist'] = detection_distance
            elif behavior_type == 6:
                behavior['dist'] = rng.uniform(0.8, 1.9)
            if behavior_type == 4:
                behavior['other_force_factor'] = rng.uniform(10.0, 25.0)
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


def obstacle_layer(scan_topic='/scan'):
    return {
        'plugin': 'nav2_costmap_2d::ObstacleLayer', 'enabled': True,
        'observation_sources': 'scan',
        'scan': {'topic': scan_topic, 'data_type': 'LaserScan', 'clearing': True,
                 'marking': True, 'max_obstacle_height': 2.0,
                 'raytrace_max_range': 10.0, 'obstacle_max_range': 9.5,
                 'inf_is_valid': True}}


def costmap_parameters(footprint, semantic=False):
    plugins = ['obstacle_layer']
    if semantic:
        plugins += ['human_static_layer', 'human_visibility_layer']
    plugins += ['inflation_layer']
    result = {
        'use_sim_time': True, 'global_frame': 'odom', 'robot_base_frame': 'sim_base',
        # Humble's Costmap2DROS declares width/height as integer parameters.
        'rolling_window': True, 'width': 8, 'height': 8, 'resolution': 0.05,
        'update_frequency': 10.0, 'publish_frequency': 5.0,
        'always_send_full_costmap': True, 'track_unknown_space': False,
        'transform_tolerance': 0.5, 'footprint': str(footprint),
        'footprint_padding': 0.02, 'plugins': plugins,
        'obstacle_layer': obstacle_layer(),
        'inflation_layer': {'plugin': 'nav2_costmap_2d::InflationLayer',
                            'inflation_radius': 0.45, 'cost_scaling_factor': 8.0}}
    if semantic:
        result.update({
            'tracked_agents_topic': '/tracked_agents',
            'agents_states_topic': '/agents_info',
            'human_static_layer': {'plugin': 'cohan_layers::StaticAgentLayer',
                                   'agent_radius': 0.4, 'amplitude': 150.0},
            'human_visibility_layer': {'plugin': 'cohan_layers::AgentVisibilityLayer',
                                       'agent_radius': 0.4, 'amplitude': 180.0}})
    return result


def common_controller(limits, goal_tolerance):
    return {
        'use_sim_time': True, 'controller_frequency': 20.0,
        'min_x_velocity_threshold': 0.001, 'min_y_velocity_threshold': 0.001,
        'min_theta_velocity_threshold': 0.001, 'failure_tolerance': 0.0,
        'progress_checker_plugin': 'progress_checker',
        'goal_checker_plugins': ['general_goal_checker'],
        'controller_plugins': ['FollowPath'],
        'progress_checker': {'plugin': 'nav2_controller::SimpleProgressChecker',
                             'required_movement_radius': 0.08,
                             'movement_time_allowance': 8.0},
        'general_goal_checker': {'plugin': 'nav2_controller::SimpleGoalChecker',
                                 'stateful': True, 'xy_goal_tolerance': goal_tolerance,
                                 'yaw_goal_tolerance': 0.25}}


def dwb_plugin(limits, goal_tolerance):
    return {
        'plugin': 'dwb_core::DWBLocalPlanner', 'debug_trajectory_details': True,
        'min_vel_x': 0.0, 'min_vel_y': 0.0,
        'max_vel_x': limits['max_linear'], 'max_vel_y': 0.0,
        'max_vel_theta': limits['max_angular'], 'min_speed_xy': 0.0,
        'max_speed_xy': limits['max_linear'], 'min_speed_theta': 0.0,
        'acc_lim_x': limits['acceleration'], 'acc_lim_y': 0.0,
        'acc_lim_theta': limits['angular_acceleration'],
        'decel_lim_x': -limits['deceleration'], 'decel_lim_y': 0.0,
        'decel_lim_theta': -limits['angular_deceleration'],
        'vx_samples': 20, 'vy_samples': 1, 'vtheta_samples': 30,
        'sim_time': 2.0, 'linear_granularity': 0.05,
        'angular_granularity': 0.025, 'transform_tolerance': 0.2,
        'xy_goal_tolerance': goal_tolerance, 'trans_stopped_velocity': 0.02,
        'short_circuit_trajectory_evaluation': True, 'stateful': True,
        'critics': ['RotateToGoal', 'Oscillation', 'BaseObstacle', 'GoalAlign',
                    'PathAlign', 'PathDist', 'GoalDist'],
        'BaseObstacle.scale': 0.02, 'PathAlign.scale': 32.0,
        'PathAlign.forward_point_distance': 0.1, 'GoalAlign.scale': 24.0,
        'GoalAlign.forward_point_distance': 0.1, 'PathDist.scale': 32.0,
        'GoalDist.scale': 24.0, 'RotateToGoal.scale': 32.0,
        'RotateToGoal.slowing_factor': 5.0, 'RotateToGoal.lookahead_time': -1.0}


def hateb_plugin(limits, footprint, semantic, goal_tolerance, personal_space):
    return {
        'plugin': 'hateb_local_planner::HATebLocalPlannerROS',
        'predict_srv_name': '/agent_path_prediction/predict_agent_poses',
        'reset_prediction_srv_name': '/agent_path_prediction/reset_prediction_services',
        'pose_prediction_reset_time': 10.0, 'odom_topic': '/odom',
        'map_frame': 'odom', 'global_frame': 'odom', 'base_frame': 'sim_base',
        'footprint_frame': 'sim_base', 'planning_mode': 1 if semantic else 0,
        'footprint_model': {'type': 'polygon', 'vertices': str(footprint)},
        'robot': {'max_vel_y': 0.0, 'acc_lim_y': 0.0,
                  'max_vel_x': limits['max_linear'], 'min_vel_x': 0.0,
                  'max_vel_x_backwards': 0.05, 'max_vel_theta': limits['max_angular'],
                  'min_vel_theta': 0.0, 'acc_lim_x': limits['acceleration'],
                  'acc_lim_theta': limits['angular_acceleration'],
                  'min_turning_radius': 0.0},
        'agent': {'agent_radius': 0.4, 'max_agent_vel_x': 1.5,
                  'max_agent_vel_y': 1.5, 'max_agent_vel_x_backwards': 1.5,
                  'max_agent_vel_theta': 1.2, 'agent_acc_lim_x': 0.8,
                  'agent_acc_lim_y': 0.8, 'agent_acc_lim_theta': 1.0},
        'trajectory': {'teb_autosize': True, 'dt_ref': 0.2, 'dt_hysteresis': 0.02,
                       'global_plan_overwrite_orientation': True,
                       'allow_init_with_backwards_motion': False,
                       'max_global_plan_lookahead_dist': 4.0,
                       'feasibility_check_no_poses': 5,
                       'global_plan_viapoint_sep': 0.2, 'shrink_horizon_backup': True},
        'hateb': {'use_agent_agent_safety_c': semantic,
                  'use_agent_robot_safety_c': semantic,
                  'use_agent_robot_rel_vel_c': semantic,
                  'use_agent_robot_visi_c': semantic,
                  'add_invisible_humans': False,
                  'min_agent_agent_dist': 0.4, 'min_agent_robot_dist': personal_space,
                  'rel_vel_cost_threshold': 1.5, 'visibility_cost_threshold': 2.5,
                  'invisible_human_threshold': 1.0, 'prediction_time_horizon': 5.0},
        'goal': {'xy_goal_tolerance': goal_tolerance, 'yaw_goal_tolerance': 0.25,
                 'free_goal_vel': False},
        'obstacles': {'min_obstacle_dist': 0.05, 'include_costmap_obstacles': True,
                      'costmap_obstacles_behind_robot_dist': 0.5,
                      'obstacle_poses_affected': 15, 'costmap_converter_plugin': '',
                      'costmap_converter_spin_thread': True,
                      'costmap_converter_rate': 10, 'obstacle_cost_mult': 1.0,
                      'use_nonlinear_obstacle_penalty': True},
        'optim': {'no_inner_iterations': 5, 'no_outer_iterations': 4,
                  'optimization_activate': True, 'optimization_verbose': False,
                  'penalty_epsilon': 0.01, 'weight_max_vel_x': 2.0,
                  'weight_max_vel_y': 2.0, 'weight_max_agent_vel_x': 4.0,
                  'weight_max_agent_vel_y': 4.0, 'weight_nominal_agent_vel_x': 2.0,
                  'weight_max_vel_theta': 1.0, 'weight_max_agent_vel_theta': 2.0,
                  'weight_acc_lim_x': 1.0, 'weight_acc_lim_y': 1.0,
                  'weight_agent_acc_lim_x': 2.0, 'weight_agent_acc_lim_y': 2.0,
                  'weight_acc_lim_theta': 1.0, 'weight_agent_acc_lim_theta': 2.0,
                  'weight_kinematics_nh': 1000.0,
                  'weight_kinematics_forward_drive': 100.0,
                  'weight_kinematics_turning_radius': 0.0,
                  'weight_optimaltime': 1.0, 'weight_agent_optimaltime': 3.0,
                  'weight_obstacle': 50.0, 'weight_dynamic_obstacle': 50.0,
                  'weight_agent_viapoint': 0.5, 'weight_viapoint': 1.0,
                  'weight_shortest_path': 0.5,
                  'selection_alternative_time_cost': False,
                  'cap_optimaltime_penalty': True,
                  'weight_agent_robot_safety': 5.0 if semantic else 0.0,
                  'weight_agent_agent_safety': 2.0 if semantic else 0.0,
                  'weight_agent_robot_rel_vel': 5.0 if semantic else 0.0,
                  'weight_agent_robot_visibility': 5.0 if semantic else 0.0,
                  'weight_invisible_human': 0.0, 'disable_warm_start': True},
        'visualization': {'publish_agents_global_plans': semantic,
                          'publish_agents_local_plan_fp_poses': semantic,
                          'publish_agents_local_plan_poses': semantic,
                          'publish_agents_local_plans': semantic,
                          'publish_robot_global_plan': True,
                          'publish_robot_local_plan': True,
                          'publish_robot_local_plan_fp_poses': True,
                          'publish_robot_local_plan_poses': True}}


def prepare(description, wrapper, target, experiment_config=None, headless=False,
            scenario='cafe', scenario_config=None, seed=1,
            require_intended_user_support=False):
    description, wrapper, target = Path(description), Path(wrapper), Path(target)
    try:
        seed = int(seed)
    except (TypeError, ValueError) as error:
        raise ValueError('seed must be an integer') from error
    if seed < 0:
        raise ValueError('seed must be non-negative')
    config_path = (Path(experiment_config) if experiment_config else
                   Path(__file__).parents[1]/'config/experiment.yaml')
    package_root = config_path.resolve().parent.parent
    scenario_data = _load_scenario(package_root, scenario, scenario_config)
    agents_document = _load_agents(
        scenario_data['_agents_path'], scenario_data['intended_user'])
    if require_intended_user_support and scenario_data['intended_user']['enabled']:
        raise ValueError(
            f'Scenario {scenario_data["name"]!r} requires intended-user tracking/front-following; '
            'this project currently provides local-planner evaluation only')
    runtime_agents = _materialize_random_agents(agents_document, seed)
    evaluation = scenario_data['evaluation']
    target.mkdir(parents=True, exist_ok=True)
    robot = ET.fromstring(xacro.process_file(str(description/'urdf/iwalk.urdf.xacro')).toxml())
    # Preserve all physical transforms; reroot only the runtime copy at the rear axle projection.
    original = robot.find("joint[@name='base_footprint_joint']")
    old = transform(original.find('origin'))
    height = float(old[2, 3])
    original.find('parent').set('link', 'base_link')
    original.find('child').set('link', 'base_footprint')
    inverse = np.linalg.inv(old)
    original.find('origin').set('xyz', ' '.join(map(str, inverse[:3, 3])))
    original.find('origin').set('rpy', ' '.join(pose(inverse).split()[3:]))
    add(robot, 'link', name='sim_base')
    joint = add(robot, 'joint', name='sim_base_joint', type='fixed')
    add(joint, 'parent', link='sim_base'); add(joint, 'child', link='base_link')
    add(joint, 'origin', xyz=f'0 0 {height}', rpy='0 0 0')
    laser = add(robot, 'link', name='laser_frame')
    vis = add(laser, 'visual'); add(vis, 'origin', xyz='0 0 0')
    add(add(vis, 'geometry'), 'box', size='0.03 0.03 0.03')
    lj = add(robot, 'joint', name='laser_joint', type='fixed')
    add(lj, 'parent', link='sim_base'); add(lj, 'child', link='laser_frame')
    add(lj, 'origin', xyz='0.85 0 0.35', rpy='0 0 0')
    ET.ElementTree(robot).write(target/'robot.urdf', encoding='unicode')
    frames = {'sim_base': np.eye(4)}
    pending = list(robot.findall('joint'))
    while pending:
        ready = [j for j in pending if j.find('parent').get('link') in frames]
        if not ready:
            raise ValueError('URDF contains a cycle or disconnected joint')
        for j in ready:
            frames[j.find('child').get('link')] = frames[j.find('parent').get('link')] @ transform(j.find('origin'))
            pending.remove(j)
    tree = ET.parse(scenario_data['_world_path'])
    world = tree.getroot().find('world')
    if world is None:
        raise ValueError(f'{scenario_data["_world_path"]}: missing SDF world element')
    points = [('start', scenario_data['robot']['initial_pose'][:2])]
    points.extend((f'path point {index}', point)
                  for index, point in enumerate(scenario_data['robot']['path_xy']))
    _validate_obvious_static_obstacles(world, points, scenario_data['_world_path'])
    if world.find("model[@name='iwalk']") is not None:
        raise ValueError(f'{scenario_data["_world_path"]}: model name iwalk is reserved')
    model = add(world, 'model', name='iwalk')
    initial_x, initial_y, initial_yaw = scenario_data['robot']['initial_pose']
    add(model, 'pose', f'{initial_x} {initial_y} 0 0 0 {initial_yaw}')
    link = add(model, 'link', name='chassis')
    add(link, 'gravity', 'false')
    inertial = add(link, 'inertial'); add(inertial, 'mass', '25')
    inertia = add(inertial, 'inertia')
    for k,v in {'ixx':2,'iyy':2,'izz':2,'ixy':0,'ixz':0,'iyz':0}.items(): add(inertia,k,v)
    bounds = []
    for urdf_link in robot.findall('link'):
        for element in [*urdf_link.findall('visual'), *urdf_link.findall('collision')]:
            t = frames[urdf_link.get('name')] @ transform(element.find('origin'))
            g = list(element.find('geometry'))[0]
            pts = geometry_points(g, description)
            bounds.extend((pts @ t[:3,:3].T + t[:3,3]).tolist())
            if element.tag == 'visual':
                v = add(link, 'visual', name=f'visual_{len(link)}')
                add(v, 'pose', pose(t)); sdf_geometry(v, g, description)
                mat = add(v, 'material'); add(mat, 'ambient', '0.4 0.4 0.45 1'); add(mat, 'diffuse', '0.4 0.4 0.45 1')
    pts = np.array(bounds)
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    xmin,ymin = np.floor(lo[:2]*1000)/1000
    xmax,ymax = np.ceil(hi[:2]*1000)/1000
    footprint = [[float(xmax),float(ymax)],[float(xmax),float(ymin)], [float(xmin),float(ymin)],[float(xmin),float(ymax)]]
    collision = add(link,'collision',name='navigation_envelope')
    add(collision,'pose',f'{(xmin+xmax)/2} {(ymin+ymax)/2} 0.5 0 0 0')
    add(add(add(collision,'geometry'),'box'),'size',f'{xmax-xmin} {ymax-ymin} 0.96')
    sensor = add(link, 'sensor', name='lidar', type='ray' if headless else 'gpu_ray')
    add(sensor, 'pose', '0.85 0 0.35 0 0 0'); add(sensor,'always_on','true'); add(sensor,'update_rate','15')
    ray=add(sensor,'ray'); scan=add(ray,'scan'); h=add(scan,'horizontal')
    for k,v in {'samples':720,'resolution':1,'min_angle':-math.pi,'max_angle':math.pi}.items(): add(h,k,v)
    ran=add(ray,'range')
    for k,v in {'min':0.05,'max':10,'resolution':0.01}.items(): add(ran,k,v)
    sp=add(sensor,'plugin',name='lidar_ros',filename='libgazebo_ros_ray_sensor.so')
    ros=add(sp,'ros'); add(ros,'remapping','~/out:=/scan_raw')
    add(sp,'output_type','sensor_msgs/LaserScan'); add(sp,'frame_name','laser_frame')
    plugin=add(model,'plugin',name='ideal_planar_base',filename='libgazebo_ros_planar_move.so')
    for k,v in {'update_rate':50,'publish_rate':20,'odometry_frame':'odom','robot_base_frame':'sim_base', 'publish_odom':'true','publish_odom_tf':'true'}.items(): add(plugin,k,v)
    tree.write(target/'scenario.world', encoding='unicode')
    exp = yaml.safe_load(config_path.read_text())['experiment']
    limits, timeouts, dwal_config = exp['limits'], exp['timeouts'], exp['dwal']
    # Geometry is authoritative from the real URDF. Keep configured values only as an audit check.
    if abs(float(dwal_config['front_extent']) - xmax) > 0.03 or abs(float(dwal_config['rear_extent']) + xmin) > 0.03:
        raise ValueError('experiment.yaml iWalk extents disagree with the derived URDF envelope')
    levels = [float(v) for v in dwal_config['fixed_levels']]
    common={'common/levels':levels,'common/odom_frame':'odom','use_sim_time':True}
    generator={**common,'dwal_generator/odometryTopic':'/odom','dwal_generator/occ_topic':'/local_costmap/costmap', 'dwal_generator/base_frame':'sim_base','dwal_generator/footprint':[v for p in footprint for v in p], 'dwal_generator/footprint_padding':0.02,'dwal_generator/acc_lim_x':limits['acceleration'],'dwal_generator/acc_lim_th':limits['angular_acceleration'],'dwal_generator/max_trans_vel':limits['max_linear'],'dwal_generator/min_trans_vel':0.05,'dwal_generator/max_vel_theta':limits['max_angular'],'dwal_generator/sim_period':0.2,'dwal_generator/DS':0.05,'dwal_generator/Kmax':4.0,'dwal_generator/alpha':0.05,'dwal_generator/Hz':10.0,
               'dwal_generator/dynamic_radius_enabled':True,
               'dwal_generator/dynamic_radius_topic':'/dwal/dynamic_radius',
               'dwal_generator/dynamic_radius_min':dwal_config['dynamic_radius_min'],
               'dwal_generator/dynamic_radius_max':dwal_config['dynamic_radius_max'],
               'dwal_generator/dynamic_radius_min_delta':dwal_config['dynamic_min_delta']}
    clustering={**common,'dwal_clustering/postfix':['near','far'],'dwal_clustering/spin':[1,1],'dwal_clustering/min_cluster_span':0.2,'dwal_clustering/cluster_separation':5,'dwal_clustering/subsample_step':3}
    (target/'dwal.yaml').write_text(dump_yaml({'/dwal_planner/dwal_generator':{'ros__parameters':generator},'/dwal_planner/dwal_clustering':{'ros__parameters':clustering}}))
    costmap=costmap_parameters(footprint, False)
    (target/'costmap.yaml').write_text(dump_yaml({'/costmap/costmap':{'ros__parameters':costmap}}))
    for family in ('dwb', 'hateb'):
        for semantic in (False, True):
            controller = common_controller(limits, evaluation['goal_tolerance'])
            if family == 'hateb':
                # HATEB calls setGoalControl() on its concrete goal checker.
                # The pinned implementation does not safely handle Nav2's
                # SimpleGoalChecker here, so use the plugin it ships with.
                controller['general_goal_checker']['plugin'] = \
                    'hateb_local_planner::HATEBGoalChecker'
            controller['FollowPath'] = (
                dwb_plugin(limits, evaluation['goal_tolerance']) if family == 'dwb'
                else hateb_plugin(limits, footprint, semantic,
                                  evaluation['goal_tolerance'],
                                  evaluation['personal_space']))
            params = {}
            params.update(ros_params('/controller_server', controller))
            params.update(ros_params('/local_costmap/local_costmap',
                                     costmap_parameters(footprint, semantic and family == 'dwb')))
            (target/f'{family}_{"on" if semantic else "off"}.yaml').write_text(dump_yaml(params))

    reference = common_controller(limits, evaluation['goal_tolerance'])
    reference['FollowPath'] = {
        'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
        'desired_linear_vel': limits['max_linear'], 'lookahead_dist': 0.6,
        'min_lookahead_dist': 0.3, 'max_lookahead_dist': 0.9,
        'lookahead_time': 1.5, 'rotate_to_heading_angular_vel': 0.5,
        'transform_tolerance': 0.2, 'use_velocity_scaled_lookahead_dist': True,
        'min_approach_linear_velocity': 0.05, 'approach_velocity_scaling_dist': 0.6,
        'use_collision_detection': True, 'max_allowed_time_to_collision_up_to_carrot': 1.0,
        'use_rotate_to_heading': True, 'allow_reversing': False,
        'max_angular_accel': limits['angular_acceleration'], 'rotate_to_heading_min_angle': 0.785}
    reference_params = {}
    reference_params.update(ros_params('/reference/controller_server', reference))
    reference_params.update(ros_params('/reference/local_costmap/local_costmap', costmap_parameters(footprint, False)))
    (target/'reference.yaml').write_text(dump_yaml(reference_params))

    smoother = {'use_sim_time': True, 'smoothing_frequency': 20.0,
                'scale_velocities': False, 'feedback': 'OPEN_LOOP',
                'max_velocity': [limits['max_linear'], 0.0, limits['max_angular']],
                'min_velocity': [0.0, 0.0, -limits['max_angular']],
                'max_accel': [limits['acceleration'], 0.0, limits['angular_acceleration']],
                'max_decel': [-limits['deceleration'], 0.0, -limits['angular_deceleration']],
                'odom_topic': '/odom', 'odom_duration': 0.1,
                'deadband_velocity': [0.0, 0.0, 0.0], 'velocity_timeout': 0.30}
    collision = {'use_sim_time': True, 'base_frame_id': 'sim_base', 'odom_frame_id': 'odom',
                 'cmd_vel_in_topic': '/cmd_vel_smoothed', 'cmd_vel_out_topic': '/cmd_vel_safe',
                 'transform_tolerance': 0.3, 'source_timeout': 0.30,
                 'base_shift_correction': True, 'stop_pub_timeout': 0.30,
                 'polygons': ['FootprintApproach'],
                 'FootprintApproach': {'type': 'polygon', 'action_type': 'approach',
                                       'footprint_topic': '/local_costmap/published_footprint',
                                       'time_before_collision': 1.2,
                                       'simulation_time_step': 0.05, 'max_points': 1,
                                       'visualize': True, 'enabled': True},
                 'observation_sources': ['scan'],
                 'scan': {'type': 'scan', 'topic': '/scan', 'enabled': True}}
    pipeline = {}
    pipeline.update(ros_params('/velocity_smoother', smoother))
    pipeline.update(ros_params('/collision_monitor', collision))
    pipeline.update(ros_params('/command_guard', {
        'use_sim_time': True, 'input_timeout': timeouts['selected_command'],
        'speed_limit_timeout': timeouts['speed_limit'], 'output_rate': 20.0,
        'max_linear': limits['max_linear'], 'max_angular': limits['max_angular'],
        'allow_reverse': False}))
    pipeline.update(ros_params('/final_cmd_watchdog', {
        'use_sim_time': True, 'input_timeout': timeouts['final_input'], 'output_rate': 20.0}))
    (target/'command_pipeline.yaml').write_text(dump_yaml(pipeline))

    context = dict(exp['context'])
    context.update({'use_sim_time': True, 'tracks_topic': '/tracked_agents_logging',
                    'costmap_topic': '/local_costmap/costmap', 'odom_topic': '/odom',
                    'output_topic': '/crowd_context', 'output_rate': context.pop('rate')})
    (target/'context.yaml').write_text(dump_yaml(ros_params('/context_estimator', context)))

    radius_base = {'use_sim_time': True, 'odom_timeout': timeouts['odom'],
                   'context_timeout': timeouts['context'], 'fixed_radius': levels[-1],
                   'radius_min': dwal_config['dynamic_radius_min'],
                   'sensor_reliable_radius': dwal_config['reliable_sensor_radius'],
                   'costmap_reliable_radius': dwal_config['reliable_costmap_radius'],
                   'front_extent': float(xmax), 'brake_deceleration': limits['deceleration'],
                   'base_margin': exp['profiles']['OPEN_AREA']['margin'],
                   'base_reaction_time': exp['profiles']['OPEN_AREA']['reaction_time'],
                   'base_preview_time': exp['profiles']['OPEN_AREA']['preview_time'],
                   'max_speed': limits['max_linear'],
                   'radius_decrease_rate': dwal_config['radius_decrease_rate'],
                   'minimum_radius_delta': dwal_config['dynamic_min_delta'],
                   'discrete_levels': [dwal_config['dynamic_radius_min'], levels[-1], 2.6,
                                       dwal_config['dynamic_radius_max']]}
    for profile, values in exp['profiles'].items():
        for key, value in values.items(): radius_base[f'profiles.{profile}.{key}'] = value
    (target/'radius_policy.yaml').write_text(dump_yaml(ros_params('/dynamic_radius_policy', radius_base)))

    shared_common = {'use_sim_time': True, 'odom_topic': '/odom',
                     'near_cluster_topic': '/dwal_planner/clusters_near',
                     'far_cluster_topic': '/dwal_planner/clusters_far',
                     'user_cmd_topic': '/reference_cmd', 'output_cmd_topic': '/cmd_vel_selected',
                     'use_stamped_twist_input': False, 'use_stamped_twist_output': False,
                     'tracked_agents_topic': '/tracked_agents',
                     'context_profile_topic': '/crowd_context/profile',
                     'odom_timeout': timeouts['odom'], 'cluster_timeout': timeouts['clusters'],
                     'reference_timeout': timeouts['reference'],
                     'tracks_timeout': timeouts['tracks'],
                     'allow_reverse': False, 'allow_no_cluster_passthrough': False,
                     'personal_space': evaluation['personal_space'], 'social_weight': 0.35,
                     'c_max': 255.0, 'alpha': 0.7, 's_min': 0.2,
                     'Kphi': 1.5, 'Kmax': 4.0, 'v_low': 0.22, 'v_high': 0.28,
                     'intent.alpha': 0.92, 'intent.beta': 8.0,
                     'intent.eta': 0.5, 'intent.sigma_min': 0.03,
                     'intent.min_confidence': 0.0}
    for semantic in (False, True):
        shared = dict(shared_common)
        shared['semantic_mode'] = semantic
        (target/f'shared_control_{"on" if semantic else "off"}.yaml').write_text(
            dump_yaml(ros_params('/shared_controller', shared)))

    path_xy = [value for point in scenario_data['robot']['path_xy'] for value in point]
    task = {'use_sim_time': True, 'path_xy': path_xy,
            'timeout': evaluation['timeout'],
            'odom_topic': '/odom', 'path_topic': '/experiment/path'}
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
    goals_file.write_text(dump_yaml({'window_size': 10, 'goals': prediction_goals}))
    predictor = {'use_sim_time': True, 'tracked_agents_sub_topic': '/tracked_agents',
                 'robot_frame_id': 'sim_base', 'map_frame_id': 'odom',
                 'goals_file': str(goals_file), 'publish_markers': True}
    (target/'agent_prediction.yaml').write_text(dump_yaml(ros_params('/agent_path_prediction', predictor)))
    self_filter_padding = 0.03
    scan_filter = {
        'use_sim_time': True,
        'filter1': {
            'name': 'iwalk_self_box',
            'type': 'laser_filters/LaserScanBoxFilter',
            'params': {
                'box_frame': 'sim_base',
                'min_x': float(xmin - self_filter_padding),
                'max_x': float(xmax + self_filter_padding),
                'min_y': float(ymin - self_filter_padding),
                'max_y': float(ymax + self_filter_padding),
                'min_z': -1.0, 'max_z': 2.0, 'invert': False}}}
    (target/'scan_filter.yaml').write_text(dump_yaml(ros_params('/scan_self_filter', scan_filter)))
    (target/'geometry.yaml').write_text(dump_yaml({
        'footprint':footprint, 'front_extent':float(xmax), 'rear_extent':float(-xmin),
        'laser_xyz':[0.85,0,0.35], 'base_link_height':height,
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
    return target


if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('description')
    p.add_argument('wrapper')
    p.add_argument('output')
    p.add_argument('--experiment-config')
    p.add_argument('--scenario', default='cafe')
    p.add_argument('--scenario-config')
    p.add_argument('--seed', type=int, default=1)
    p.add_argument('--headless', action='store_true')
    p.add_argument('--require-intended-user-support', action='store_true')
    a = p.parse_args()
    prepare(a.description, a.wrapper, a.output, a.experiment_config, a.headless,
            a.scenario, a.scenario_config, a.seed, a.require_intended_user_support)
