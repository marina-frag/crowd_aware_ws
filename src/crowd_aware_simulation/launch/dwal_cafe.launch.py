"""Repeatable local-only iWalk / HuNav scenario experiments."""
import importlib.util
import os
import json
import sys
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node




def truth(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('1', 'true', 'yes', 'on')


def lifecycle_manager(name, nodes, namespace=None, *, cfg):
    return Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name=name, namespace=namespace, output='screen',
        parameters=[{'use_sim_time': cfg['runtime']['use_sim_time'], 'autostart': cfg['launch']['lifecycle_autostart'],
                     'node_names': nodes, 'bond_timeout': cfg['launch']['bond_timeout']}])


def configured_node(exp, **kwargs):
    api_topics = {
        '/odom': 'odom', '/scan': 'scan', '/human_states': 'human_states',
        '/robot_states': 'robot_states', '/tracked_agents': 'tracked_agents_controller',
        '/tracked_agents_logging': 'tracked_agents_logging', '/agents_info': 'agents_info',
        '/crowd_context': 'context', '/crowd_context/profile': 'context_profile',
        '/reference_cmd': 'reference_cmd', '/cmd_vel_selected': 'selected_cmd',
        '/cmd_vel_guarded': 'guarded_cmd', '/cmd_vel_smoothed': 'smoothed_cmd',
        '/cmd_vel_safe': 'collision_checked_cmd', '/cmd_vel': 'final_cmd',
        '/experiment/speed_limit': 'speed_limit', '/experiment/path': 'path'}
    mappings = [(source, exp['topics'][key]) for source, key in api_topics.items()
                if source != exp['topics'][key]]
    if mappings:
        kwargs['remappings'] = [*kwargs.get('remappings', []), *mappings]
    return Node(**kwargs)


def setup(context):
    controller = LaunchConfiguration('controller').perform(context)
    semantics = LaunchConfiguration('semantics').perform(context)
    reference_mode = LaunchConfiguration('reference_mode').perform(context)
    seed = LaunchConfiguration('seed').perform(context)
    scenario = LaunchConfiguration('scenario').perform(context)
    share = Path(get_package_share_directory('crowd_aware_simulation'))
    runtime = json.loads((share/'config/runtime.json').read_text())
    if controller not in runtime['controllers']:
        raise RuntimeError(f"controller must be one of {runtime['controllers']}, got {controller!r}")
    if semantics not in runtime['semantic_modes']:
        raise RuntimeError(f"semantics must be one of {runtime['semantic_modes']}, got {semantics!r}")
    if reference_mode not in runtime['reference_modes']:
        raise RuntimeError('reference_mode must be autonomous or teleop')
    timeout_override = LaunchConfiguration('teleop_input_timeout').perform(context)
    try:
        int(seed)
    except ValueError as error:
        raise RuntimeError('seed must be an integer') from error

    result_file = LaunchConfiguration('result_file').perform(context) or str(
        Path(runtime['result_directory'])/f'metrics_{scenario}_{controller}_{semantics}_seed{seed}')
    semantic_on = semantics == 'on'
    is_dwal = controller in ('fixed_dwal', 'dynamic_dwal')
    is_dynamic_dwal = controller == 'dynamic_dwal'
    headless = truth(context, 'headless')
    share = Path(get_package_share_directory('crowd_aware_simulation'))
    wrapper = Path(get_package_share_directory('hunav_gazebo_wrapper'))
    description = Path(get_package_share_directory('iwalk_description'))
    spec = importlib.util.spec_from_file_location('prepare_scene', share/'scripts/prepare_scene.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    cfg = module.load_configuration(share/'config/experiment.yaml')
    exp = cfg['experiment']
    topics, frames = exp['topics'], exp['frames']
    teleop_input_timeout = float(timeout_override) if timeout_override else exp['timeouts']['reference']
    if teleop_input_timeout <= 0.0:
        raise RuntimeError('teleop_input_timeout must be positive')
    run = module.prepare(
        description, wrapper, Path(tempfile.mkdtemp(prefix='crowd_aware_')),
        share/'config/experiment.yaml', headless=headless, scenario=scenario,
        seed=int(seed), controller=controller, require_intended_user_support=True)
    resolved = cfg | {'selection': {'controller': controller, 'semantics': semantics,
        'reference_mode': reference_mode, 'scenario': scenario, 'seed': int(seed),
        'headless': headless, 'gui': truth(context, 'gui'), 'rviz': truth(context, 'rviz'),
        'evaluator': truth(context, 'evaluator'), 'reference_timeout': teleop_input_timeout,
        'result_file': result_file}}
    (run/'resolved_config.yaml').write_text(module.dump_yaml(resolved))
    print(f'Scenario={scenario}, experiment={controller}/{semantics}, '
          f'reference={reference_mode}, seed={seed}; runtime files={run}', flush=True)

    if is_dwal:
        print('DWAL_UPSTREAM_V4: dwal_planner/dwal_generator + dwal_clustering + '
              f'bayesian_shared_control/shared_control_node; variable_length={is_dynamic_dwal}', flush=True)

    model_paths = {str(wrapper/'models'), '/usr/share/gazebo-11/models', '/opt/gazebo_models'}
    for model_name in ('cafe', 'cafe_table', 'ground_plane'):
        if not (Path('/opt/gazebo_models')/model_name/'model.sdf').is_file():
            raise RuntimeError(f'Missing bundled Gazebo model {model_name}; rebuild the image')
    model_paths.update(str(path.parent) for path in (wrapper/'models').rglob('*.dae'))
    model_paths.update(str(path.parent) for path in (wrapper/'models').rglob('*.bvh'))
    resource_paths = {str(wrapper/'models'), '/usr/share/gazebo-11'} | model_paths

    def environment(name, paths):
        return SetEnvironmentVariable(
            name, os.pathsep.join(sorted(paths) + [os.environ.get(name, '')]))

    loader = configured_node(exp, 
        package='hunav_agent_manager', executable='hunav_loader', output='screen',
        parameters=[str(run/'agents.yaml')])
    generator = configured_node(exp, 
        package='hunav_gazebo_wrapper', executable='hunav_gazebo_world_generator',
        output='screen', parameters=[{
            'base_world': str(run/'scenario.world'), 'use_gazebo_obs': cfg['simulation']['hunav']['use_gazebo_obs'],
            'use_collision': is_dynamic_dwal, 'update_rate': cfg['launch']['hunav_update_rate'], 'robot_name': cfg['simulation']['robot_name'],
            'global_frame_to_publish': frames['global'], 'use_navgoal_to_start': cfg['simulation']['hunav']['use_navgoal_to_start'],
            'navgoal_topic': cfg['simulation']['hunav']['navgoal_topic'], 'ignore_models': cfg['simulation']['hunav']['ignore_models']}])
    manager = configured_node(exp, 
        package='hunav_agent_manager', executable='hunav_agent_manager',
        output='screen', parameters=[{'use_sim_time': runtime['use_sim_time']}])
    world_ready = ExecuteProcess(
        cmd=[sys.executable, str(share/'scripts/wait_world.py'),
             str(run/'generatedWorld.world')], output='screen')
    server = ExecuteProcess(
        cmd=['gzserver', '--verbose', '--seed', seed, str(run/'generatedWorld.world'),
             '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen')
    client = ExecuteProcess(
        cmd=['gzclient'], output='screen', condition=IfCondition(LaunchConfiguration('gui')))
    rsp = configured_node(exp, 
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': (run/'robot.urdf').read_text(), 'use_sim_time': runtime['use_sim_time']}])
    jsp = configured_node(exp, 
        package='joint_state_publisher', executable='joint_state_publisher',
        parameters=[{'use_sim_time': runtime['use_sim_time']}])
    scan_filter = configured_node(exp, 
        package='laser_filters', executable='scan_to_scan_filter_chain',
        name='scan_self_filter', output='screen',
        parameters=[str(run/'scan_filter.yaml')],
        remappings=[('scan', topics['scan_raw']),
                    ('scan_filtered', topics['scan'])])

    adapter = configured_node(exp, 
        package='crowd_aware_simulation', executable='hunav_to_cohan_bridge.py',
        name='hunav_to_cohan_bridge', output='screen', parameters=[{
            'use_sim_time': runtime['use_sim_time'], 'publish_controller_output': semantic_on}])
    context_estimator = configured_node(exp, 
        package='crowd_aware_simulation', executable='context_estimator.py',
        name='context_estimator', output='screen', parameters=[str(run/'context.yaml')])
    context_speed_limit = configured_node(exp, 
        package='crowd_aware_simulation', executable='context_speed_limit.py',
        name='context_speed_limit', output='screen',
        parameters=[str(run/'context_speed_limit.yaml'), {
            'semantic_mode': semantic_on}])
    guard_parameters = [str(run/'command_pipeline.yaml')]
    if reference_mode == 'teleop' and not is_dwal:
        guard_parameters.append({
            'reference_gate_enabled': True,
            'reference_gate_timeout': teleop_input_timeout})
    guard = configured_node(exp, 
        package='crowd_aware_simulation', executable='command_guard.py',
        name='command_guard', output='screen', parameters=guard_parameters)
    smoother = configured_node(exp, 
        package='nav2_velocity_smoother', executable='velocity_smoother',
        name='velocity_smoother', output='screen', parameters=[str(run/'command_pipeline.yaml')],
        remappings=[('cmd_vel', topics['guarded_cmd']),
                    ('cmd_vel_smoothed', topics['smoothed_cmd'])])
    collision = configured_node(exp, 
        package='nav2_collision_monitor', executable='collision_monitor',
        name='collision_monitor', output='screen', parameters=[str(run/'command_pipeline.yaml')])
    final_watchdog = configured_node(exp, 
        package='crowd_aware_simulation', executable='final_cmd_watchdog.py',
        name='final_cmd_watchdog', output='screen',
        parameters=[str(run/'command_pipeline.yaml')])
    evaluator = configured_node(exp, 
        package='hunav_evaluator', executable='hunav_evaluator_node',
        name='hunav_evaluator_node', output='screen',
        parameters=[{'use_sim_time': runtime['use_sim_time'], 'frequency': cfg['launch']['evaluator_frequency'],
                     'result_file': result_file}],
        remappings=[('human_states', topics['human_states']),
                    ('robot_states', topics['robot_states'])],
        condition=IfCondition(LaunchConfiguration('evaluator')))
    pipeline_manager = lifecycle_manager(
        'command_pipeline_lifecycle_manager', ['velocity_smoother', 'collision_monitor'], cfg=cfg)

    task_action = '/reference/follow_path' if is_dwal else '/follow_path'
    task_lifecycle_node = '/reference/controller_server' if is_dwal else '/controller_server'
    if reference_mode == 'teleop' and not is_dwal:
        task_nodes = [configured_node(exp, 
            package='crowd_aware_simulation', executable='teleop_path_adapter.py',
            name='teleop_path_adapter', output='screen', parameters=[{
                'use_sim_time': runtime['use_sim_time'],
                'action_name': task_action,
                'controller_lifecycle_node': task_lifecycle_node,
                'input_timeout': teleop_input_timeout}])]
    else:
        task_nodes = [configured_node(exp, 
            package='crowd_aware_simulation', executable='local_task.py', name='local_task',
            output='screen', parameters=[str(run/'task.yaml'), {
                'evaluator_enabled': truth(context, 'evaluator'),
                'experiment_tag': f'{scenario}_{controller}_{semantics}',
                'run_id': int(seed),
                'action_name': task_action,
                'controller_lifecycle_node': task_lifecycle_node,
                'autostart': reference_mode == 'autonomous'}])]

    condition_nodes = []
    if is_dwal:
        costmap = configured_node(exp, 
            package='nav2_costmap_2d', executable='nav2_costmap_2d',
            name='costmap', namespace='costmap', output='screen',
            parameters=[str(run/'costmap.yaml')],
            remappings=[('costmap', topics['costmap']),
                        ('costmap_updates', topics['costmap_updates']),
                        ('published_footprint', topics['published_footprint'])])
        condition_nodes += [
            costmap, lifecycle_manager(
                'costmap_lifecycle_manager', ['/costmap/costmap'], cfg=cfg)]
    if is_dwal:
        condition_nodes += [
            configured_node(exp, package='dwal_planner', executable='dwal_clustering',
                 name='dwal_clustering', namespace='dwal_planner',
                 output='screen', parameters=[str(run/'dwal.yaml')]),
            configured_node(exp, package='dwal_planner', executable='dwal_generator',
                 name='dwal_generator', namespace='dwal_planner',
                 output='screen', parameters=[str(run/'dwal.yaml')]),
            configured_node(exp, package='bayesian_shared_control', executable='shared_control_node',
                 name='shared_controller', output='screen',
                 parameters=[str(run/f'shared_control_{semantics}.yaml'),
                             {'reference_timeout':teleop_input_timeout}]
                 if reference_mode == 'teleop' else [str(run/f'shared_control_{semantics}.yaml')]),
            configured_node(exp, package='crowd_aware_simulation', executable='constant_speed_limit.py',
                 name='constant_speed_limit', output='screen',
                 parameters=[{'use_sim_time':runtime['use_sim_time'],'max_speed':exp['limits']['max_linear']}])]
        if reference_mode == 'autonomous':
            condition_nodes += [
                configured_node(exp, package='nav2_controller', executable='controller_server',
                     name='controller_server', namespace='reference', output='screen',
                     parameters=[str(run/'reference.yaml')],
                     remappings=[('cmd_vel',topics['reference_cmd'])]),
                lifecycle_manager('reference_lifecycle_manager', ['controller_server'], namespace='reference', cfg=cfg)]
    else:
        controller_server = configured_node(exp, 
            package='nav2_controller', executable='controller_server',
            name='controller_server', output='screen',
            parameters=[str(run/f'{controller}_{semantics}.yaml')],
            remappings=[('cmd_vel', topics['selected_cmd'])])
        condition_nodes += [
            controller_server,
            lifecycle_manager('controller_lifecycle_manager', ['/controller_server'], cfg=cfg)]
        if controller == 'hateb':
            condition_nodes.append(configured_node(exp, 
                package='agent_path_prediction', executable='agent_path_predict',
                name='agent_path_prediction', output='screen',
                parameters=[str(run/'agent_prediction.yaml'), {
                    # HATEB requires prediction reset services even in mode 0.
                    # OFF points the service host at an intentionally empty topic.
                    'tracked_agents_sub_topic':
                        topics['tracked_agents_controller'] if semantic_on else topics['tracked_agents_disabled'],
                    'publish_markers': semantic_on}]))
        elif controller == 'dwb' and semantic_on:
            condition_nodes.append(configured_node(exp, 
                package='crowd_aware_simulation',
                executable='cohan_agents_info_adapter.py', output='screen'))

    rviz = configured_node(exp, 
        package='rviz2', executable='rviz2',
        arguments=['-d', str(share/'rviz'/('dwal_dynamic.rviz'
                    if is_dynamic_dwal else 'dwal.rviz'))],
        parameters=[{'use_sim_time': runtime['use_sim_time']}],
        condition=IfCondition(LaunchConfiguration('rviz')))
    adaptation_nodes = (
        [context_estimator, context_speed_limit] if not is_dwal else [])
    runtime_nodes = [
        server, client, rsp, jsp, scan_filter, adapter, *adaptation_nodes,
        guard, smoother, collision, final_watchdog, evaluator, pipeline_manager,
        *condition_nodes, *task_nodes, rviz]

    def after_world(event, _):
        if event.returncode:
            return [EmitEvent(event=Shutdown(reason='HuNav world generation failed'))]
        return runtime_nodes

    return [
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        environment('GAZEBO_MODEL_PATH', model_paths),
        environment('GAZEBO_RESOURCE_PATH', resource_paths),
        environment('GAZEBO_PLUGIN_PATH', {
            str(Path(get_package_prefix('hunav_gazebo_wrapper'))/'lib')}),
        RegisterEventHandler(OnProcessExit(target_action=world_ready, on_exit=after_world)),
        RegisterEventHandler(OnProcessExit(
            target_action=server,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo server stopped'))])),
        loader, generator, manager, world_ready]


def generate_launch_description():
    share = Path(get_package_share_directory('crowd_aware_simulation'))
    defaults = json.loads((share/'config/runtime.json').read_text())
    arguments = [
        DeclareLaunchArgument(name, default_value=(str(defaults[name]).lower()
            if isinstance(defaults[name], bool) else str(defaults[name])))
        for name in ('controller', 'semantics', 'reference_mode', 'scenario', 'seed',
                     'headless', 'gui', 'rviz', 'evaluator')]
    return LaunchDescription(arguments + [
        DeclareLaunchArgument('teleop_input_timeout', default_value=''),
        DeclareLaunchArgument('result_file', default_value=''),
        OpaqueFunction(function=setup)])
