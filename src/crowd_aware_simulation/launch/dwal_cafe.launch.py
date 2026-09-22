"""Eight-condition local-only iWalk / HuNav café experiment."""
import importlib.util
import os
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


CONTROLLERS = ('fixed_dwal', 'dwb', 'hateb', 'dynamic_dwal')
SEMANTIC_MODES = ('off', 'on')


def truth(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('1', 'true', 'yes', 'on')


def lifecycle_manager(name, nodes, namespace=None):
    return Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name=name, namespace=namespace, output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True,
                     'node_names': nodes, 'bond_timeout': 0.0}])


def setup(context):
    controller = LaunchConfiguration('controller').perform(context)
    semantics = LaunchConfiguration('semantics').perform(context)
    reference_mode = LaunchConfiguration('reference_mode').perform(context)
    radius_mode = LaunchConfiguration('radius_mode').perform(context)
    seed = LaunchConfiguration('seed').perform(context)
    if controller not in CONTROLLERS:
        raise RuntimeError(f'controller must be one of {CONTROLLERS}, got {controller!r}')
    if semantics not in SEMANTIC_MODES:
        raise RuntimeError(f'semantics must be one of {SEMANTIC_MODES}, got {semantics!r}')
    if reference_mode not in ('autonomous', 'teleop'):
        raise RuntimeError('reference_mode must be autonomous or teleop')
    if radius_mode not in ('discrete', 'continuous'):
        raise RuntimeError('radius_mode must be discrete or continuous')
    teleop_input_timeout = float(
        LaunchConfiguration('teleop_input_timeout').perform(context))
    if teleop_input_timeout <= 0.0:
        raise RuntimeError('teleop_input_timeout must be positive')
    try:
        int(seed)
    except ValueError as error:
        raise RuntimeError('seed must be an integer') from error

    semantic_on = semantics == 'on'
    is_dwal = controller in ('fixed_dwal', 'dynamic_dwal')
    adaptation_mode = ('fixed' if controller != 'dynamic_dwal' else radius_mode)
    headless = truth(context, 'headless')
    share = Path(get_package_share_directory('crowd_aware_simulation'))
    wrapper = Path(get_package_share_directory('hunav_gazebo_wrapper'))
    description = Path(get_package_share_directory('iwalk_description'))
    spec = importlib.util.spec_from_file_location('prepare_scene', share/'scripts/prepare_scene.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    run = module.prepare(
        description, wrapper, Path(tempfile.mkdtemp(prefix='crowd_aware_cafe_')),
        share/'config/experiment.yaml', headless=headless)
    print(f'Experiment={controller}/{semantics}, reference={reference_mode}, '
          f'radius={adaptation_mode}, seed={seed}; runtime files={run}', flush=True)

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

    loader = Node(
        package='hunav_agent_manager', executable='hunav_loader', output='screen',
        parameters=[str(wrapper/'scenarios/agents_cafe.yaml')])
    generator = Node(
        package='hunav_gazebo_wrapper', executable='hunav_gazebo_world_generator',
        output='screen', parameters=[{
            'base_world': str(run/'cafe.world'), 'use_gazebo_obs': True,
            'use_collision': False, 'update_rate': 20.0, 'robot_name': 'iwalk',
            'global_frame_to_publish': 'odom', 'use_navgoal_to_start': False,
            'navgoal_topic': '/goal_pose', 'ignore_models': 'ground_plane'}])
    manager = Node(
        package='hunav_agent_manager', executable='hunav_agent_manager',
        output='screen', parameters=[{'use_sim_time': True}])
    world_ready = ExecuteProcess(
        cmd=[sys.executable, str(share/'scripts/wait_world.py'),
             str(run/'generatedWorld.world')], output='screen')
    server = ExecuteProcess(
        cmd=['gzserver', '--verbose', '--seed', seed, str(run/'generatedWorld.world'),
             '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen')
    client = ExecuteProcess(
        cmd=['gzclient'], output='screen', condition=IfCondition(LaunchConfiguration('gui')))
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': (run/'robot.urdf').read_text(), 'use_sim_time': True}])
    jsp = Node(
        package='joint_state_publisher', executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}])
    scan_filter = Node(
        package='laser_filters', executable='scan_to_scan_filter_chain',
        name='scan_self_filter', output='screen',
        parameters=[str(run/'scan_filter.yaml')],
        remappings=[('scan', '/scan_raw'),
                    ('scan_filtered', '/scan')])

    adapter = Node(
        package='crowd_aware_simulation', executable='hunav_to_cohan_bridge.py',
        name='hunav_to_cohan_bridge', output='screen', parameters=[{
            'use_sim_time': True, 'publish_controller_output': semantic_on}])
    context_estimator = Node(
        package='crowd_aware_simulation', executable='context_estimator.py',
        name='context_estimator', output='screen', parameters=[str(run/'context.yaml')])
    radius_policy = Node(
        package='crowd_aware_simulation', executable='dynamic_radius_policy.py',
        name='dynamic_radius_policy', output='screen',
        parameters=[str(run/'radius_policy.yaml'), {
            'semantic_mode': semantic_on, 'adaptation_mode': adaptation_mode}])
    guard_parameters = [str(run/'command_pipeline.yaml')]
    if reference_mode == 'teleop' and not is_dwal:
        guard_parameters.append({
            'reference_gate_enabled': True,
            'reference_gate_timeout': teleop_input_timeout})
    guard = Node(
        package='crowd_aware_simulation', executable='command_guard.py',
        name='command_guard', output='screen', parameters=guard_parameters)
    smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother',
        name='velocity_smoother', output='screen', parameters=[str(run/'command_pipeline.yaml')],
        remappings=[('cmd_vel', '/cmd_vel_guarded'),
                    ('cmd_vel_smoothed', '/cmd_vel_smoothed')])
    collision = Node(
        package='nav2_collision_monitor', executable='collision_monitor',
        name='collision_monitor', output='screen', parameters=[str(run/'command_pipeline.yaml')])
    final_watchdog = Node(
        package='crowd_aware_simulation', executable='final_cmd_watchdog.py',
        name='final_cmd_watchdog', output='screen',
        parameters=[str(run/'command_pipeline.yaml')])
    evaluator = Node(
        package='hunav_evaluator', executable='hunav_evaluator_node',
        name='hunav_evaluator_node', output='screen',
        parameters=[{'use_sim_time': True, 'frequency': 10.0,
                     'result_file': LaunchConfiguration('result_file')}],
        remappings=[('human_states', '/human_states'),
                    ('robot_states', '/robot_states')],
        condition=IfCondition(LaunchConfiguration('evaluator')))
    pipeline_manager = lifecycle_manager(
        'command_pipeline_lifecycle_manager', ['velocity_smoother', 'collision_monitor'])

    task_action = '/reference/follow_path' if is_dwal else '/follow_path'
    task_lifecycle_node = '/reference/controller_server' if is_dwal else '/controller_server'
    if reference_mode == 'teleop' and not is_dwal:
        task_nodes = [Node(
            package='crowd_aware_simulation', executable='teleop_path_adapter.py',
            name='teleop_path_adapter', output='screen', parameters=[{
                'use_sim_time': True,
                'action_name': task_action,
                'controller_lifecycle_node': task_lifecycle_node,
                'input_timeout': teleop_input_timeout}])]
    else:
        task_nodes = [Node(
            package='crowd_aware_simulation', executable='local_task.py', name='local_task',
            output='screen', parameters=[str(run/'task.yaml'), {
                'evaluator_enabled': truth(context, 'evaluator'),
                'experiment_tag': f'{controller}_{semantics}',
                'run_id': int(seed),
                'action_name': task_action,
                'controller_lifecycle_node': task_lifecycle_node,
                'autostart': reference_mode == 'autonomous'}])]

    condition_nodes = []
    if is_dwal:
        costmap = Node(
            package='nav2_costmap_2d', executable='nav2_costmap_2d',
            name='costmap', namespace='costmap', output='screen',
            parameters=[str(run/'costmap.yaml')],
            remappings=[('costmap', '/local_costmap/costmap'),
                        ('costmap_updates', '/local_costmap/costmap_updates'),
                        ('published_footprint', '/local_costmap/published_footprint')])
        condition_nodes += [
            costmap, lifecycle_manager(
                'costmap_lifecycle_manager', ['/costmap/costmap'])]
        clustering = Node(
            package='dwal_planner', executable='dwal_clustering',
            name='dwal_clustering', namespace='dwal_planner',
            output='screen', parameters=[str(run/'dwal.yaml')])
        generation = Node(
            package='dwal_planner', executable='dwal_generator',
            name='dwal_generator', namespace='dwal_planner',
            output='screen', parameters=[str(run/'dwal.yaml')])
        clustering_ready = ExecuteProcess(
            cmd=[sys.executable, str(share/'scripts/wait_clustering.py')], output='screen')

        def after_clustering(event, _):
            if event.returncode:
                return [EmitEvent(event=Shutdown(reason='DWAL clustering failed'))]
            return [generation]

        condition_nodes += [
            RegisterEventHandler(OnProcessExit(
                target_action=clustering_ready, on_exit=after_clustering)),
            clustering, clustering_ready,
            Node(package='bayesian_shared_control', executable='shared_control_node',
                 name='shared_controller', output='screen',
                 parameters=[str(run/f'shared_control_{semantics}.yaml')])]
        if reference_mode == 'autonomous':
            condition_nodes += [
                Node(package='nav2_controller', executable='controller_server',
                     name='controller_server', namespace='reference', output='screen',
                     parameters=[str(run/'reference.yaml')],
                     remappings=[('cmd_vel', '/reference_cmd')]),
                lifecycle_manager('reference_lifecycle_manager',
                                  ['controller_server'], namespace='reference')]
    else:
        controller_server = Node(
            package='nav2_controller', executable='controller_server',
            name='controller_server', output='screen',
            parameters=[str(run/f'{controller}_{semantics}.yaml')],
            remappings=[('cmd_vel', '/cmd_vel_selected')])
        condition_nodes += [
            controller_server,
            lifecycle_manager('controller_lifecycle_manager', ['/controller_server'])]
        if controller == 'hateb':
            condition_nodes.append(Node(
                package='agent_path_prediction', executable='agent_path_predict',
                name='agent_path_prediction', output='screen',
                parameters=[str(run/'agent_prediction.yaml'), {
                    # HATEB requires prediction reset services even in mode 0.
                    # OFF points the service host at an intentionally empty topic.
                    'tracked_agents_sub_topic':
                        '/tracked_agents' if semantic_on else '/tracked_agents_disabled',
                    'publish_markers': semantic_on}]))
        elif controller == 'dwb' and semantic_on:
            condition_nodes.append(Node(
                package='crowd_aware_simulation',
                executable='cohan_agents_info_adapter.py', output='screen'))

    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', str(share/'rviz/dwal.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')))
    runtime_nodes = [
        server, client, rsp, jsp, scan_filter, adapter, context_estimator, radius_policy,
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
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='fixed_dwal'),
        DeclareLaunchArgument('semantics', default_value='off'),
        DeclareLaunchArgument('reference_mode', default_value='autonomous'),
        DeclareLaunchArgument('teleop_input_timeout', default_value='0.35'),
        DeclareLaunchArgument('radius_mode', default_value='continuous'),
        DeclareLaunchArgument('seed', default_value='1'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('evaluator', default_value='true'),
        DeclareLaunchArgument('result_file', default_value='/rosbags/metrics'),
        OpaqueFunction(function=setup)])
