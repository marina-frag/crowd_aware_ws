"""Run inside the supplied Humble / Gazebo Classic image."""
import importlib.util
import os
import sys
import tempfile
from pathlib import Path
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess,OpaqueFunction,RegisterEventHandler,EmitEvent,SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share=Path(get_package_share_directory('crowd_aware_simulation'))
    wrapper=Path(get_package_share_directory('hunav_gazebo_wrapper'))
    description=Path(get_package_share_directory('iwalk_description'))
    spec=importlib.util.spec_from_file_location('prepare_scene',share/'scripts/prepare_scene.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    run=module.prepare(description,wrapper,Path(tempfile.mkdtemp(prefix='dwal_cafe_')))
    print(f'DWAL runtime files and derived footprint: {run}',flush=True)
    # Rendering resources for the existing actors and cafe; use installed model directories.
    model_paths={str(wrapper/'models'),'/usr/share/gazebo-11/models','/opt/gazebo_models'}
    for model_name in ['cafe','cafe_table','ground_plane']:
        if not (Path('/opt/gazebo_models')/model_name/'model.sdf').is_file():
            raise RuntimeError(f'Missing bundled Gazebo model {model_name}; rebuild the updated Docker image')
    model_paths.update(str(p.parent) for p in (wrapper/'models').rglob('*.dae'))
    model_paths.update(str(p.parent) for p in (wrapper/'models').rglob('*.bvh'))
    resource_paths={str(wrapper/'models'),'/usr/share/gazebo-11'} | model_paths
    def env(name,paths):
        return SetEnvironmentVariable(name,os.pathsep.join(sorted(paths)+[os.environ.get(name,'')]))
    loader=Node(package='hunav_agent_manager',executable='hunav_loader',output='screen',
                parameters=[str(wrapper/'scenarios/agents_cafe.yaml')])
    generator=Node(package='hunav_gazebo_wrapper',executable='hunav_gazebo_world_generator',output='screen',parameters=[{
        'base_world':str(run/'cafe.world'),'use_gazebo_obs':True,'use_collision':False,
        'update_rate':20.0,'robot_name':'iwalk','global_frame_to_publish':'odom',
        'use_navgoal_to_start':False,'navgoal_topic':'/goal_pose','ignore_models':'ground_plane'}])
    manager=Node(package='hunav_agent_manager',executable='hunav_agent_manager',output='screen',parameters=[{'use_sim_time':True}])
    ready=ExecuteProcess(cmd=[sys.executable,str(share/'scripts/wait_world.py'),str(run/'generatedWorld.world')],output='screen')
    server=ExecuteProcess(cmd=['gzserver','--verbose',str(run/'generatedWorld.world'),'-s','libgazebo_ros_init.so','-s','libgazebo_ros_factory.so'],output='screen')
    client=ExecuteProcess(cmd=['gzclient'],output='screen',condition=IfCondition(LaunchConfiguration('gui')))
    rsp=Node(package='robot_state_publisher',executable='robot_state_publisher',output='screen',parameters=[{'robot_description':(run/'robot.urdf').read_text(),'use_sim_time':True}])
    jsp=Node(package='joint_state_publisher',executable='joint_state_publisher',parameters=[{'use_sim_time':True}])
    costmap=Node(package='nav2_costmap_2d',executable='nav2_costmap_2d',output='screen',parameters=[str(run/'costmap.yaml')],remappings=[('costmap','/local_costmap/costmap'),('costmap_updates','/local_costmap/costmap_updates')])
    lifecycle=Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='costmap_lifecycle_manager',output='screen',parameters=[{'use_sim_time':True,'autostart':True,'node_names':['/costmap/costmap'],'bond_timeout':0.0}])
    dwal=[Node(package='dwal_planner',executable=name,name=name,namespace='dwal_planner',output='screen',parameters=[str(run/'dwal.yaml')]) for name in ['dwal_clustering','dwal_generator']]
    dwal_ready=ExecuteProcess(cmd=[sys.executable,str(share/'scripts/wait_clustering.py')],output='screen')
    def after_clustering(event,ctx):
        return [dwal[1]] if event.returncode==0 else [EmitEvent(event=Shutdown(reason='DWAL clustering failed'))]
    dwal_handler=RegisterEventHandler(OnProcessExit(target_action=dwal_ready,on_exit=after_clustering))
    rviz=Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'rviz/dwal.rviz')],parameters=[{'use_sim_time':True}],condition=IfCondition(LaunchConfiguration('rviz')))
    def after_world(event,ctx):
        if event.returncode!=0:
            return [EmitEvent(event=Shutdown(reason='HuNav world generation failed'))]
        return [server,client,rsp,jsp,costmap,lifecycle,dwal_handler,dwal[0],dwal_ready,rviz]
    return [SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI',''),env('GAZEBO_MODEL_PATH',model_paths),env('GAZEBO_RESOURCE_PATH',resource_paths),
            env('GAZEBO_PLUGIN_PATH',{str(Path(get_package_prefix('hunav_gazebo_wrapper'))/'lib')}),
            RegisterEventHandler(OnProcessExit(target_action=ready,on_exit=after_world)),
            RegisterEventHandler(OnProcessExit(target_action=server,on_exit=[EmitEvent(event=Shutdown(reason='Gazebo server stopped'))])),
            loader,generator,manager,ready]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('gui',default_value='true'),DeclareLaunchArgument('rviz',default_value='true'),OpaqueFunction(function=setup)])
