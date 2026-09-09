"""Offline launch helper: derive simulator geometry and configs from the real URDF.
No ROS publishers or planner implementation live here.
"""
from pathlib import Path
import copy
import math
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


def prepare(description, wrapper, target):
    description, wrapper, target = Path(description), Path(wrapper), Path(target)
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
    tree = ET.parse(wrapper/'worlds/cafe.world')
    world = tree.getroot().find('world')
    model = add(world, 'model', name='iwalk')
    add(model, 'pose', '0 0 0 0 0 0')
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
    sensor = add(link, 'sensor', name='lidar', type='gpu_ray')
    add(sensor, 'pose', '0.85 0 0.35 0 0 0'); add(sensor,'always_on','true'); add(sensor,'update_rate','15')
    ray=add(sensor,'ray'); scan=add(ray,'scan'); h=add(scan,'horizontal')
    for k,v in {'samples':720,'resolution':1,'min_angle':-math.pi,'max_angle':math.pi}.items(): add(h,k,v)
    ran=add(ray,'range')
    for k,v in {'min':0.05,'max':10,'resolution':0.01}.items(): add(ran,k,v)
    sp=add(sensor,'plugin',name='lidar_ros',filename='libgazebo_ros_ray_sensor.so')
    ros=add(sp,'ros'); add(ros,'remapping','~/out:=/scan')
    add(sp,'output_type','sensor_msgs/LaserScan'); add(sp,'frame_name','laser_frame')
    plugin=add(model,'plugin',name='ideal_planar_base',filename='libgazebo_ros_planar_move.so')
    for k,v in {'update_rate':50,'publish_rate':20,'odometry_frame':'odom','robot_base_frame':'sim_base', 'publish_odom':'true','publish_odom_tf':'true'}.items(): add(plugin,k,v)
    tree.write(target/'cafe.world',encoding='unicode')
    common={'common/levels':[1.0,1.8],'common/odom_frame':'odom','use_sim_time':True}
    generator={**common,'dwal_generator/odometryTopic':'/odom','dwal_generator/occ_topic':'/local_costmap/costmap', 'dwal_generator/base_frame':'sim_base','dwal_generator/footprint':[v for p in footprint for v in p], 'dwal_generator/footprint_padding':0.02,'dwal_generator/acc_lim_x':0.5,'dwal_generator/acc_lim_th':1.0,'dwal_generator/max_trans_vel':0.3,'dwal_generator/min_trans_vel':0.1,'dwal_generator/max_vel_theta':0.8,'dwal_generator/sim_period':0.2,'dwal_generator/DS':0.05,'dwal_generator/Kmax':4.0,'dwal_generator/alpha':0.05,'dwal_generator/Hz':10.0}
    clustering={**common,'dwal_clustering/postfix':['near','far'],'dwal_clustering/spin':[1,1],'dwal_clustering/min_cluster_span':0.2,'dwal_clustering/cluster_separation':5,'dwal_clustering/subsample_step':3}
    (target/'dwal.yaml').write_text(dump_yaml({'/dwal_planner/dwal_generator':{'ros__parameters':generator},'/dwal_planner/dwal_clustering':{'ros__parameters':clustering}}))
    costmap={'use_sim_time':True,'global_frame':'odom','robot_base_frame':'sim_base','rolling_window':True,'width':10,'height':10,'resolution':0.05,'update_frequency':10.0,'publish_frequency':10.0,'always_send_full_costmap':True,'track_unknown_space':False,'transform_tolerance':0.5,'footprint':str(footprint),'footprint_padding':0.02,'plugins':['obstacle_layer','inflation_layer'],'obstacle_layer':{'plugin':'nav2_costmap_2d::ObstacleLayer','enabled':True,'observation_sources':'scan','scan':{'topic':'/scan','data_type':'LaserScan','clearing':True,'marking':True,'max_obstacle_height':2.0,'raytrace_max_range':10.0,'obstacle_max_range':9.5,'inf_is_valid':True}},'inflation_layer':{'plugin':'nav2_costmap_2d::InflationLayer','inflation_radius':0.45,'cost_scaling_factor':8.0}}
    (target/'costmap.yaml').write_text(dump_yaml({'/costmap/costmap':{'ros__parameters':costmap}}))
    (target/'geometry.yaml').write_text(dump_yaml({'footprint':footprint,'laser_xyz':[0.85,0,0.35],'base_link_height':height,'description':'Conservative zero-joint visual/collision envelope; provisional lidar mount included.'}))
    return target


if __name__ == '__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('description');p.add_argument('wrapper');p.add_argument('output')
    a=p.parse_args(); prepare(a.description,a.wrapper,a.output)
