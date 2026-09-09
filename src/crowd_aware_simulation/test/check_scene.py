"""Offline physical-frame and configuration checks. Pass description, wrapper paths."""
import sys,tempfile,importlib.util,xml.etree.ElementTree as E
from pathlib import Path
import numpy as np
import xacro,yaml
script=Path(__file__).resolve().parents[1]/'scripts/prepare_scene.py'
spec=importlib.util.spec_from_file_location('scene',script);s=importlib.util.module_from_spec(spec);spec.loader.exec_module(s)
description,wrapper=map(Path,sys.argv[1:])

def frames(root):
    children={j.find('child').get('link') for j in root.findall('joint')}
    names={l.get('name') for l in root.findall('link')}
    roots=names-children
    assert len(roots)==1, roots
    result={roots.pop():np.eye(4)}
    while len(result)<len(names):
        old=len(result)
        for j in root.findall('joint'):
            a,b=j.find('parent').get('link'),j.find('child').get('link')
            if a in result: result[b]=result[a]@s.transform(j.find('origin'))
        assert len(result)>old,'Disconnected TF tree'
    return result

with tempfile.TemporaryDirectory() as t:
    p=s.prepare(description,wrapper,t)
    original=E.fromstring(xacro.process_file(str(description/'urdf/iwalk.urdf.xacro')).toxml())
    generated=E.parse(p/'robot.urdf').getroot()
    before,after=frames(original),frames(generated)
    # Every physical relative transform must survive the reroot, including the existing footprint.
    for name in before:
        np.testing.assert_allclose(np.linalg.inv(before['base_link'])@before[name],np.linalg.inv(after['base_link'])@after[name],atol=1e-10)
    np.testing.assert_allclose(after['base_link'][:2,3],[0,0],atol=1e-10)
    for config in ['dwal.yaml','costmap.yaml']:
        events=list(yaml.parse((p/config).read_text()))
        assert not any(isinstance(e,yaml.AliasEvent) or getattr(e,'anchor',None) for e in events), 'ROS parameters must not contain YAML anchors/aliases'
    dwal=yaml.safe_load((p/'dwal.yaml').read_text())['/dwal_planner/dwal_generator']['ros__parameters']
    cm=yaml.safe_load((p/'costmap.yaml').read_text())['/costmap/costmap']['ros__parameters']
    fp=np.array(yaml.safe_load(cm['footprint']))
    assert list(fp.flat)==dwal['dwal_generator/footprint']
    assert cm['always_send_full_costmap'] is True
    assert cm['global_frame']==dwal['common/odom_frame']=='odom'
    assert cm['robot_base_frame']==dwal['dwal_generator/base_frame']=='sim_base'
    for link in generated.findall('link'):
        for e in [*link.findall('visual'),*link.findall('collision')]:
            pts=s.geometry_points(list(e.find('geometry'))[0],description)
            tf=after[link.get('name')]@s.transform(e.find('origin'))
            xy=(pts@tf[:3,:3].T+tf[:3,3])[:,:2]
            assert np.all(xy>=fp.min(axis=0)-1e-9) and np.all(xy<=fp.max(axis=0)+1e-9)
    world=E.parse(p/'cafe.world').getroot().find('world')
    assert len(world.findall('include'))==len(E.parse(wrapper/'worlds/cafe.world').getroot().find('world').findall('include'))
    model=world.find("model[@name='iwalk']")
    assert model.find("plugin[@name='ideal_planar_base']/robot_base_frame").text=='sim_base'
    assert model.find("link/sensor/plugin/frame_name").text=='laser_frame'
    print('PASS: physical transforms preserved, unique TF root, shared conservative footprint, odometry/costmap frames, full map updates, cafe preserved, sensor/odom plugins.')
