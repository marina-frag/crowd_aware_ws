"""Offline description-to-SDF regression checks; no Gazebo is started."""
import importlib.util
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import xacro

script = Path(__file__).resolve().parents[1] / 'scripts/prepare_scene.py'
spec = importlib.util.spec_from_file_location('scene', script)
scene = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scene)
if len(sys.argv) > 1:
    description = Path(sys.argv.pop(1)).resolve()
else:
    from ament_index_python.packages import get_package_share_directory
    description = Path(get_package_share_directory('iwalk_description'))


class RobotGeometry(unittest.TestCase):
    def setUp(self):
        self.robot = ET.fromstring(xacro.process_file(str(description/'urdf/iwalk.urdf.xacro')).toxml())
        self.frames = scene.robot_frames(self.robot)

    def test_mount_and_existing_physical_frames(self):
        np.testing.assert_allclose(self.frames['laser_frame'][:3, 3], [0.885, 0, 0.35])
        np.testing.assert_allclose(self.frames['laser_frame'][:3, :3], np.eye(3))
        np.testing.assert_allclose(self.frames['base_link'][:3, 3], [0, 0, 0.095])
        np.testing.assert_allclose(self.frames['base_footprint'][:3, 3], [0.39, 0, 0], atol=1e-12)
        for side, y in [('left', 0.305), ('right', -0.305)]:
            np.testing.assert_allclose(self.frames[f'{side}_rear_wheel_link'][:3, 3], [0, y, 0.095])

    def test_sdf_preserves_each_collision_geometry_and_pose(self):
        chassis = ET.Element('link', name='chassis')
        scene.append_description_geometry(chassis, self.robot, self.frames, description)
        originals = [(link, element) for link in self.robot.findall('link') for element in link.findall('collision')]
        copied = chassis.findall('collision')
        self.assertEqual(len(originals), len(copied))
        self.assertFalse(any(e.get('name') == 'navigation_envelope' for e in copied))
        for (link, source), target in zip(originals, copied):
            expected = ET.Element('collision')
            scene.sdf_geometry(expected, list(source.find('geometry'))[0], description)
            self.assertEqual(ET.tostring(expected.find('geometry')), ET.tostring(target.find('geometry')))
            tf = self.frames[link.get('name')] @ scene.transform(source.find('origin'))
            np.testing.assert_allclose(list(map(float, target.findtext('pose').split())), list(map(float, scene.pose(tf).split())))

    def test_body_uses_physical_mesh_instead_of_filled_box(self):
        collision = self.robot.find("link[@name='base_link']/collision[@name='iwalk_body_collision']")
        self.assertIsNotNone(collision.find('geometry/mesh'))
        self.assertIsNone(collision.find('geometry/box'))
        points = scene.geometry_points(collision.find('geometry/mesh'), description)
        self.assertTrue(np.isfinite(points).all())
        self.assertGreater(len(points), 0)


if __name__ == '__main__':
    unittest.main()
