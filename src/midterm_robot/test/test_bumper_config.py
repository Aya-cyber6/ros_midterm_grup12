"""Check contact sensor configuration and, when available, URDF conversion."""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


class BumperConfigurationTests(unittest.TestCase):
    def test_bumper_preserved_with_inertia_and_contact_reference(self):
        robot = ET.parse(ROOT / 'description/robot.urdf.xacro').getroot()
        bumper = robot.find("link[@name='bumper_link']")
        self.assertGreater(float(bumper.find('inertial/mass').get('value')), 0)
        self.assertIsNone(bumper.find('collision').get('name'))
        self.assertEqual(robot.findtext(
            "gazebo[@reference='bumper_joint']/preserveFixedJoint"), 'true')
        self.assertEqual(robot.findtext(
            "gazebo[@reference='bumper_link']/sensor/contact/collision"),
            'bumper_link_collision')
        world = ET.parse(ROOT / 'description/road.sdf').getroot().find('world')
        self.assertIsNotNone(world.find("plugin[@name='gz::sim::systems::Contact']"))

    @unittest.skipUnless(shutil.which('xacro') and shutil.which('gz'),
                         'Requires the ROS/Gazebo command-line tools')
    def test_converted_sdf_contact_references_an_existing_collision(self):
        urdf = subprocess.run(
            ['xacro', str(ROOT / 'description/robot.urdf.xacro')],
            check=True, capture_output=True, text=True).stdout
        with tempfile.TemporaryDirectory() as folder:
            file = Path(folder) / 'robot.urdf'
            file.write_text(urdf)
            sdf = subprocess.run(['gz', 'sdf', '-p', str(file)],
                                 check=True, capture_output=True, text=True).stdout
        root = ET.fromstring(sdf)
        bumper = root.find("model/link[@name='bumper_link']")
        self.assertIsNotNone(bumper, 'Bumper was lost during fixed-joint reduction')
        collision = bumper.find('sensor/contact/collision').text
        self.assertIsNotNone(bumper.find("collision[@name='" + collision + "']"))


if __name__ == '__main__':
    unittest.main()
