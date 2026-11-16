"""Validate the complete bridge declarations and sensor references."""

import ast
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def test_sensor_bridges_defined():
    tree = ast.parse((ROOT / 'launch/sim.launch.py').read_text())
    strings = {node.value for node in ast.walk(tree)
               if isinstance(node, ast.Constant) and isinstance(node.value, str)}
    expected = {
        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        '/bumper_states@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
        '/sonar/range@sensor_msgs/msg/Range[gz.msgs.LaserScan',
        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
    }
    assert expected <= strings


def test_sonar_has_link_and_no_classic_plugin():
    robot = ET.parse(ROOT / 'description/robot.urdf.xacro').getroot()
    assert robot.find("./link[@name='sonar_link']") is not None
    assert robot.find("./joint/child[@link='sonar_link']") is not None
    sensor = robot.find("./gazebo[@reference='sonar_link']/sensor")
    assert sensor.attrib['type'] == 'gpu_lidar'
    assert sensor.find('plugin') is None


def test_world_has_sensor_systems():
    world = ET.parse(ROOT / 'description/road.sdf').getroot().find('world')
    plugins = {plugin.attrib['name'] for plugin in world.findall('plugin')}
    assert {'gz::sim::systems::Sensors', 'gz::sim::systems::Imu'} <= plugins
