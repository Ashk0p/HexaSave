"""Check the xacro model is consistent with the controller config and camera setup."""

import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro
import yaml  # type: ignore[import-untyped]

HERE = os.path.dirname(__file__)
CONTROLLERS_YAML = os.path.join(HERE, '..', 'config', 'ros2_controllers.yaml')
WORLD = os.path.join(HERE, '..', 'worlds', 'hexapod_world.sdf')
LEGS = ['leg1_front_right', 'leg2_mid_right', 'leg3_back_right',
        'leg4_front_left', 'leg5_mid_left', 'leg6_back_left']
LEG_JOINTS = [f'{leg}_{part}_joint' for leg in LEGS for part in ('coxa', 'femur', 'tibia')]


@pytest.fixture(scope='module')
def urdf():
    path = os.path.join(get_package_share_directory('hexapod_description'),
                        'urdf', 'hexapod.urdf.xacro')
    return ET.fromstring(xacro.process_file(path).toxml())


@pytest.fixture(scope='module')
def controllers():
    with open(CONTROLLERS_YAML) as f:
        return yaml.safe_load(f)


def test_eighteen_revolute_leg_joints(urdf):
    revolute = {j.get('name') for j in urdf.findall('joint') if j.get('type') == 'revolute'}
    assert revolute == set(LEG_JOINTS)


def test_joint_limits_are_valid(urdf):
    for j in urdf.findall('joint'):
        lim = j.find('limit')
        if lim is not None:
            assert float(lim.get('lower')) < float(lim.get('upper')), j.get('name')


def test_controller_joints_match_urdf(urdf, controllers):
    yaml_joints = controllers['hexapod_joint_controller']['ros__parameters']['joints']
    rc_joints = [j.get('name') for j in urdf.find('ros2_control').findall('joint')]
    assert yaml_joints == LEG_JOINTS
    assert rc_joints == LEG_JOINTS


def test_controller_interface_is_position(urdf, controllers):
    assert controllers['hexapod_joint_controller']['ros__parameters'][
        'interface_name'] == 'position'
    for j in urdf.find('ros2_control').findall('joint'):
        assert [c.get('name') for c in j.findall('command_interface')] == ['position']


def test_camera_matches_pi_camera_v2(urdf):
    assert any(link.get('name') == 'camera_link' for link in urdf.findall('link'))
    sensor = urdf.find("gazebo[@reference='camera_link']/sensor")
    assert sensor.find('topic').text == 'camera/image_raw'
    hfov = float(sensor.find('camera/horizontal_fov').text)
    assert math.degrees(hfov) == pytest.approx(62.2, abs=0.1)


def test_world_loads_sensors_system():
    root = ET.parse(WORLD).getroot()
    plugins = [p.get('filename') for p in root.iter('plugin')]
    assert 'gz-sim-sensors-system' in plugins
