"""
Real-robot person following: camera -> detector -> follower -> gait -> servos.

  ros2 launch hexapod_perception hexapod_robot.launch.py
  ros2 launch hexapod_perception hexapod_robot.launch.py servos:=false   # dry run
  ros2 launch hexapod_perception hexapod_robot.launch.py record:=true    # + review

servo_bridge talks to the ESP boards on /dev/ttyUSB0 (right legs) and
/dev/ttyUSB1 (left legs).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    servos = LaunchConfiguration('servos')
    record = LaunchConfiguration('record')
    camera_height = LaunchConfiguration('camera_height')

    return LaunchDescription([
        DeclareLaunchArgument('servos', default_value='true',
                              description='Start servo_bridge (false = dry run, no motion)'),
        DeclareLaunchArgument('record', default_value='false',
                              description='Start review_recorder (~/hexapod_review/<time>/)'),
        DeclareLaunchArgument('camera_height', default_value='0.15',
                              description='Camera lens height above the floor [m]'),

        Node(package='hexapod_perception', executable='pi_camera', output='screen'),
        Node(package='hexapod_perception', executable='person_detector', output='screen',
             parameters=[{'camera_height': camera_height}]),
        Node(package='hexapod_perception', executable='person_follower', output='screen'),
        Node(package='hexapod_controller', executable='gait_controller', output='screen'),
        Node(package='hexapod_controller', executable='servo_bridge', output='screen',
             condition=IfCondition(servos)),
        Node(package='hexapod_perception', executable='review_recorder', output='screen',
             parameters=[{'video_fps': 2.4}], condition=IfCondition(record)),
    ])
