import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('ros_gz_sim')
    pkg_desc = get_package_share_directory('hexapod_description')
    pkg_hexapod_gazebo = get_package_share_directory('hexapod_gazebo')

    xacro_file = os.path.join(pkg_desc, 'urdf', 'hexapod.urdf.xacro')
    world_file = os.path.join(pkg_hexapod_gazebo, 'worlds', 'hexapod_world.sdf')
    controllers_yaml = os.path.join(pkg_hexapod_gazebo, 'config', 'ros2_controllers.yaml')

    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    return LaunchDescription([
        # 1. Start Gazebo (world loads the Sensors system so the camera renders)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': f'-r {world_file}'}.items()
        ),
        # 2. Bridge clock and camera (Gazebo -> ROS 2)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
        # 3. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': True
            }]
        ),
        # 4. Spawn Robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'hexapod', '-z', '0.3'],
            output='screen'
        ),
        # 5. Controller Spawners with explicit parameter files
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--param-file', controllers_yaml],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['hexapod_joint_controller', '--param-file', controllers_yaml],
            output='screen'
        )
    ])
