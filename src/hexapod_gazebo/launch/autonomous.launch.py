import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_hexapod_gazebo = get_package_share_directory('hexapod_gazebo')
    sim_time = {'use_sim_time': True}

    # 1. Include the base simulation stack (Gazebo, robot_state_publisher, spawners)
    base_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_hexapod_gazebo, 'launch', 'sim.launch.py')
        )
    )

    # 2. Gait Engine Node
    gait_node = Node(
        package='hexapod_controller',
        executable='gait_controller',
        parameters=[sim_time],
        output='screen'
    )

    # 3. Vision Detection Node
    detector_node = Node(
        package='hexapod_perception',
        executable='person_detector',
        parameters=[sim_time],
        output='screen'
    )

    # 4. Navigation Follower Node (compares TF stamps to its clock -> needs sim time)
    follower_node = Node(
        package='hexapod_perception',
        executable='person_follower',
        parameters=[sim_time],
        output='screen'
    )

    # Delay the high-level control nodes to ensure the TF tree and controllers are active
    delayed_logic_nodes = TimerAction(
        period=4.0,
        actions=[gait_node, detector_node, follower_node]
    )

    return LaunchDescription([
        base_sim,
        delayed_logic_nodes
    ])
