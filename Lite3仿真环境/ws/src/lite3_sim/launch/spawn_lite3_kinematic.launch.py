import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('lite3_sim')
    gazebo_share = get_package_share_directory('gazebo_ros')

    world = os.path.join(pkg_share, 'worlds', 'empty.world')
    xacro_file = os.path.join(pkg_share, 'urdf', 'Lite3_kinematic.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    gui = LaunchConfiguration('gui')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world,
            'verbose': 'true',
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(gui),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
        ],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'lite3',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.36',
        ],
    )

    robot_pose = Node(
        package='lite3_sim',
        executable='arm_pose.py',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
        robot_pose,
    ])
