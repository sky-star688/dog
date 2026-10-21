
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_static_tf_broadcaster():
    base2lidar_tf_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base2lidar_tf_broadcaster',
        arguments=['0.16', '0.0', '0.47', '1.57', '0.0', '0.0', 'base_link', 'rslidar']
    )
    return base2lidar_tf_broadcaster

def launch_dr_nav2():
    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value='/home/ysc/lite_cog_ros2/system/map/lite3.yaml'
    )
    declare_dr_nav2_config_file = DeclareLaunchArgument(
        'dr_nav2_config_file',
        default_value=os.path.join(
            get_package_share_directory('dr_nav2'),'config','lite_nav2.yaml'
        )
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments=[
            ('map', LaunchConfiguration("map_file")),
            ('use_sim_time', 'False'),  # 'False' should be a string
            ('params_file', LaunchConfiguration('dr_nav2_config_file')),
            ('autostart', "True"),
            ('map_subscribe_transient_local', 'True')  # 'True' should be a string
        ]
    )
    return declare_map_file_cmd, declare_dr_nav2_config_file, navigation_launch

def launch_rviz2():
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('dr_nav2'), 'rviz', 'dr_nav2.rviz')],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )
    return rviz2

def generate_launch_description():
    ld = LaunchDescription()
    # static_tf_broadcaster = launch_static_tf_broadcaster()
    # ld.add_action(static_tf_broadcaster)
    declare_map_file_cmd, declare_dr_nav2_config_file, navigation_launch = launch_dr_nav2()
    ld.add_action(declare_map_file_cmd)
    ld.add_action(declare_dr_nav2_config_file)
    ld.add_action(navigation_launch)
    rviz2 = launch_rviz2()
    ld.add_action(rviz2)
    return ld
