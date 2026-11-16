from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cmd_vel_guard = Node(
        package='lite3_inspection',
        executable='cmd_vel_guard',
        name='cmd_vel_guard',
        output='screen',
        parameters=[{
            'input_topic': '/inspection/cmd_vel_raw',
            'output_topic': '/cmd_vel',
            'enable_topic': '/inspection/motion_enable',
            'publish_hz': 20.0,
            'timeout_sec': 0.25,
            'max_linear_x': 0.30,
            'max_linear_y': 0.15,
            'max_angular_z': 0.50,
            'start_enabled': True,
        }],
    )

    arm_serial_bridge = Node(
        package='lite3_inspection',
        executable='arm_serial_bridge',
        name='arm_serial_bridge',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB0',
            'baudrate': 115200,
            'dry_run': True,
            'command_topic': '/arm/command',
            'status_topic': '/arm/status',
        }],
    )

    inspection_manager = Node(
        package='lite3_inspection',
        executable='inspection_manager',
        name='inspection_manager',
        output='screen',
        parameters=[{
            'detections_topic': '/inspection/detections',
            'event_topic': '/inspection/event',
            'arm_command_topic': '/arm/command',
            'motion_enable_topic': '/inspection/motion_enable',
            'target_classes': ['anomaly', 'block'],
            'minimum_confidence': 0.65,
            'stable_frames': 5,
            'detection_reset_sec': 1.0,
            'auto_pick': False,
        }],
    )

    return LaunchDescription([
        cmd_vel_guard,
        arm_serial_bridge,
        inspection_manager,
    ])
