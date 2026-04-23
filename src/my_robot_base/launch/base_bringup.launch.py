import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_base_dir = get_package_share_directory('my_robot_base')
    urdf_pkg_dir = get_package_share_directory('wheeltec_robot_urdf')

    urdf_file = os.path.join(urdf_pkg_dir, 'urdf', 'mini_mec_robot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    base_node = Node(
        package='my_robot_base',
        executable='serial_bridge_node',
        name='daydream_base_node',
        output='screen',
        parameters=[{
            'port': '/dev/ttyACM0',   
            'baudrate': 115200,
            'publish_tf': False,      
            'cmd_timeout_ms': 500,
            'max_linear_vel': 1.5,
            'max_angular_vel': 3.0,
            'min_wheel_vel': 500.0
        }]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    ekf_config_path = os.path.join(pkg_base_dir, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )

    # ================= ⚡核心修复区域⚡ =================
    # 1. 修复 footprint：以 base_link 为父，向下关联 base_footprint (避免双亲冲突)
    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='footprint_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )

    # 2. 修复 雷达断连：以 base_link 为父，向上关联 laser_link
    # 假设你的雷达装在小车中心正上方约 0.15 米处 (Z=0.15)
    laser_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf_publisher',
        arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'laser_link']
    )
    # ===================================================

    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': 320,
            'image_height': 240,
            'framerate': 10.0,
            'pixel_format': 'yuyv',
            'camera_name': 'default_cam',
            'io_method': 'mmap'
        }]
    )

    return LaunchDescription([
        base_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        ekf_node,
        footprint_publisher, # 发布投影
        laser_tf_publisher,  # 发布雷达位置，拼接坐标树
        camera_node
    ])