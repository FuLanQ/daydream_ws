import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_base_dir = get_package_share_directory('my_robot_base')
    urdf_pkg_dir = get_package_share_directory('wheeltec_robot_urdf')

    # 1. 读取 URDF 文件内容
    urdf_file = os.path.join(urdf_pkg_dir, 'urdf', 'mini_mec_robot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 2. 底层通讯节点
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

    # 3. 机器人状态发布者 (读取URDF并发布骨架 /tf_static)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    # 4. 关节状态发布者 (填补轮子处的连续关节数据，默认为0)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # 5. 卡尔曼滤波节点 (EKF)
    ekf_config_path = os.path.join(pkg_base_dir, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )

    # 6. 摄像头节点 (新增)
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': 320,
            'image_height': 240,
            'framerate': 5.0,
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
        camera_node  # 将摄像头添加到启动队列
    ])