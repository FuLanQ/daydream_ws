import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # 1. 抓取你原有的导航与底盘 Launch 文件
    # 假设你的包名是 my_robot_base，文件名是 navigation_bringup.launch.py
    nav_launch_dir = os.path.join(get_package_share_directory('my_robot_base'), 'launch')
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_launch_dir, 'navigation_bringup.launch.py')
        )
    )

    # 2. 启动我们写的 RFID 底层 C++ 通讯节点
    rfid_reader_node = Node(
        package='rfid_inventory',
        executable='rfid_reader',  # C++ 编译生成的可执行文件名
        name='ma60_rfid_node',
        output='screen'
    )

    # 3. 启动我们的 Python 业务调度节点
    # 注意：确保你的 Python 脚本名字和下面的 executable 一致
    patrol_logic_node = Node(
        package='rfid_inventory',
        executable='rviz_driven_patrol_node.py', # 你的 python 脚本全名
        name='rviz_driven_patrol_node',
        output='screen'
    )

    # 把这三个家伙打包在一起，同时启动！
    return LaunchDescription([
        navigation_launch,
        rfid_reader_node,
        patrol_logic_node
    ])