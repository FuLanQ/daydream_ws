import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # 1. 启动小车底盘 (通讯 + EKF + URDF)
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('my_robot_base'), 'launch', 'base_bringup.launch.py')
        )
    )

    # 2. 启动激光雷达
    # 👇【再次提醒】换成你真实的雷达包名和 launch 名 (跟刚刚建图脚本里的一样)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('lslidar_driver'), 'launch', 'lslidar_x10_launch.py')
        )
    )

    # 3. 启动 Nav2 导航框架 (定位 + 路径规划 + 避障)
    # 假设你之前把地图保存在了工作空间根目录，取名叫 my_room_map.yaml
    map_file_path = '/app/daydream_ws/src/my_robot_base/maps/my_map.yaml'
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            # 注意：导航调用的不是 slam_toolbox，而是 nav2_bringup 里的 bringup_launch.py
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file_path,        # 将你的地图喂给导航系统
            'use_sim_time': 'false'      # 真车必须设为 false
        }.items()
    )

    return LaunchDescription([
        base_launch,
        lidar_launch,
        nav2_launch
    ])