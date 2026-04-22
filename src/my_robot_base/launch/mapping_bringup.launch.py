import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # 1. 启动小车底盘 (通讯 + EKF + URDF)
    # 假设你的底盘包叫 my_robot_base，启动文件叫 base_bringup.launch.py
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('my_robot_base'), 'launch', 'base_bringup.launch.py')
        )
    )

    # 2. 启动激光雷达
    # 👇【注意】这里需要换成你自己雷达的包名和启动文件名！
    # 下面以思岚雷达 (sllidar_ros2 和 sllidar_launch.py) 为例：
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('lslidar_driver'), 'launch', 'lslidar_x10_launch.py')
        )
    )

    # 3. 启动 SLAM Toolbox 并加载你的专属参数文件
    slam_params_file = '/app/daydream_ws/src/my_robot_base/config/my_slam.yaml'
    
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        # 这就相当于在命令行里敲 slam_params_file:=...
        launch_arguments={'slam_params_file': slam_params_file}.items()
    )

    # 将上面三个组件打包成一个启动描述符返回
    return LaunchDescription([
        base_launch,
        lidar_launch,
        slam_launch
    ])