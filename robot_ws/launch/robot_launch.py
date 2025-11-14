from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('pi_bot')

    return LaunchDescription([
        # 1. 传感器 + 串口桥
        Node(package='pi_bot', executable='lidar_node',    name='lidar'),
        Node(package='pi_bot', executable='camera_node',  name='camera'),
        Node(package='pi_bot', executable='esp32_interface',name='esp32'),
        # 2. 工具
        Node(package='pi_bot', executable='qr_decoder',   name='qr_decoder'),
        # 3. 业务节点
        Node(package='pi_bot', executable='task_node',    name='task_node'),
        # 4. 导航栈（静态地图）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_dir+'/launch/nav2_static.launch.py')),
        # 5. QR 扫描记录 -> PDF
        Node(package='pi_bot', executable='qr_logger', name='qr_logger', output='screen'),
    ])