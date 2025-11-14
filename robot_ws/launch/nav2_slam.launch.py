from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('pi_bot')
    return LaunchDescription([
        Node(package='pi_bot', executable='lidar_node', name='lidar'),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             parameters=[pkg_dir+'/config/slam_params.yaml'],
             remappings=[('/scan', '/scan')]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', pkg_dir+'/config/slam.rviz']),
    ])