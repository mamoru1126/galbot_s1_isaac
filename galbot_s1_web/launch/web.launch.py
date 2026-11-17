"""ROS 2: rosbridge + web console.  ros2 launch galbot_s1_web web.launch.py"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    web_root = os.path.join(get_package_share_directory('galbot_s1_web'), 'web')
    port = LaunchConfiguration('port')
    rb_port = LaunchConfiguration('rosbridge_port')
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8000'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090'),
        Node(package='rosbridge_server', executable='rosbridge_websocket', name='rosbridge_websocket',
             output='screen', parameters=[{'port': rb_port, 'address': '', 'use_sim_time': True}]),
        Node(package='galbot_s1_web', executable='web_server.py', name='galbot_web_server', output='screen',
             arguments=['--root', web_root, '--port', port, '--rosbridge-port', rb_port]),
    ])
