"""ROS 2 side of the Galbot S1 Isaac Sim demo (everything except the simulator).

    ros2 launch galbot_s1_isaac bringup.launch.py

Starts robot_state_publisher, the omni base controller, the commander (teleoperation,
IK, pick-and-place tasks), the camera relay and rosbridge + the web console.
The simulator (isaac/run_sim.py) is started separately or with sim.launch.py.

Web UI: http://<host>:8000   (rosbridge ws://<host>:9090)
"""
import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def robot_description():
    desc = get_package_share_directory('galbot_s1_description')
    return xacro.process_file(os.path.join(desc, 'urdf', 'galbot_s1.urdf.xacro'),
                              mappings={'use_mimic': 'false'}).toxml()


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    share = get_package_share_directory('galbot_s1_isaac')
    scene_file = arg('scene_file') or os.path.join(share, 'config', 'scene.yaml')
    with open(scene_file) as f:
        scene = yaml.safe_load(f)
    urdf = robot_description()
    common = {'use_sim_time': True}
    actions = [
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': urdf}, common]),
        Node(package='galbot_s1_control', executable='base_controller.py', name='galbot_base_controller',
             output='screen', parameters=[common]),
        Node(package='galbot_s1_control', executable='commander.py', name='galbot_commander',
             output='screen', parameters=[common, {'robot_description': urdf, 'scene_file': scene_file,
                                                   'robot_model_name': scene['robot']['name']}]),
    ]
    if arg('cameras').lower() == 'true':
        actions.append(Node(package='galbot_s1_control', executable='camera_relay.py',
                            name='galbot_camera_relay', output='screen', parameters=[common]))
    if arg('web').lower() == 'true':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('galbot_s1_web'), 'launch', 'web.launch.py')),
            launch_arguments={'port': arg('web_port'), 'rosbridge_port': arg('rosbridge_port')}.items()))
    if arg('rviz').lower() == 'true':
        actions.append(Node(package='rviz2', executable='rviz2', parameters=[common],
                            arguments=['-d', os.path.join(get_package_share_directory('galbot_s1_description'),
                                                          'rviz', 'galbot_s1_ros2.rviz'), '-f', 'odom']))
    return actions


def declare_arguments():
    return [
        DeclareLaunchArgument('cameras', default_value='true', description='JPEG relay of the camera images'),
        DeclareLaunchArgument('web', default_value='true', description='rosbridge + web UI'),
        DeclareLaunchArgument('web_port', default_value='8000'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('scene_file', default_value='', description='scene yaml (default: config/scene.yaml)'),
    ]


def generate_launch_description():
    return LaunchDescription(declare_arguments() + [OpaqueFunction(function=launch_setup)])
