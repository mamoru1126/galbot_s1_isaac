"""Isaac Sim + ROS 2 side + web UI in one command.

    ros2 launch galbot_s1_isaac sim.launch.py                     # Isaac Sim with window
    ros2 launch galbot_s1_isaac sim.launch.py headless:=true      # Isaac Sim without window
    ros2 launch galbot_s1_isaac sim.launch.py backend:=kinematic  # no Isaac Sim (stand-in)

The Isaac Sim python is taken from $ISAACSIM_PYTHON, else $ISAACSIM_PATH/python.sh,
else /isaac-sim/python.sh (the NGC container). Web UI: http://<host>:8000
"""
import importlib.util
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

_spec = importlib.util.spec_from_file_location(
    'galbot_s1_isaac_bringup_launch', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'bringup.launch.py'))
_bringup = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_bringup)
declare_arguments = _bringup.declare_arguments


def isaac_python():
    for cand in (os.environ.get('ISAACSIM_PYTHON'),
                 os.path.join(os.environ['ISAACSIM_PATH'], 'python.sh') if os.environ.get('ISAACSIM_PATH') else None,
                 '/isaac-sim/python.sh'):
        if cand and os.path.exists(cand):
            return cand
    raise RuntimeError('Isaac Sim python not found: set ISAACSIM_PATH (folder with python.sh) '
                       'or ISAACSIM_PYTHON (e.g. the python of a pip installed isaacsim venv)')


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    share = get_package_share_directory('galbot_s1_isaac')
    backend = arg('backend')
    cmd = [isaac_python() if backend == 'isaac' else sys.executable,
           os.path.join(share, 'isaac', 'run_sim.py'), '--backend', backend]
    if arg('headless').lower() == 'true':
        cmd.append('--headless')
    if arg('cameras').lower() != 'true':
        cmd.append('--no-cameras')
    if arg('scene_file'):
        cmd += ['--scene', arg('scene_file')]
    if arg('sim_args'):
        cmd += arg('sim_args').split()
    sim = ExecuteProcess(cmd=cmd, name='isaac_sim', output='screen')
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'bringup.launch.py')),
        launch_arguments={k: arg(k) for k in ('cameras', 'web', 'web_port', 'rosbridge_port', 'rviz',
                                              'scene_file')}.items())
    return [sim, bringup,
            RegisterEventHandler(OnProcessExit(target_action=sim, on_exit=[EmitEvent(event=Shutdown(
                reason='simulator exited'))]))]


def generate_launch_description():
    return LaunchDescription(declare_arguments() + [
        DeclareLaunchArgument('backend', default_value='isaac', description='isaac | kinematic'),
        DeclareLaunchArgument('headless', default_value='false', description='Isaac Sim without a window'),
        DeclareLaunchArgument('sim_args', default_value='', description='extra arguments for run_sim.py'),
        OpaqueFunction(function=launch_setup),
    ])
