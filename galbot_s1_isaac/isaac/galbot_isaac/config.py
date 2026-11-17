"""Paths and settings of the Isaac Sim app (no Isaac / ROS imports here)."""
import copy
import os
import re
import sys

import yaml

ISAAC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # .../galbot_s1_isaac/isaac
PKG_DIR = os.path.dirname(ISAAC_DIR)                                     # galbot_s1_isaac (src or share)

PLANAR_JOINTS = ('base_x_joint', 'base_y_joint', 'base_yaw_joint')

DEFAULTS = {
    'physics': {'dt': 1.0 / 120.0, 'device': 'cpu', 'solver': 'TGS',
                'position_iterations': 32, 'velocity_iterations': 1},
    'realtime_factor': 1.0,
    'robot': {'prim_path': '/World/galbot_s1', 'usd_cache_dir': '', 'disable_gravity': True,
              'attach_mode': 'follow', 'drives': []},
    'cameras': {'enabled': True, 'robot': {}, 'overview_rate': 10, 'near': 0.03, 'far': 20.0},
    'publish': {'joint_states_rate': 60, 'odom_rate': 50, 'objects_rate': 20},
}


def _merge(base, override):
    out = copy.deepcopy(base)
    for k, v in (override or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _merge(out[k], v)
        else:
            out[k] = v
    return out


def load_settings(path=None):
    path = path or os.path.join(PKG_DIR, 'config', 'isaac_sim.yaml')
    with open(path) as f:
        return _merge(DEFAULTS, yaml.safe_load(f) or {})


def default_scene_file():
    return os.path.join(PKG_DIR, 'config', 'scene.yaml')


def isaac_urdf_file():
    return os.path.join(PKG_DIR, 'urdf', 'galbot_s1_isaac.urdf')


def description_urdf_file():
    return os.path.join(PKG_DIR, 'urdf', 'galbot_s1.urdf')


def _candidates_next_to(pkg_dir, name):
    parent = os.path.dirname(pkg_dir)
    yield os.path.join(parent, name)                                   # source tree / merged install share
    # isolated colcon install: <ws>/install/galbot_s1_isaac/share/galbot_s1_isaac
    install = os.path.dirname(os.path.dirname(parent))
    yield os.path.join(install, name, 'share', name)


def find_package_dir(name, env_var=None):
    if env_var and os.environ.get(env_var):
        return os.environ[env_var]
    for path in _candidates_next_to(PKG_DIR, name):
        if os.path.isdir(path):
            return path
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(name)
    except Exception:
        pass
    raise RuntimeError('package %s not found (set %s)' % (name, env_var or 'its path'))


def description_dir():
    return find_package_dir('galbot_s1_description', 'GALBOT_DESCRIPTION_DIR')


def ensure_control_importable():
    """galbot_s1_control (pure python parts) from the colcon install or the source tree."""
    try:
        import galbot_s1_control  # noqa: F401
        return
    except ImportError:
        pass
    for path in _candidates_next_to(PKG_DIR, 'galbot_s1_control'):
        if os.path.isfile(os.path.join(path, 'galbot_s1_control', 'robot_config.py')):
            sys.path.insert(0, path)
            return
    raise RuntimeError('galbot_s1_control python package not found')


def cache_dir(settings):
    d = settings['robot'].get('usd_cache_dir') or ''
    if not d:
        base = os.environ.get('XDG_CACHE_HOME') or os.path.join(os.path.expanduser('~'), '.cache')
        d = os.path.join(base, 'galbot_s1_isaac')
    return os.path.abspath(os.path.expanduser(d))


def drive_for_joint(settings, joint):
    """Resolved drive settings of one joint (later `drives` entries override earlier ones)."""
    out = {'mode': 'position', 'stiffness': 0.0, 'damping': 0.0, 'max_effort': None}
    for entry in settings['robot'].get('drives', []):
        if re.search(entry['pattern'], joint):
            for k in ('mode', 'stiffness', 'damping', 'max_effort'):
                if k in entry:
                    out[k] = entry[k]
    return out
