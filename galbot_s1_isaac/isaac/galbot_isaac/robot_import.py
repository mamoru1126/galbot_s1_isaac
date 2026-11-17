"""URDF -> USD with the Isaac Sim 6.x URDF importer, cached on disk.

The cache key covers the URDF text, the drive settings and the importer options, so
editing config/isaac_sim.yaml `robot.drives` or regenerating the URDF re-imports.
"""
import hashlib
import json
import os
import shutil
import time

from .config import drive_for_joint

IMPORT_VERSION = 1   # bump when the import options below change


def _importer():
    import isaacsim.core.experimental.utils.app as app_utils
    for ext in ('omni.scene.optimizer.core', 'isaacsim.robot.schema', 'isaacsim.asset.importer.urdf'):
        try:
            app_utils.enable_extension(ext)
        except Exception as exc:  # optional helpers may not exist in every release
            print('[galbot_isaac] enable %s: %s' % (ext, exc), flush=True)
    app_utils.update_app(steps=2)
    try:
        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    except ImportError:
        from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig
    return URDFImporter, URDFImporterConfig


def _joint_names(urdf_text):
    import xml.etree.ElementTree as ET
    root = ET.fromstring(urdf_text)
    return [j.get('name') for j in root.findall('joint') if j.get('type') in ('revolute', 'continuous', 'prismatic')]


def drive_overrides(settings, urdf_text):
    """Per-joint (exact name) drive settings for the importer."""
    target, stiffness, damping = {}, {}, {}
    for name in _joint_names(urdf_text):
        d = drive_for_joint(settings, name)
        key = '^%s$' % name
        target[key] = d['mode']
        stiffness[key] = float(d['stiffness'])
        damping[key] = float(d['damping'])
    return target, stiffness, damping


def ensure_robot_usd(settings, urdf_path, description_dir, cache_root, rebuild=False, log=print):
    with open(urdf_path) as f:
        urdf_text = f.read()
    target, stiffness, damping = drive_overrides(settings, urdf_text)
    options = {
        'version': IMPORT_VERSION,
        'asset_transformer': bool(settings['robot'].get('asset_transformer', True)),
        'target': target, 'stiffness': stiffness, 'damping': damping,
    }
    key = hashlib.sha1((urdf_text + json.dumps(options, sort_keys=True)).encode()).hexdigest()[:12]
    out_dir = os.path.join(cache_root, 'robot_' + key)
    marker = os.path.join(out_dir, 'import.json')
    if not rebuild and os.path.isfile(marker):
        with open(marker) as f:
            usd = json.load(f).get('usd')
        if usd and os.path.isfile(usd):
            log('robot USD (cached): %s' % usd)
            return usd
    if os.path.isdir(out_dir):
        shutil.rmtree(out_dir, ignore_errors=True)
    os.makedirs(out_dir)

    URDFImporter, URDFImporterConfig = _importer()
    cfg = URDFImporterConfig()
    cfg.urdf_path = os.path.abspath(urdf_path)
    cfg.usd_path = out_dir
    cfg.merge_fixed_joints = False
    cfg.merge_mesh = False
    cfg.collision_from_visuals = False
    cfg.allow_self_collision = False
    cfg.ros_package_paths = [{'name': 'galbot_s1_description', 'path': os.path.abspath(description_dir)}]
    cfg.fix_base = True                 # isaac_world (root of the planar base joints) is anchored
    cfg.joint_drive_type = 'force'
    cfg.joint_target_type = target
    cfg.override_joint_stiffness = stiffness
    cfg.override_joint_damping = damping
    cfg.run_asset_transformer = options['asset_transformer']
    log('importing %s (first start only, takes a minute) ...' % urdf_path)
    t0 = time.time()
    usd = URDFImporter(cfg).import_urdf()
    if not usd or not os.path.isfile(usd):
        raise RuntimeError('URDF import produced no USD (got %r)' % usd)
    with open(marker, 'w') as f:
        json.dump({'usd': usd, 'urdf': os.path.abspath(urdf_path), 'options': options,
                   'seconds': round(time.time() - t0, 1)}, f, indent=1)
    log('robot USD: %s (%.0f s)' % (usd, time.time() - t0))
    return usd
