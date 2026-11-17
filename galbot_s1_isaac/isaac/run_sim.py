#!/usr/bin/env python3
"""Galbot S1 table demo in Isaac Sim 6.x with the ROS 2 interface of the Gazebo version.

Isaac Sim (needs an RTX GPU; run with the Isaac Sim python, ROS 2 Jazzy sourced):
    $ISAACSIM_PATH/python.sh run_sim.py                # GUI window
    $ISAACSIM_PATH/python.sh run_sim.py --headless     # no window (web UI shows the cameras)

Kinematic stand-in (no Isaac Sim, no GPU - CI / UI development):
    python3 run_sim.py --backend kinematic

The ROS side (commander, base controller, camera relay, rosbridge, web UI) is started
separately:  ros2 launch galbot_s1_isaac bringup.launch.py   (or sim.launch.py = both)
"""
import argparse
import os
import sys
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--backend', choices=('isaac', 'kinematic'), default='isaac')
    ap.add_argument('--headless', action='store_true', help='Isaac Sim without a window')
    ap.add_argument('--config', default=None, help='settings yaml (default: config/isaac_sim.yaml)')
    ap.add_argument('--scene', default=None, help='scene yaml (default: config/scene.yaml)')
    ap.add_argument('--no-cameras', action='store_true', help='do not render / publish camera images')
    ap.add_argument('--realtime-factor', type=float, default=None, help='1 = wall clock, 0 = max speed')
    ap.add_argument('--physics-device', default=None, help='cpu | cuda:0 (overrides the settings)')
    ap.add_argument('--renderer', default='RaytracedLighting',
                    help='RaytracedLighting (fast) | RealTimePathTracing (Isaac default)')
    ap.add_argument('--rebuild-usd', action='store_true', help='re-import the URDF even if cached')
    ap.add_argument('--max-sim-time', type=float, default=None, help='stop after this many sim seconds')
    ap.add_argument('--attach-mode', choices=('follow', 'joint'), default=None)
    # Isaac Sim / Kit append their own arguments (e.g. --/app/...); ignore them
    args, _unknown = ap.parse_known_args(argv)
    return args


def log(text):
    print('[galbot_isaac] %s' % text, flush=True)


def main(argv=None):
    args = parse_args(argv)
    app = None
    if args.backend == 'isaac':
        # SimulationApp has to exist before anything from omni / isaacsim / pxr is imported
        from isaacsim import SimulationApp
        app = SimulationApp({'headless': args.headless, 'renderer': args.renderer,
                             'width': 1280, 'height': 720, 'anti_aliasing': 3})

    exit_code = 0
    backend = ros = None
    try:
        try:
            import yaml
        except ImportError:   # Isaac Sim python without PyYAML: use the distribution's (same 3.12 ABI)
            sys.path.append('/usr/lib/python3/dist-packages')
            import yaml
        from galbot_isaac import config
        config.ensure_control_importable()
        from galbot_s1_control import kinematics
        from galbot_s1_control import robot_config as rc
        from galbot_s1_control.scene import Scene

        settings = config.load_settings(args.config)
        if args.no_cameras:
            settings['cameras']['enabled'] = False
        if args.realtime_factor is not None:
            settings['realtime_factor'] = args.realtime_factor
        if args.physics_device:
            settings['physics']['device'] = args.physics_device
        if args.attach_mode:
            settings['robot']['attach_mode'] = args.attach_mode
        scene_file = args.scene or config.default_scene_file()
        with open(scene_file) as f:
            raw_scene = yaml.safe_load(f)
        scene = Scene(raw_scene)
        scene.raw = raw_scene
        log('scene %s, settings %s' % (scene_file, args.config or 'default'))

        if args.backend == 'isaac':
            from galbot_isaac.backend_isaac import IsaacBackend
            backend = IsaacBackend(app, settings, scene, rc, kinematics, headless=args.headless,
                                   rebuild_usd=args.rebuild_usd, log=log)
        else:
            from galbot_isaac.backend_kinematic import KinematicBackend
            with open(config.description_urdf_file()) as f:
                backend = KinematicBackend(settings, scene, f.read(), rc, kinematics, log=log)
        backend.start()

        if args.backend == 'isaac':
            ensure_rclpy()
        from galbot_isaac.geometry import camera_intrinsics
        from galbot_isaac.ros_io import RosIO
        from galbot_isaac.sim_core import SimCore, controller_table

        cams = {name: (w, h, camera_intrinsics(w, h, hfov))
                for name, (w, h, hfov, _rate) in backend.camera_specs().items()}
        ros = RosIO(list(controller_table(rc)), backend.object_names(), cams)
        core = SimCore(backend, ros, settings, scene.robot_name, rc, log=log)
        log('ready: publishing /clock, /joint_states, /galbot/ground_truth/odom, objects %s, cameras %s'
            % (backend.object_names(), sorted(cams)))
        core.run(max_sim_time=args.max_sim_time)
        log('finished after %d frames (sim %.1fs)' % (core.frames, backend.sim_time()))
    except KeyboardInterrupt:
        log('interrupted')
    except Exception:
        traceback.print_exc()
        exit_code = 1
    finally:
        if ros is not None:
            ros.shutdown()
        if backend is not None:
            try:
                backend.close()
            except Exception:
                traceback.print_exc()
        if app is not None:
            time.sleep(0.2)
            app.close()   # exits the process
    return exit_code


def ensure_rclpy():
    """System ROS 2 (sourced before starting Isaac Sim) or the libraries bundled with Isaac Sim."""
    try:
        import rclpy  # noqa: F401
        return
    except ImportError as exc:
        log('rclpy not importable (%s) - enabling isaacsim.ros2.bridge (bundled ROS 2 libraries)' % exc)
    import isaacsim.core.experimental.utils.app as app_utils
    app_utils.enable_extension('isaacsim.ros2.bridge')
    app_utils.update_app(steps=2)
    import rclpy  # noqa: F401,F811


if __name__ == '__main__':
    sys.exit(main())
