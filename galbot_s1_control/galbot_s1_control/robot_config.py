"""Joint groups, controllers and named poses of Galbot S1 (shared by all nodes)."""

ARM_JOINTS = {
    'left': ['left_arm_joint%d' % i for i in range(1, 8)],
    'right': ['right_arm_joint%d' % i for i in range(1, 8)],
}
GRIPPER_JOINTS = {
    side: ['%s_active_joint1' % side, '%s_active_joint2' % side, '%s_passive_joint' % side]
    for side in ('left', 'right')
}
TORSO_JOINTS = ['torso_lift_joint1']
HEAD_JOINTS = ['head_joint1', 'head_joint2']
STEERING_JOINTS = ['wheel_steering_joint%d' % i for i in range(1, 5)]
WHEEL_JOINTS = ['wheel_driving_joint%d' % i for i in range(1, 5)]

# group name -> (controller name, joints)
GROUPS = {
    'left_arm': ('left_arm_controller', ARM_JOINTS['left']),
    'right_arm': ('right_arm_controller', ARM_JOINTS['right']),
    'left_gripper': ('left_gripper_controller', GRIPPER_JOINTS['left']),
    'right_gripper': ('right_gripper_controller', GRIPPER_JOINTS['right']),
    'torso': ('torso_controller', TORSO_JOINTS),
    'head': ('head_controller', HEAD_JOINTS),
}

# Wheel module positions in base_link (x, y) - from the URDF steering joints.
WHEEL_POSITIONS = [(0.247, -0.247), (0.247, 0.247), (-0.247, -0.247), (-0.247, 0.247)]

ROBOT_BASE_FRAME = 'base_footprint'

# Symmetric arm configuration with both grippers in front of the chest pointing
# down (found with the IK in kinematics.py, lift = 0.2 m). Left = -Right.
_READY_RIGHT = [-1.275, -1.355, 0.86, 1.737, 1.659, 0.267, -1.164]
_READY_LEFT = [-v for v in _READY_RIGHT]

POSES = {
    # name: {group: positions}
    'ready': {
        'torso': [0.2],
        'head': [0.0, -0.35],
        'right_arm': _READY_RIGHT,
        'left_arm': _READY_LEFT,
    },
    'ready_high': {
        'torso': [0.5],
        'head': [0.0, -0.45],
        'right_arm': _READY_RIGHT,
        'left_arm': _READY_LEFT,
    },
    'zero': {
        'torso': [0.0],
        'head': [0.0, 0.0],
        'right_arm': [0.0] * 7,
        'left_arm': [0.0] * 7,
    },
}

# Initial joint positions in simulation (used by the launch files / xacro too).
INITIAL_POSITIONS = {}
for _g, _vals in POSES['ready'].items():
    for _n, _v in zip(GROUPS[_g][1], _vals):
        INITIAL_POSITIONS[_n] = _v
