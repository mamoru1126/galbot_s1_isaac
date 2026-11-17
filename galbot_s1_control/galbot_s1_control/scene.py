"""Demo scene description (table, tray, objects) shared by the world generator
(Gazebo world generator / Isaac Sim scene builder) and the commander node."""
import math

import yaml


class SceneObject(object):
    def __init__(self, d):
        self.name = d['name']
        self.shape = d['shape']
        if self.shape == 'cylinder':
            self.radius = float(d['radius'])
            self.length = float(d['length'])
            self.size = (2 * self.radius, 2 * self.radius, self.length)
        elif self.shape == 'box':
            self.size = tuple(float(v) for v in d['size'])
        else:
            raise ValueError('unsupported shape %s' % self.shape)
        self.mass = float(d.get('mass', 0.2))
        self.color = tuple(float(v) for v in d.get('color', (0.8, 0.8, 0.8)))
        self.position = tuple(float(v) for v in d['position'])
        self.yaw = float(d.get('yaw', 0.0))
        self.label = d.get('label', self.name)

    @property
    def height(self):
        return self.size[2]

    def width_along(self, direction_yaw, object_yaw=0.0):
        """Extent of the object along a horizontal direction (for grasping)."""
        if self.shape == 'cylinder':
            return 2.0 * self.radius
        a = direction_yaw - object_yaw
        return abs(math.cos(a)) * self.size[0] + abs(math.sin(a)) * self.size[1]

    def grasp_yaws(self, object_yaw, preferred):
        """Candidate closing directions (yaw, rad) sorted by closeness to `preferred`."""
        if self.shape == 'cylinder':
            cands = [preferred + math.radians(d) for d in (0, 30, -30, 60, -60, 90, -90, 120, -120, 180)]
        else:
            cands = [object_yaw + k * math.pi / 2 for k in range(4)]

        def dist(a):
            return abs((a - preferred + math.pi) % (2 * math.pi) - math.pi)
        return sorted(cands, key=dist)


class Scene(object):
    def __init__(self, d):
        self.world_name = d.get('world_name', 'galbot_world')
        self.robot_name = d['robot']['name']
        self.robot_spawn = tuple(float(v) for v in d['robot']['spawn'])
        t = d['table']
        self.table_name = t.get('name', 'table')
        self.table_center = tuple(float(v) for v in t['center'])
        self.table_size = tuple(float(v) for v in t['size'])  # depth(x), width(y), top height
        self.table_top_thickness = float(t.get('top_thickness', 0.04))
        tr = d.get('tray')
        self.tray = None
        if tr:
            self.tray = {'name': tr.get('name', 'tray'),
                         'center': tuple(float(v) for v in tr['center']),
                         'size': tuple(float(v) for v in tr['size']),
                         'color': tuple(float(v) for v in tr.get('color', (0.2, 0.2, 0.2)))}
        self.objects = [SceneObject(o) for o in d.get('objects', [])]
        self.by_name = {o.name: o for o in self.objects}

    @classmethod
    def from_file(cls, path):
        with open(path) as f:
            return cls(yaml.safe_load(f))

    @property
    def table_height(self):
        return self.table_size[2]

    @property
    def table_front_x(self):
        return self.table_center[0] - self.table_size[0] / 2.0

    def tray_top(self):
        if not self.tray:
            return self.table_height
        return self.table_height + self.tray['size'][2]

    def object_spawn_z(self, obj):
        return self.table_height + obj.height / 2.0

    def to_dict(self):
        return {
            'table': {'center': self.table_center, 'size': self.table_size},
            'tray': self.tray,
            'objects': [{'name': o.name, 'label': o.label, 'shape': o.shape, 'size': o.size,
                         'color': o.color, 'position': o.position} for o in self.objects],
            'robot_spawn': self.robot_spawn,
        }
