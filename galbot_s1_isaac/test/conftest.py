import os
import sys

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
REPO = os.path.dirname(PKG)
sys.path.insert(0, os.path.join(PKG, 'isaac'))
sys.path.insert(0, os.path.join(REPO, 'galbot_s1_control'))


@pytest.fixture
def isaac_urdf():
    with open(os.path.join(PKG, 'urdf', 'galbot_s1_isaac.urdf')) as f:
        return f.read()


@pytest.fixture
def desc_urdf():
    with open(os.path.join(PKG, 'urdf', 'galbot_s1.urdf')) as f:
        return f.read()


@pytest.fixture
def settings():
    from galbot_isaac import config
    return config.load_settings()


@pytest.fixture
def scene():
    from galbot_s1_control.scene import Scene
    with open(os.path.join(PKG, 'config', 'scene.yaml')) as f:
        raw = yaml.safe_load(f)
    s = Scene(raw)
    s.raw = raw
    return s
