import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))  # galbot_s1_control package (source tree)
SRC = os.environ.get('GALBOT_SRC', os.path.abspath(os.path.join(HERE, '..', '..')))


@pytest.fixture
def robot_urdf():
    """Expanded robot description (committed, see galbot_s1_isaac/tools/generate_urdf.py)."""
    with open(os.path.join(SRC, 'galbot_s1_isaac', 'urdf', 'galbot_s1.urdf')) as f:
        return f.read()


@pytest.fixture
def src():
    return SRC
