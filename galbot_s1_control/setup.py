# Used by catkin (ROS 1) only: catkin_python_setup() installs the galbot_s1_control module.
# ROS 2 installs it with ament_python_install_package() (see CMakeLists.txt).
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(packages=['galbot_s1_control'], package_dir={'': '.'}))
