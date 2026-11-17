cmake_minimum_required(VERSION 3.8)
project(galbot_s1_control)

# The nodes are shared with mamoru1126/galbot_s1_gazebo (ros_compat.py also
# supports rospy there); in this repository they run on ROS 2 only.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/commander.py scripts/base_controller.py scripts/camera_relay.py
        DESTINATION lib/${PROJECT_NAME})
ament_package()
