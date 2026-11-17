cmake_minimum_required(VERSION 3.8)
project(galbot_s1_isaac)

find_package(ament_cmake REQUIRED)

# The Isaac Sim app (isaac/run_sim.py) runs with the Isaac Sim python, it is installed
# as plain files next to its configuration.
install(DIRECTORY config launch urdf isaac tools DESTINATION share/${PROJECT_NAME}
        PATTERN "__pycache__" EXCLUDE)
install(PROGRAMS isaac/run_sim.py tools/generate_urdf.py DESTINATION lib/${PROJECT_NAME})

ament_package()
