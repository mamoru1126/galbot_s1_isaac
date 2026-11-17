cmake_minimum_required(VERSION 3.8)
project(galbot_s1_web)

find_package(ament_cmake REQUIRED)
install(DIRECTORY web launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/web_server.py DESTINATION lib/${PROJECT_NAME})
ament_package()
