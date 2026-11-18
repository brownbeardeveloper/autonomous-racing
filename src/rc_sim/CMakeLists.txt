cmake_minimum_required(VERSION 3.8)
project(rc_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/scan_relay_node.py DESTINATION lib/${PROJECT_NAME} RENAME scan_relay_node)

ament_package()
