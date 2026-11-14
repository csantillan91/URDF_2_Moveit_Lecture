cmake_minimum_required(VERSION 3.22)
project(buzz_arm_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf worlds models config DESTINATION share/${PROJECT_NAME})

ament_package()
