cmake_minimum_required(VERSION 2.8.3)
project(heading_ros_service)

find_package(catkin_simple REQUIRED)

catkin_simple()

# Executables
cs_add_executable(example_ros_service src/example_ros_service.cpp)
cs_add_executable(example_ros_client src/example_ros_client.cpp)

cs_install()
cs_export()
    
