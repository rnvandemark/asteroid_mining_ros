cmake_minimum_required(VERSION 3.5)

project(asteroid_mining_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(asteroid_mining_interfaces
    msg/Asteroid.msg
    msg/ReleasedPayload.msg
    msg/Siphon.msg
    msg/State.msg
    msg/ThirdOrderKinematicsOneDof.msg
    msg/ThirdOrderKinematicsThreeDof.msg
    DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_package()
