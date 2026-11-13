cmake_minimum_required(VERSION 3.21)
project(adcs_msgs)

find_package(ament_cmake)
find_package(rosidl_default_generators)

find_package(geometry_msgs)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Actuator.msg"
    "msg/Magnetorquer.msg"
    "msg/PDControllerFloat.msg"
    "msg/ReactionWheel.msg"
    "msg/Rigidbody.msg"
    "msg/Spacecraft.msg"
    "msg/Transform.msg"
    DEPENDENCIES geometry_msgs
)

ament_package()
