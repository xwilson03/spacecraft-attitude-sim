find_package(Eigen3 REQUIRED)
find_package(adcs_msgs REQUIRED)

add_library(msg_helpers)
target_sources(msg_helpers
  PUBLIC
    FILE_SET CXX_MODULES FILES
      msg_helpers.cppm
)
target_link_libraries(msg_helpers
    PUBLIC
        Eigen3::Eigen
        adcs_msgs::adcs_msgs__rosidl_typesupport_cpp
        actuator
        magnetorquer
        pdcontroller
        reactionwheel
        rigidbody
        spacecraft
        transform
)
