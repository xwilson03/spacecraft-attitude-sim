find_package(rclcpp REQUIRED)
find_package(adcs_msgs REQUIRED)

add_library(simsnooper
    simsnooper.cpp
)
target_include_directories(simsnooper
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(simsnooper
    PUBLIC
        rclcpp::rclcpp
        adcs_msgs::adcs_msgs__rosidl_typesupport_cpp
        spacecraft
)
