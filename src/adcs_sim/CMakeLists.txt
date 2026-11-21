cmake_minimum_required(VERSION 3.29)
project(adcs_sim)

find_package(ament_cmake)

add_subdirectory(lib)
add_subdirectory(app)

ament_package()
