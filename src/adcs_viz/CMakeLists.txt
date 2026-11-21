cmake_minimum_required(VERSION 3.29)
project(adcs_viz)

find_package(ament_cmake)

add_subdirectory(lib)
add_subdirectory(app)

install(
    DIRECTORY assets
    DESTINATION ${CMAKE_INSTALL_DATADIR}
)

ament_package()
