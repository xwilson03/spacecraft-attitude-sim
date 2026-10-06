find_package(Qt6 COMPONENTS Widgets OpenGLWidgets REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

set(AUTOMOC ON)
set(AUTOUIC ON)
set(AUTORCC ON)

add_executable(${PROJECT_NAME}
    main.cpp
)
target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Qt6::Widgets
        Qt6::OpenGLWidgets
        Eigen3::Eigen
)

install(
    TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
