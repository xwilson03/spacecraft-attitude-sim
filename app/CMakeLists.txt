find_package(Qt6 COMPONENTS Widgets OpenGLWidgets REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

add_executable(${PROJECT_NAME}
    main.cpp
)
target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Qt6::OpenGLWidgets
        Qt6::Widgets
        Eigen3::Eigen
        glsimwidget
        spacecraft
        transform
)

install(
    TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
