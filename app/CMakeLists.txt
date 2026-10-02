find_package(Qt6 COMPONENTS Widgets OpenGLWidgets REQUIRED)

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
)

install(
    TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
