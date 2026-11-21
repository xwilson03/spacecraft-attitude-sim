find_package(Eigen3 3.4 REQUIRED NO_MODULE)

add_executable(${PROJECT_NAME}
    main.cpp
)
target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Eigen3::Eigen
        simsnooper
        spacecraft
        transform
)

install(
    TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
