find_package(Eigen3)

add_library(reactionwheel)
target_sources(reactionwheel
  PUBLIC
    FILE_SET CXX_MODULES FILES
      reactionwheel.cppm
)
target_link_libraries(reactionwheel
    PUBLIC
        Eigen3::Eigen
        actuator
        pdcontroller
        rigidbody
)
