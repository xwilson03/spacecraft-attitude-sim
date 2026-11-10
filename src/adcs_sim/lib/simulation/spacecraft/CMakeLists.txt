find_package(Eigen3)

add_library(spacecraft)
target_sources(spacecraft
  PUBLIC
    FILE_SET CXX_MODULES FILES
      spacecraft.cppm
)
target_link_libraries(spacecraft
    PUBLIC
        Eigen3::Eigen
        reactionwheel
        magnetorquer
        rigidbody
)
