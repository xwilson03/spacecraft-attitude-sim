find_package(Eigen3)

add_library(rigidbody)
target_sources(rigidbody
  PUBLIC
    FILE_SET CXX_MODULES FILES
      rigidbody.cppm
)
target_link_libraries(rigidbody
    PUBLIC
        Eigen3::Eigen
        transform
)
