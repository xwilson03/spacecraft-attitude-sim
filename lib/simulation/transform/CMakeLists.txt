find_package(Eigen3)

add_library(transform)
target_sources(transform
  PUBLIC
    FILE_SET CXX_MODULES FILES
      transform.cppm
)
target_link_libraries(transform
    PUBLIC
        Eigen3::Eigen
)
