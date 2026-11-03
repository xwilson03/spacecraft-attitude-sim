find_package(Eigen3)

add_library(magnetorquer)
target_sources(magnetorquer
  PUBLIC
    FILE_SET CXX_MODULES FILES
      magnetorquer.cppm
)
target_link_libraries(magnetorquer
    PUBLIC
        Eigen3::Eigen
)
