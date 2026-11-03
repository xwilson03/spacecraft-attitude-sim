add_library(actuator)
target_sources(actuator
  PUBLIC
    FILE_SET CXX_MODULES FILES
      actuator.cppm
)
