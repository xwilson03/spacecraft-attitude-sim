add_library(pdcontroller)
target_sources(pdcontroller
  PUBLIC
    FILE_SET CXX_MODULES FILES
      pdcontroller.cppm
)
