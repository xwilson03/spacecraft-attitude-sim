find_package(Qt6 COMPONENTS OpenGLWidgets REQUIRED)
find_package(Eigen3)

add_library(glsimwidget)
qt_wrap_cpp(GLSIMWIDGET_MOC
    glsimwidget.hpp
    TARGET glsimwidget
)
target_sources(glsimwidget
    PUBLIC FILE_SET CXX_MODULES FILES
        glsimwidget.cppm
    PRIVATE
        glsimwidget.cpp
        glsimwidget.hpp
        ${GLSIMWIDGET_MOC}
)
target_link_libraries(glsimwidget
    PUBLIC
        Eigen3::Eigen
        Qt6::OpenGLWidgets
        transform
)
