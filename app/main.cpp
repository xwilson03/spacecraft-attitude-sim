#include <QApplication>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>


int main(int argc, char* argv[]) {

    QApplication app(argc, argv);

    QOpenGLWidget window;
    window.resize(800, 600);
    window.setWindowTitle("Spacecraft Attitude Sim");

    // Initialize OpenGL functions
    QObject::connect(&window, &QOpenGLWidget::aboutToCompose, [&window]() {
        window.makeCurrent();
        QOpenGLFunctions* f = window.context()->functions();

        f->initializeOpenGLFunctions();
        f->glClearColor(0.0, 0.0, 0.0, 1.0);

        window.doneCurrent();
    });

    // Connect directly to the frame rendering phase
    QObject::connect(&window, &QOpenGLWidget::frameSwapped, [&window]() {
        window.makeCurrent();
        QOpenGLFunctions* f = window.context()->functions();

        f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        window.doneCurrent();
    });

    window.show();
    return app.exec();

}
