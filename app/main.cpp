#include <iostream>

#include <QApplication>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>


int main(int argc, char* argv[]) {

    QApplication app(argc, argv);

    QOpenGLWidget window;
    window.resize(800, 600);
    window.setWindowTitle("Spacecraft Attitude Sim");


    unsigned int VBO;
    std::array<float, 9> vertices = {
        -0.5, -0.5, 0.0,
         0.5, -0.5, 0.0,
         0.0,  0.5, 0.0,
    };

    unsigned int vertexShader;
    const char* vertexShaderSrc = R"(
        #version 330 core
        layout (location = 0) in vec3 aPos;

        void main() {
            gl_Position = vec4(aPos.x, aPos.y, aPos.z, 1.0);
        }
    )";


    // Initialize OpenGL context
    QObject::connect(&window, &QOpenGLWidget::aboutToCompose, [&]() {
        window.makeCurrent();
        QOpenGLFunctions* f = window.context()->functions();

        // Initialize function ptrs, set clear color
        f->initializeOpenGLFunctions();
        f->glClearColor(0.0, 0.0, 0.0, 1.0);

        // Initialize vertex buffer object with data
        f->glGenBuffers(1, &VBO);
        f->glBindBuffer(GL_ARRAY_BUFFER, VBO);
        f->glBufferData(GL_ARRAY_BUFFER, vertices.size(), vertices.data(), GL_DYNAMIC_DRAW);

        // Initialize and compile vertex shader
        int success;
        vertexShader = f->glCreateShader(GL_VERTEX_SHADER);
        f->glShaderSource(vertexShader, 1, &vertexShaderSrc, NULL);
        f->glCompileShader(vertexShader);
        f->glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
        
        if (!success) {
            constexpr int SHADER_LOG_SIZE = 512;
            char log[SHADER_LOG_SIZE];
            f->glGetShaderInfoLog(vertexShader, SHADER_LOG_SIZE, NULL, log);
            std::cout << "Vertex shader compilation failed. What: " << log << std::endl;
        }

        window.doneCurrent();
    });

    // Connect directly to the frame rendering phase
    QObject::connect(&window, &QOpenGLWidget::frameSwapped, [&]() {
        window.makeCurrent();
        QOpenGLFunctions* f = window.context()->functions();

        f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        window.doneCurrent();
    });

    window.show();
    return app.exec();

}
