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
    unsigned int vertexShader;
    unsigned int fragmentShader;
    unsigned int shaderProgram;

    int compileSuccess;
    constexpr int COMPILE_LOG_SIZE = 512;
    char log[COMPILE_LOG_SIZE];

    std::array<float, 9> vertices = {
        -0.5, -0.5, 0.0,
         0.5, -0.5, 0.0,
         0.0,  0.5, 0.0,
    };

    const char* vertexShaderSrc = R"(
        #version 330 core
        layout (location = 0) in vec3 aPos;

        void main() {
            gl_Position = vec4(aPos.x, aPos.y, aPos.z, 1.0);
        }
    )";

    const char* fragmentShaderSrc = R"(
        #version 330 core
        out vec4 color;
        void main() {
            color = vec4(1.0, 1.0, 1.0, 1.0);
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
        vertexShader = f->glCreateShader(GL_VERTEX_SHADER);
        f->glShaderSource(vertexShader, 1, &vertexShaderSrc, NULL);
        f->glCompileShader(vertexShader);
        f->glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &compileSuccess);
        
        if (!compileSuccess) {
            f->glGetShaderInfoLog(vertexShader, COMPILE_LOG_SIZE, NULL, log);
            std::cout << "Vertex shader compilation failed. What: " << log << std::endl;
        }

        // Initialize and compile fragment shader
        fragmentShader = f->glCreateShader(GL_FRAGMENT_SHADER);
        f->glShaderSource(fragmentShader, 1, &fragmentShaderSrc, NULL);
        f->glCompileShader(fragmentShader);
        f->glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &compileSuccess);
        
        if (!compileSuccess) {
            f->glGetShaderInfoLog(fragmentShader, COMPILE_LOG_SIZE, NULL, log);
            std::cout << "Fragment shader compilation failed. What: " << log << std::endl;
        }

        // Initialize shader program
        shaderProgram = f->glCreateProgram();
        f->glAttachShader(shaderProgram, vertexShader);
        f->glAttachShader(shaderProgram, fragmentShader);
        f->glLinkProgram(shaderProgram);
        f->glGetProgramiv(shaderProgram, GL_LINK_STATUS, &compileSuccess);
        
        if (!compileSuccess) {
            f->glGetProgramInfoLog(shaderProgram, COMPILE_LOG_SIZE, NULL, log);
            std::cout << "Shader program linking failed. What: " << log << std::endl;
        }

        f->glUseProgram(shaderProgram);

        f->glDeleteShader(vertexShader);
        f->glDeleteShader(fragmentShader);
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
