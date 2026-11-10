#include <Eigen/Dense>
#include <QOpenGLContext>
#include <QOpenGLExtraFunctions>
#include <QOpenGLWidget>
#include <iostream>

#include "glsimwidget.hpp"

import adcs.Transform;

using Eigen::Matrix4f;
using Eigen::Vector3f;


namespace adcs {

void GLSimWidget::initializeGL() {

    makeCurrent();
    auto f = context()->extraFunctions();

    // Initialize function ptrs, set clear color, depth bit
    f->initializeOpenGLFunctions();
    f->glClearColor(0.0, 0.0, 0.0, 1.0);
    f->glEnable(GL_DEPTH_TEST);


    // Initialize vertex data containers
    f->glGenVertexArrays(1, &mVAO);
    f->glBindVertexArray(mVAO);
    
    f->glGenBuffers(1, &mVBO);
    f->glBindBuffer(GL_ARRAY_BUFFER, mVBO);
    f->glBufferData(GL_ARRAY_BUFFER, sizeof(mVertices), mVertices.data(), GL_STATIC_DRAW);

    f->glGenBuffers(1, &mEBO);
    f->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mEBO);
    f->glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(mIndices), mIndices.data(), GL_STATIC_DRAW);

    f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*) 0);
    f->glEnableVertexAttribArray(0);


    // Shader Data

    unsigned int vertexShader, fragmentShader;
    int compileSuccess;
    constexpr int COMPILE_LOG_SIZE = 512;
    char log[COMPILE_LOG_SIZE];

    const char* vertexShaderSrc = R"(
        #version 330 core
        layout (location = 0) in vec3 aPos;
        uniform mat4 mvp;

        void main() {
            gl_Position = mvp * vec4(aPos, 1.0);
        }
    )";

    const char* fragmentShaderSrc = R"(
        #version 330 core
        out vec4 color;
        void main() {
            color = vec4(1.0, 1.0, 1.0, 1.0);
        }
    )";

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

    // Clean up
    f->glDeleteShader(vertexShader);
    f->glDeleteShader(fragmentShader);
    doneCurrent();
}

void GLSimWidget::paintGL() {

    makeCurrent();
    const auto f = context()->extraFunctions();

    f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Recalculate mvp in case aspect ratio or model transform changed
    const auto aspectRatio = static_cast<float>(width()) / static_cast<float>(height());
    mProjectionMatrix(0, 0) = focalDistance / aspectRatio;

    mModelMatrix = mScene.model.affine().matrix();
    mViewMatrix = mScene.camera.affine().inverse().matrix();
    mvp = mProjectionMatrix * mViewMatrix * mModelMatrix;

    // Set shader program and update MVP matrix
    f->glUseProgram(shaderProgram);
    f->glUniformMatrix4fv(f->glGetUniformLocation(shaderProgram, "mvp"), 1, GL_FALSE, mvp.data());

    // Copy vertex data
    f->glBindVertexArray(mVAO);
    f->glDrawElements(GL_TRIANGLES, mIndices.size(), GL_UNSIGNED_INT, 0);

    doneCurrent();
}

GLSimWidget::GLSimWidget(const Scene& aScene) 
: QOpenGLWidget()
, mScene(aScene)
{
    resize(800, 600);
    setWindowTitle("Spacecraft Attitude Sim");

    // Projection Matrix
    float aspectRatio = static_cast<float>(width()) / static_cast<float>(height());

    mProjectionMatrix(0, 0) = focalDistance / aspectRatio;                                                          // scale X to +-1 in screen-space
    mProjectionMatrix(1, 1) = focalDistance;                                                                        // scale Y to +-1 in screen-space
    mProjectionMatrix(2, 2) = (cameraNearPlane + cameraFarPlane) / (cameraFarPlane - cameraNearPlane) * -1;         // depth = (Az + B) / (-z); scale
    mProjectionMatrix(2, 3) = cameraFarPlane * 2.0f * (cameraNearPlane / (cameraFarPlane - cameraNearPlane) * -1);  //    near plane to -1 and far to +1
    mProjectionMatrix(3, 2) = -1.0f;                                                                                // divide all by -z for perspective scaling

    // Vertex Data
    int i = 0;
    for (const auto x: {-0.5f, 0.5f}) {
        for (const auto y: {0.5f, -0.5f}) {
            for (const auto z: {0.5f, -0.5f}) {
                mVertices[i++] = {x, y, z};
            }
        }
    }

    // 0 = top left front
    // +1 = front -> back
    // +2 = top -> bottom
    // +4 = left -> right
    mIndices = {
        // front
        0u, 2u, 6u,
        0u, 4u, 6u,

        // left
        1u, 3u, 2u,
        1u, 0u, 2u,

        // right
        4u, 6u, 7u,
        4u, 5u, 7u,

        // back
        5u, 7u, 3u,
        5u, 1u, 3u,

        // top
        1u, 0u, 4u,
        1u, 5u, 4u,

        // bottom
        2u, 3u, 7u,
        2u, 6u, 7u,
    };
}

} // namespace adcs
