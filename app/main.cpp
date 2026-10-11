#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <numbers>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <QApplication>
#include <QOpenGLWidget>
#include <QOpenGLContext>
#include <QOpenGLExtraFunctions>
#include <QTimer>

using Eigen::Vector3f;
using Eigen::Quaternionf;
using Eigen::Translation3f;
using Eigen::Affine3f;
using Eigen::Matrix4f;
using Eigen::AngleAxisf;


class GLSimWidget : public QOpenGLWidget {
public:
    std::function<void()> onInit, onPaint;
protected:
    void initializeGL() override { if (onInit)  onInit();  }
    void paintGL()      override { if (onPaint) onPaint(); }
};


class Transform {

protected:

    Vector3f    mPosition = Vector3f::Zero();
    Quaternionf mRotation = Quaternionf::Identity();
    Vector3f    mScale    = Vector3f(1.0f, 1.0f, 1.0f);

public:

    const auto& position () const { return mPosition; }
    const auto& rotation () const { return mRotation; }
    const auto& scale    () const { return mScale;    }

    Affine3f affine() const {
        auto transform = Affine3f::Identity();
        transform.translate(mPosition);
        transform.rotate(mRotation);
        transform.scale(mScale);
        return transform;
    }

    void setPosition (const auto& aPosition) { mPosition = aPosition; }
    void setScale    (const auto& aScale)    { mScale = aScale; }

    void setRotation (const Quaternionf& aRotation) {
        mRotation = aRotation;
    }

    void setRotation (const Vector3f& aRotation) {
        mRotation = (
            AngleAxisf(aRotation.y(), Vector3f::UnitY())
          * AngleAxisf(aRotation.x(), Vector3f::UnitX())
          * AngleAxisf(aRotation.z(), Vector3f::UnitZ())
        );
    }
};


class Rigidbody : public Transform {

protected:

    Vector3f mAngularVelocity   = Vector3f::Zero();
    Vector3f mTorque            = Vector3f::Zero();
    Vector3f mRotationalInertia;

    Vector3f computeAngularAcceleration(const Vector3f& initialVelocity) const {
        const Vector3f L = initialVelocity.cwiseProduct(mRotationalInertia);
        return (mTorque - initialVelocity.cross(L)).cwiseQuotient(mRotationalInertia);
    }

public:

    Rigidbody()
    : Transform()
    , mRotationalInertia(
        mScale.y() * mScale.y() + mScale.z() * mScale.z(),
        mScale.x() * mScale.x() + mScale.z() * mScale.z(),
        mScale.x() * mScale.x() + mScale.y() * mScale.y()
    )
    {};

    const auto& angularVelocity   () const { return mAngularVelocity;   }
    const auto& torque            () const { return mTorque;            }
    const auto& rotationalInertia () const { return mRotationalInertia; }

    void setAngularVelocity   (const auto& aAngularVelocity)   { mAngularVelocity = aAngularVelocity;     }
    void setTorque            (const auto& aTorque)            { mTorque = aTorque;                       }
    void setRotationalInertia (const auto& aRotationalInertia) { mRotationalInertia = aRotationalInertia; }

    void setScale (const auto& aScale) {
        mScale = aScale;
        mRotationalInertia = Vector3f(
            mScale.y() * mScale.y() + mScale.z() * mScale.z(),
            mScale.x() * mScale.x() + mScale.z() * mScale.z(),
            mScale.x() * mScale.x() + mScale.y() * mScale.y()
        );
    }

    void step (const float deltaTime) {

        // Update angular velocity (RK4)
        const Vector3f k1 = computeAngularAcceleration(mAngularVelocity);
        const Vector3f k2 = computeAngularAcceleration(mAngularVelocity + 0.5f * deltaTime * k1);
        const Vector3f k3 = computeAngularAcceleration(mAngularVelocity + 0.5f * deltaTime * k2);
        const Vector3f k4 = computeAngularAcceleration(mAngularVelocity + deltaTime * k3);

        mAngularVelocity += deltaTime / 6.0f * (k1 + 2.0f * k2 + 2.0f * k3 + k4);
        mTorque.setZero();

        // Update cube orientation
        const float angle = mAngularVelocity.norm() * deltaTime;
        if (angle <= 0.0f) return;

        mRotation *= Quaternionf(AngleAxisf(angle, mAngularVelocity.normalized()));
        mRotation.normalize();
    }
};

template <typename T>
concept HasZeroFunc = requires {
    { T::Zero() } -> std::same_as<T>;
};

template <typename T>
concept Controllable = requires(T t, float f) {
    { t + t } -> std::same_as<T>;
    { t - t } -> std::same_as<T>;
    { f * t } -> std::same_as<T>;
    { t / f } -> std::same_as<T>;
};

template <typename T>
constexpr T Initializer() noexcept {
    if constexpr (HasZeroFunc<T>) return T::Zero();
    else return T {};
};

template <typename T>
class PDController {

protected:

    float mKP = 1.0f;
    float mKD = 0.0f;

    T mLastError = Initializer<T>();

public:

    const auto  kP        () const { return mKP; }
    const auto  kD        () const { return mKD; }
    const auto& lastError () const { return mLastError; }

    void setKP (const auto aKP) { mKP = aKP; }
    void setKD (const auto aKD) { mKD = aKD; }

    T computeEffort(const T& current, const T& goal) {

        const auto& error = goal - current;
        const auto& deltaError = error - mLastError;
        mLastError = error;
        
        return mKP * error + mKD * deltaError;
    }

};


void step(
    float deltaTime,
    Rigidbody& cube,
    PDController<Vector3f> controller
) {
    // Calculate goal rotation (zero non-intermediate axes)

    const auto& J = cube.rotationalInertia();
    std::vector<std::pair<float, size_t>> JSorted = {{J.x(), 0u}, {J.y(), 1u}, {J.z(), 2u}};
    std::sort(JSorted.begin(), JSorted.end());
    const size_t intermediateAxis = JSorted[2].second;
    
    Vector3f goal = Vector3f::Zero();
    goal(intermediateAxis) = cube.angularVelocity()[intermediateAxis];

    // Compute effort values
    const Vector3f torque = controller.computeEffort(cube.angularVelocity(), goal); 

    cube.setTorque(torque);
    cube.step(deltaTime);
}


int main(int argc, char* argv[]) {

    // App Setup

    QApplication app(argc, argv);

    GLSimWidget window;
    window.resize(800, 600);
    window.setWindowTitle("Spacecraft Attitude Sim");


    // Shader Data

    unsigned int vertexShader, fragmentShader, shaderProgram;
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


    // Vertex Data

    unsigned int VAO, VBO, EBO;

    struct Vertex {
        float x, y, z;
    };

    std::array<Vertex, 8> vertices;
    int i = 0;
    for (const auto x: {-0.5f, 0.5f}) {
        for (const auto y: {0.5f, -0.5f}) {
            for (const auto z: {0.5f, -0.5f}) {
                vertices[i++] = {x, y, z};
            }
        }
    }

    // 0 = top left front
    // +1 = front -> back
    // +2 = top -> bottom
    // +4 = left -> right
    const std::array indices = {
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


    // State

    auto lastTime = std::chrono::steady_clock::now();
    PDController<Vector3f> controller;

    const float& pi = std::numbers::pi_v<float>;

    Rigidbody cube;
    cube.setScale( Vector3f(1.0f, 3.0f, 5.0f) );
    cube.setAngularVelocity( Vector3f(30.0f, 60.0f, 90.0f) * pi / 180.0f );

    Transform camera;
    camera.setPosition( Vector3f(5.0f, 5.0f, 5.0f) );
    camera.setRotation( Vector3f(-36.0f, 45.0f, 0.0f) * pi / 180.0f );


    // Rendering

    // View Matrix
    const Matrix4f view = camera.affine().inverse().matrix();
    
    // Projection Matrix
    const float cameraVertFovDeg = 70.0f;
    const float cameraNearPlane = 0.1f;
    const float cameraFarPlane = 100.0f;

    const float focalDistance = 1.0f / std::tan((cameraVertFovDeg * pi / 180.0f) / 2.0f); // use vertical FOV to compute distance from camera where screen height = 2 world units (+-1)
    float aspectRatio = static_cast<float>(window.width()) / static_cast<float>(window.height());

    Matrix4f projection = Matrix4f::Zero();
    projection(0, 0) = focalDistance / aspectRatio;                                                          // scale X to +-1 in screen-space
    projection(1, 1) = focalDistance;                                                                        // scale Y to +-1 in screen-space
    projection(2, 2) = (cameraNearPlane + cameraFarPlane) / (cameraFarPlane - cameraNearPlane) * -1;         // depth = (Az + B) / (-z); scale
    projection(2, 3) = cameraFarPlane * 2.0f * (cameraNearPlane / (cameraFarPlane - cameraNearPlane) * -1);  //    near plane to -1 and far to +1
    projection(3, 2) = -1.0f;                                                                                // divide all by -z for perspective scaling

    // Combined Matrix (No "Model" component as cube is at (0, 0, 0))
    Matrix4f mvp = projection * view;


    // Initialize OpenGL context

    window.onInit = [&]() {
        window.makeCurrent();
        auto f = window.context()->extraFunctions();

        // Initialize function ptrs, set clear color, depth bit
        f->initializeOpenGLFunctions();
        f->glClearColor(0.0, 0.0, 0.0, 1.0);
        f->glEnable(GL_DEPTH_TEST);

        // Initialize vertex data containers
        f->glGenVertexArrays(1, &VAO);
        f->glBindVertexArray(VAO);
        
        f->glGenBuffers(1, &VBO);
        f->glBindBuffer(GL_ARRAY_BUFFER, VBO);
        f->glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices.data(), GL_STATIC_DRAW);

        f->glGenBuffers(1, &EBO);
        f->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        f->glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices.data(), GL_STATIC_DRAW);

        f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*) 0);
        f->glEnableVertexAttribArray(0);

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
        window.doneCurrent();
    };

    // Connect directly to the frame rendering phase
    window.onPaint = [&]() {
        window.makeCurrent();
        const auto f = window.context()->extraFunctions();

        f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Recalculate projection matrix in case aspect ratio changed
        aspectRatio = static_cast<float>(window.width()) / static_cast<float>(window.height());
        projection(0, 0) = focalDistance / aspectRatio;

        const Matrix4f& model = cube.affine().matrix();
        mvp = projection * view * model;

        // Set shader program and update MVP matrix
        f->glUseProgram(shaderProgram);
        f->glUniformMatrix4fv(f->glGetUniformLocation(shaderProgram, "mvp"), 1, GL_FALSE, mvp.data());

        // Copy vertex data
        f->glBindVertexArray(VAO);
        f->glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);

        window.doneCurrent();
    };

    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [&]() {
        // Calculate deltaTime
        const auto currentTime = std::chrono::steady_clock::now();
        const auto deltaTime = static_cast<std::chrono::duration<float>>(currentTime - lastTime).count();
        lastTime = currentTime;

        // Step sim and re-render
        step(deltaTime, cube, controller);
        window.update();
    });
    timer.start((1.0f / 60.0f) * 1000.0f);

    window.show();
    return app.exec();

}
