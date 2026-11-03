#include <chrono>
#include <cmath>
#include <cstdlib>
#include <format>
#include <functional>
#include <iostream>
#include <numbers>
#include <limits>
#include <source_location>

#define EIGEN_DEFAULT_IO_FORMAT Eigen::IOFormat(4, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]")

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <QApplication>
#include <QElapsedTimer>
#include <QOpenGLWidget>
#include <QOpenGLContext>
#include <QOpenGLExtraFunctions>
#include <QTimer>

import adcs.Actuator;
import adcs.Transform;
import adcs.Rigidbody;
import adcs.PDController;
import adcs.ReactionWheel;

using adcs::Actuator;
using adcs::Transform;
using adcs::Rigidbody;
using adcs::PDController;
using adcs::ReactionWheel;
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


class Magnetorquer {

    // Properties
    float mCommandPollInterval = 0.0f;
    float mMaxVoltage = 0.0f;
    float mResistance = 0.0f;
    float mInductance = 0.0f;
    float mDipolePerAmp = 0.0f;

    // State
    float mVoltage = 0.0f;
    float mCurrent = 0.0f;
    float mNextVoltage = 0.0f;
    float mTimeToNextPoll = 0.0f;

public:

    const auto commandPollInterval () const { return mCommandPollInterval; }
    const auto maxVoltage          () const { return mMaxVoltage;          }
    const auto resistance          () const { return mResistance;          }
    const auto inductance          () const { return mInductance;          }
    const auto dipolePerAmp        () const { return mDipolePerAmp;        }
    const auto voltage             () const { return mVoltage;             }
    const auto current             () const { return mCurrent;             }
    const auto nextVoltage         () const { return mNextVoltage;         }
    const auto timeToNextPoll      () const { return mTimeToNextPoll;      }

    void setCommandPollInterval (const auto commandPollInterval) { mCommandPollInterval = commandPollInterval; }
    void setMaxVoltage          (const auto maxVoltage)          { mMaxVoltage = maxVoltage;                   }
    void setResistance          (const auto resistance)          { mResistance = resistance;                   }
    void setInductance          (const auto inductance)          { mInductance = inductance;                   }
    void setDipolePerAmp        (const auto dipolePerAmp)        { mDipolePerAmp = dipolePerAmp;               }

    void setNextDipole (const auto nextDipole) {
        const float nextVoltage = nextDipole / mDipolePerAmp * mResistance;
        mNextVoltage = std::clamp(nextVoltage, -mMaxVoltage, mMaxVoltage);
    }

private:
    float substep (const float deltaTime) {

        const auto targetCurrent = mVoltage / mResistance;
        const auto timeConstant = mInductance / mResistance;

        const auto startCurrent = mCurrent;
        const auto currentError = targetCurrent - startCurrent;
        const auto endCurrent = startCurrent + currentError * -std::expm1(-deltaTime / timeConstant);
        const auto deltaCurrent = endCurrent - startCurrent;

        mCurrent = endCurrent;
        return targetCurrent * deltaTime - deltaCurrent * timeConstant;
    }

public:
    float step (const float deltaTime) {

        float charge = 0.0f;

        if (deltaTime == 0.0f) return charge;
        if (deltaTime > mCommandPollInterval) {
            std::cerr
                << std::format(
                    "{}: deltaTime of {} greater than Command Poll Interval of {}.",
                    std::source_location::current().function_name(),
                    deltaTime,
                    mCommandPollInterval
                )
            << std::endl;
            std::abort();
        }

        if (deltaTime <= mTimeToNextPoll) {
            charge = substep(deltaTime);
        }

        else {
            charge = substep(mTimeToNextPoll);
            mVoltage = mNextVoltage;
            charge += substep(deltaTime - mTimeToNextPoll);

            mTimeToNextPoll += mCommandPollInterval;
        }
        
        mTimeToNextPoll -= deltaTime;
        return charge * mDipolePerAmp;
    }

    friend std::ostream& operator<< (std::ostream& ostream, const Magnetorquer& instance) {
        return ostream
            << "Command Poll Interval: " << instance.mCommandPollInterval << "\n"
            << "Max Voltage: " << instance.mMaxVoltage << "\n"
            << "Resistance: " << instance.mResistance << "\n"
            << "Inductance: " << instance.mInductance << "\n"
            << "Dipole per Amp: " << instance.mDipolePerAmp << "\n"
            << "Voltage: " << instance.mVoltage << "\n"
            << "Current: " << instance.mCurrent << "\n"
            << "Next Voltage: " << instance.mNextVoltage << "\n"
            << "Time To Next Poll: " << instance.mTimeToNextPoll;
    }
};

void step (
    const float absoluteTime,
    const float deltaTime,
    Rigidbody& craft,
    ReactionWheel& rwheelX,
    ReactionWheel& rwheelY,
    ReactionWheel& rwheelZ,
    Magnetorquer& magtorqueX,
    Magnetorquer& magtorqueY,
    Magnetorquer& magtorqueZ
) {
    if (deltaTime == 0.0f) return;

    // Find craft intermediate axis
    const auto& J = craft.rotationalInertia();
    std::vector<std::pair<float, size_t>> JSorted = {{J.x(), 0u}, {J.y(), 1u}, {J.z(), 2u}};
    std::sort(JSorted.begin(), JSorted.end());
    const size_t intermediateAxis = JSorted[1].second;

    // Calculate craft ang momentum error (goal is to zero non-intermediate axes)
    Vector3f craftAngMomentumError = -craft.angularMomentum();
    craftAngMomentumError(intermediateAxis) = 0.0f;

    // Update reaction wheel setpoints; rwheel error = -craft error, 3rd law
    rwheelX.setNextAngularMomentumError(-craftAngMomentumError.x());
    rwheelY.setNextAngularMomentumError(-craftAngMomentumError.y());
    rwheelZ.setNextAngularMomentumError(-craftAngMomentumError.z());

    // Apply wheel setpoints and update craft ang momentum
    const Vector3f rwheelPreStepAngularMomentum (
        rwheelX.rigidbody().angularMomentum().x(),
        rwheelY.rigidbody().angularMomentum().y(),
        rwheelZ.rigidbody().angularMomentum().z()
    );

    rwheelX.step(deltaTime);
    rwheelY.step(deltaTime);
    rwheelZ.step(deltaTime);

    const Vector3f rwheelPostStepAngularMomentum (
        rwheelX.rigidbody().angularMomentum().x(),
        rwheelY.rigidbody().angularMomentum().y(),
        rwheelZ.rigidbody().angularMomentum().z()
    );

    const Vector3f rwheelDeltaAngularMomentum = rwheelPostStepAngularMomentum - rwheelPreStepAngularMomentum;

    // Apply magtorque setpoints and update craft ang momentum
    const float orbitalRate = 2.0f * M_PI / 5400.0f; // rad/s, estimated 90 minute orbit
    const Vector3f earthField = 30e-6f * Vector3f(std::cos(orbitalRate * absoluteTime), std::sin(orbitalRate * absoluteTime), 0.0f);
    const Vector3f earthFieldCraft = craft.rotation().conjugate().matrix() * earthField;

    const float desaturationTime = 1.0f; // realistically, the mtorqs are way too weak for this to actually limit their effort
    Vector3f magtorqueTorqueGoal (
        -rwheelX.rigidbody().angularMomentum().x() / desaturationTime,
        -rwheelY.rigidbody().angularMomentum().y() / desaturationTime,
        -rwheelZ.rigidbody().angularMomentum().z() / desaturationTime
    );
    const Vector3f magtorqueDipoleGoal = earthFieldCraft.cross(magtorqueTorqueGoal) / earthFieldCraft.squaredNorm();

    magtorqueX.setNextDipole(magtorqueDipoleGoal.x());
    magtorqueY.setNextDipole(magtorqueDipoleGoal.y());
    magtorqueZ.setNextDipole(magtorqueDipoleGoal.z());

    const Vector3f magtorqueImpulse = Vector3f(
        magtorqueX.step(deltaTime),
        magtorqueY.step(deltaTime),
        magtorqueZ.step(deltaTime)
    ).cross(earthFieldCraft);

    craft.setAngularMomentum(craft.angularMomentum() + -rwheelDeltaAngularMomentum + magtorqueImpulse);
    craft.step(deltaTime, rwheelPostStepAngularMomentum);
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
    const float& pi = std::numbers::pi_v<float>;

    ReactionWheel rwheelX;
    ReactionWheel rwheelY;
    ReactionWheel rwheelZ;

    const auto applyWheelConfig = [&](unsigned axis, ReactionWheel& rwheel) {
        rwheel.setWheelRotationalInertia(axis, 0.05f);
        rwheel.setActuatorMaxVoltage(24.0f);
        rwheel.setActuatorMaxCurrent(5.0f);
        rwheel.setActuatorResistance(0.5f);
        rwheel.setActuatorMotorConstant(0.1f);
        rwheel.setActuatorControllerKP(1.0f);
        rwheel.setActuatorControllerKD(0.0f);
        rwheel.setCommandPollInterval(0.2f);
        rwheel.setActuatorCommandPollInterval(0.1f);
    };

    applyWheelConfig(0, rwheelX);
    applyWheelConfig(1, rwheelY);
    applyWheelConfig(2, rwheelZ);

    Magnetorquer magtorqueX;
    Magnetorquer magtorqueY;
    Magnetorquer magtorqueZ;

    const auto applyMagtorqueConfig = [&](Magnetorquer& magtorque) {
        magtorque.setCommandPollInterval(0.1f);
        magtorque.setMaxVoltage(24.0f);
        magtorque.setResistance(100.0f);
        magtorque.setInductance(1.0f);
        magtorque.setDipolePerAmp(100.0f);
    };

    applyMagtorqueConfig(magtorqueX);
    applyMagtorqueConfig(magtorqueY);
    applyMagtorqueConfig(magtorqueZ);

    Rigidbody cube;
    cube.setScale( Vector3f(1.0f, 3.0f, 5.0f) );
    cube.setAngularVelocity( Vector3f(30.0f, 90.0f, 30.0f) * pi / 180.0f );

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

    float absoluteTime = 0.0f;

    QElapsedTimer elapsed;
    elapsed.start();

    QTimer physics;
    physics.setTimerType(Qt::PreciseTimer);
    QObject::connect(&physics, &QTimer::timeout, [&]() {
        const float deltaTime = 1e-3f;
        absoluteTime += deltaTime;
        step(
            absoluteTime, deltaTime, cube,
            rwheelX, rwheelY, rwheelZ,
            magtorqueX, magtorqueY, magtorqueZ
        );
    });
    physics.start(1);

    QTimer display;
    QObject::connect(&display, &QTimer::timeout, [&]() {
        window.update();
    });
    display.start(static_cast<int>(1.0f / 120.0f * 1000.0f));

    window.show();
    return app.exec();
}
