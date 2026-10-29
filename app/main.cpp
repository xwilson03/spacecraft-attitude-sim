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

    friend std::ostream& operator<< (std::ostream& ostream, const Transform& instance) {
        return ostream
            << "Position: " << instance.position() << "\n"
            << "Rotation: " << instance.rotation().coeffs() << "\n"
            << "Scale: " << instance.scale() << "\n";
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
    const auto  angularMomentum   () const {
        return Vector3f(mAngularVelocity.cwiseProduct(mRotationalInertia));
    }

    void setAngularVelocity   (const auto& aAngularVelocity)   { mAngularVelocity = aAngularVelocity;     }
    void setTorque            (const auto& aTorque)            { mTorque = aTorque;                       }
    void setRotationalInertia (const auto& aRotationalInertia) { mRotationalInertia = aRotationalInertia; }
    void setAngularMomentum   (const auto& aAngularMomentum) {
        mAngularVelocity = aAngularMomentum.cwiseQuotient(mRotationalInertia);
    }

    void setScale (const auto& aScale) {
        mScale = aScale;
        mRotationalInertia = Vector3f(
            mScale.y() * mScale.y() + mScale.z() * mScale.z(),
            mScale.x() * mScale.x() + mScale.z() * mScale.z(),
            mScale.x() * mScale.x() + mScale.y() * mScale.y()
        );
    }

    void step (const float deltaTime) {

        if (deltaTime == 0.0f) return;

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

    friend std::ostream& operator<< (std::ostream& ostream, const Rigidbody& instance) {
        return ostream
            << static_cast<const Transform&>(instance)
            << "Angular Velocity: " << instance.angularVelocity() << "\n"
            << "Torque: " << instance.torque() << "\n"
            << "Rotational Inertia: " << instance.rotationalInertia() << "\n";
    }
};

template <typename T>
concept HasZeroFunc = requires {
    { T::Zero() } -> std::convertible_to<T>;
};

template <typename T>
concept ZeroInitializable = std::is_arithmetic_v<T> || HasZeroFunc<T>;

template <typename T>
concept Controllable = ZeroInitializable<T> && requires(T t, float f) {
    { t + t } -> std::convertible_to<T>;
    { t - t } -> std::convertible_to<T>;
    { f * t } -> std::convertible_to<T>;
    { t / f } -> std::convertible_to<T>;
};

template <ZeroInitializable T>
constexpr T Initializer() noexcept {
    if constexpr (HasZeroFunc<T>) return T::Zero();
    else return T {};
};

template <Controllable T>
class PDController {

protected:

    float mKP = 0.0f;
    float mKD = 0.0f;

    T mLastError = Initializer<T>();

public:

    const auto  kP        () const { return mKP; }
    const auto  kD        () const { return mKD; }
    const auto& lastError () const { return mLastError; }

    void setKP (const auto aKP) { mKP = aKP; }
    void setKD (const auto aKD) { mKD = aKD; }

    T computeEffort(
        const T& current,
        const T& goal,
        const float deltaTime
    ) {
        if (deltaTime == 0.0f) return current * 0.0f;

        const auto& error = goal - current;
        const auto& deltaError = error - mLastError;
        mLastError = error;

        return mKP * error + mKD * (deltaError / deltaTime);
    }

    T computeEffort(
        const T& error,
        const float deltaTime
    ) {
        if (deltaTime == 0.0f) return error * 0.0f;

        const auto& deltaError = error - mLastError;
        mLastError = error;

        return mKP * error + mKD * (deltaError / deltaTime);
    }

    friend std::ostream& operator<< (std::ostream& ostream, const PDController& instance) {
        return ostream
            << "kP: " << instance.kP() << "\n"
            << "kD: " << instance.kD() << "\n"
            << "lastError: " << instance.lastError() << "\n";
    }
};


class Actuator {

    // Properties
    float mCommandPollInterval = 0.0f;
    float mMaxVoltage = 0.0f;
    float mMaxCurrent = 0.0f;
    float mResistance = 0.0f;
    float mMotorConstant = 0.0f; // Nm/Amp

    // State
    float mVoltage = 0.0f;
    float mNextVoltage= 0.0f;
    float mTimeToNextPoll = 0.0f;

public:

    const auto commandPollInterval () const { return mCommandPollInterval; }
    const auto maxVoltage          () const { return mMaxVoltage;          }
    const auto maxCurrent          () const { return mMaxCurrent;          }
    const auto resistance          () const { return mResistance;          }
    const auto motorConstant       () const { return mMotorConstant;       }
    const auto voltage             () const { return mVoltage;             }
    const auto nextVoltage         () const { return mNextVoltage;         }
    const auto timeToNextPoll      () const { return mTimeToNextPoll;      }

    void setCommandPollInterval (const auto commandPollInterval) { mCommandPollInterval = commandPollInterval; }
    void setMaxVoltage          (const auto maxVoltage)          { mMaxVoltage = maxVoltage;                   }
    void setMaxCurrent          (const auto maxCurrent)          { mMaxCurrent = maxCurrent;                   }
    void setResistance          (const auto resistance)          { mResistance = resistance;                   }
    void setMotorConstant       (const auto motorConstant)       { mMotorConstant = motorConstant;             }

    void setNextVoltage (const auto nextVoltage) {
        mNextVoltage = std::clamp(nextVoltage, -mMaxVoltage, mMaxVoltage);
    }

private:
    float computeImpulse (
            const float startSpeed,
            const float inertia,
            const float deltaTime,
            const float timeConstant
    ) const {

        const auto targetSpeed = mVoltage / mMotorConstant;
        const auto maxTorque = mMotorConstant * mMaxCurrent;
        const auto maxAcceleration = maxTorque / inertia;

        const auto error = targetSpeed - startSpeed;
        const auto acceleration = std::clamp(
            error * -std::expm1(-deltaTime / timeConstant),
            -maxAcceleration * deltaTime,
            maxAcceleration * deltaTime
        );

        return acceleration * inertia;
    }

public:

    float step (
        const float deltaTime,
        const float startSpeed,
        const float inertia
    ) {

        if (deltaTime == 0.0f) return 0.0f;

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

        const float timeConstant = (mResistance * inertia) / (mMotorConstant * mMotorConstant);
        float impulse = 0.0f;

        if (deltaTime <= mTimeToNextPoll) {
            impulse = computeImpulse(startSpeed, inertia, deltaTime, timeConstant);
        }

        else {
            impulse = computeImpulse(startSpeed, inertia, mTimeToNextPoll, timeConstant);

            mVoltage = mNextVoltage;
            impulse += computeImpulse(
                startSpeed + impulse / inertia,
                inertia,
                deltaTime - mTimeToNextPoll,
                timeConstant
            );

            mTimeToNextPoll += mCommandPollInterval;
        }
        
        mTimeToNextPoll -= deltaTime;
        return impulse;
    }

    friend std::ostream& operator<< (std::ostream& ostream, const Actuator& instance) {
        return ostream
            << "Command Poll Interval: " << instance.mCommandPollInterval << "\n"
            << "Max Voltage: " << instance.mMaxVoltage << "\n"
            << "Max Current: " << instance.mMaxCurrent << "\n"
            << "Resistance: " << instance.mResistance << "\n"
            << "Motor Constant: " << instance.mMotorConstant << "\n"
            << "Voltage: " << instance.mVoltage << "\n"
            << "Next Voltage: " << instance.mNextVoltage << "\n"
            << "Time To Next Poll: " << instance.mTimeToNextPoll;
    }
};


class ReactionWheel {

    // Parameters
    float mCommandPollInterval = 0.0f;
    float mWheelRotationalInertia = 0.0f;
    unsigned mWheelAxis = 0u;

    // State
    float mTimeToNextPoll = 0.0f;
    float mAngularMomentumGoal = 0.0f;
    float mNextAngularMomentumError = 0.0f;
    Rigidbody mRigidbody;
    Actuator mActuator;
    PDController<float> mActuatorController;

public:
    const auto  commandPollInterval         () const { return mCommandPollInterval;         }
    const auto  wheelRotationalInertia      () const { return mWheelRotationalInertia;      }
    const auto  timeToNextPoll              () const { return mTimeToNextPoll;              }
    const auto& angularMomentumGoal         () const { return mAngularMomentumGoal;         }
    const auto& nextAngularMomentumError    () const { return mNextAngularMomentumError;    }
    const auto& rigidbody                   () const { return mRigidbody;                   }
    const auto& actuator                    () const { return mActuator;                    }
    const auto& actuatorController          () const { return mActuatorController;          }

    void setCommandPollInterval (const auto commandPollInterval) {
        mCommandPollInterval = commandPollInterval;
    }

    void setWheelRotationalInertia (const auto axis, const auto wheelRotationalInertia) {
        mWheelAxis = axis;
        mWheelRotationalInertia = wheelRotationalInertia;

        // Using zero breaks division by J, e.g. Rigidbody::computeAngularAcceleration
        const float epsilon = std::numeric_limits<float>::epsilon();
        Vector3f J (epsilon, epsilon, epsilon);
        J(mWheelAxis) = wheelRotationalInertia;
        mRigidbody.setRotationalInertia(J);
    }

    void setActuatorCommandPollInterval (const auto actuatorCommandPollInterval) {
        mActuator.setCommandPollInterval(actuatorCommandPollInterval);
    }

    void setActuatorMaxVoltage (const auto actuatorMaxVoltage) {
        mActuator.setMaxVoltage(actuatorMaxVoltage);
    }

    void setActuatorMaxCurrent (const auto actuatorMaxCurrent) {
        mActuator.setMaxCurrent(actuatorMaxCurrent);
    }

    void setActuatorResistance (const auto actuatorResistance) {
        mActuator.setResistance(actuatorResistance);
    }

    void setActuatorMotorConstant (const auto actuatorMotorConstant) {
        mActuator.setMotorConstant(actuatorMotorConstant);
    }

    void setNextAngularMomentumError (const auto angularMomentumError) {
        mNextAngularMomentumError = angularMomentumError;
    }

    void setActuatorControllerKP (const auto kP) {
        mActuatorController.setKP(kP);
    }

    void setActuatorControllerKD (const auto kD) {
        mActuatorController.setKD(kD);
    }

private:
    void substep(const float deltaTime) {

            if (deltaTime == 0.0f) return;

            const auto angVelError = (mAngularMomentumGoal / mWheelRotationalInertia) - mRigidbody.angularVelocity()(mWheelAxis);
            const auto actuatorEffort = mActuatorController.computeEffort(angVelError, deltaTime);

            mActuator.setNextVoltage(actuatorEffort);
            const auto actuatorImpulse = mActuator.step(
                deltaTime,
                mRigidbody.angularVelocity()(mWheelAxis),
                mWheelRotationalInertia
            );

            Vector3f torque = Vector3f::Zero();
            torque(mWheelAxis) = actuatorImpulse / deltaTime;
            mRigidbody.setTorque(torque);
            mRigidbody.step(deltaTime);
    }

public:
    void step (const float deltaTime) {

        if (deltaTime == 0.0f) return;
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
            substep(deltaTime);
        }

        else {
            const auto nextAngularMomentumGoal = mRigidbody.angularMomentum()(mWheelAxis) + mNextAngularMomentumError;

            substep(mTimeToNextPoll);
            mAngularMomentumGoal = nextAngularMomentumGoal;
            substep(deltaTime - mTimeToNextPoll);

            mTimeToNextPoll += mCommandPollInterval;
        }
        
        mTimeToNextPoll -= deltaTime;
    }

    friend std::ostream& operator<< (std::ostream& ostream, const ReactionWheel& instance) {
        return ostream
            << "Command Poll Interval: " << instance.mCommandPollInterval << "\n"
            << "Wheel Rotational Inertia: " << instance.mWheelRotationalInertia << "\n"
            << "Time to Next Poll: " << instance.mTimeToNextPoll << "\n"
            << "Angular Momentum Goal: " << instance.mAngularMomentumGoal << "\n"
            << "Next Angular Momentum Error: " << instance.mNextAngularMomentumError << "\n"
            << "Rigidbody: " << "\n"
            << instance.mRigidbody
            << "Actuator: " << "\n"
            << instance.mActuator;
    }
};


class Magnetorquers {

    // Parameters
    float mCommandPollInterval = 0.0f;
    float mMaxDipoleMagnitude = 0.0f;

    // State
    float mTimeToNextPoll = 0.0f;
    std::array<float, 3> mCurrentCommands {};
    std::array<float, 3> mNextCommands {};
    std::array<float, 3> mDipoleMagnitudes {};

public:
    const auto  commandPollInterval () const { return mCommandPollInterval; }
    const auto  maxDipoleMagnitude  () const { return mMaxDipoleMagnitude;  }
    const auto  timeToNextPoll      () const { return mTimeToNextPoll;      }
    const auto& currentCommands     () const { return mCurrentCommands;     }
    const auto& nextCommands        () const { return mNextCommands;        }
    const auto& dipoleMagnitudes    () const { return mDipoleMagnitudes;    }

    void setCommandPollInterval (const auto aCommandPollInterval) { mCommandPollInterval = aCommandPollInterval; }
    void setMaxDipoleMagnitude  (const auto aMaxDipoleMagnitude)  { mMaxDipoleMagnitude = aMaxDipoleMagnitude;   }

    void setNextCommands (const auto aAxis, const auto aMagnitude) {
        mNextCommands[aAxis] = std::clamp(aMagnitude, -mMaxDipoleMagnitude, mMaxDipoleMagnitude);
    }

    void step (const float deltaTime) {

        if (deltaTime == 0.0f) return;

        if (mTimeToNextPoll <= deltaTime) {
            mCurrentCommands = mNextCommands;
            mDipoleMagnitudes = mCurrentCommands;
            mTimeToNextPoll += mCommandPollInterval;
        }

        mTimeToNextPoll -= deltaTime;
    }

    friend std::ostream& operator<< (std::ostream& ostream, const Magnetorquers& instance) {
        return ostream
            << "Command Poll Interval: " << instance.commandPollInterval() << "\n"
            << "Max Dipole Magnitude: " << instance.maxDipoleMagnitude() << "\n"
            << "Dipole Magnitudes: ["
                << instance.mDipoleMagnitudes[0] << ", "
                << instance.mDipoleMagnitudes[1] << ", "
                << instance.mDipoleMagnitudes[2] << "]";
    }
};

void step (
    const float deltaTime,
    Rigidbody& craft,
    ReactionWheel& rwheelX,
    ReactionWheel& rwheelY,
    ReactionWheel& rwheelZ,
    Magnetorquers& magtorques
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
    craft.setAngularMomentum(craft.angularMomentum() + -rwheelDeltaAngularMomentum);

    craft.step(deltaTime);
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
        rwheel.setWheelRotationalInertia(axis, 0.1f);
        rwheel.setActuatorMaxVoltage(24.0f);
        rwheel.setActuatorMaxCurrent(5.0f);
        rwheel.setActuatorResistance(1.0f);
        rwheel.setActuatorMotorConstant(0.5f);
        rwheel.setActuatorControllerKP(1.0f);
        rwheel.setActuatorControllerKD(0.0f);
        rwheel.setCommandPollInterval(0.2f);
        rwheel.setActuatorCommandPollInterval(0.1f);
    };

    applyWheelConfig(0, rwheelX);
    applyWheelConfig(1, rwheelY);
    applyWheelConfig(2, rwheelZ);

    Magnetorquers magtorques;

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

    QElapsedTimer elapsed;
    elapsed.start();

    QTimer physics;
    physics.setTimerType(Qt::PreciseTimer);
    QObject::connect(&physics, &QTimer::timeout, [&]() {
        const float deltaTime = 1e-3f;
        step(deltaTime, cube, rwheelX, rwheelY, rwheelZ, magtorques);
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
