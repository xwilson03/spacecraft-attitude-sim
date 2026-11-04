#define EIGEN_DEFAULT_IO_FORMAT Eigen::IOFormat(4, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]")

#include <Eigen/Dense>
#include <QApplication>
#include <QElapsedTimer>
#include <QTimer>

import adcs.GLSimWidget;
import adcs.Magnetorquer;
import adcs.ReactionWheel;
import adcs.Rigidbody;
import adcs.Transform;

using Eigen::Matrix4f;
using Eigen::Vector3f;
using adcs::GLSimWidget;
using adcs::Magnetorquer;
using adcs::ReactionWheel;
using adcs::Rigidbody;
using adcs::Scene;
using adcs::Transform;


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

    // State
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

    const float& pi = std::numbers::pi_v<float>;

    Rigidbody cube;
    cube.setScale( Vector3f(1.0f, 3.0f, 5.0f) );
    cube.setAngularVelocity( Vector3f(30.0f, 90.0f, 30.0f) * pi / 180.0f );

    // Rendering
    Transform camera;
    camera.setPosition( Vector3f(5.0f, 5.0f, 5.0f) );
    camera.setRotation( Vector3f(-36.0f, 45.0f, 0.0f) * pi / 180.0f );


    Scene scene {
        .camera = camera,
        .model = cube
    };

    // App Setup
    QApplication app(argc, argv);
    GLSimWidget window (scene);

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
