module;
#include <Eigen/Dense>
export module adcs.Spacecraft;

import adcs.Magnetorquer;
import adcs.ReactionWheel;
import adcs.Rigidbody;

using Eigen::AngleAxisf;
using Eigen::Quaternionf;
using Eigen::Vector3f;
using adcs::Magnetorquer;
using adcs::ReactionWheel;
using adcs::Rigidbody;


export namespace adcs {

class Spacecraft : public Rigidbody {

    // State
    ReactionWheel rwheelX;
    ReactionWheel rwheelY;
    ReactionWheel rwheelZ;
    Magnetorquer magtorqueX;
    Magnetorquer magtorqueY;
    Magnetorquer magtorqueZ;

public:
    
    Spacecraft() {
    
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

        const auto applyMagtorqueConfig = [&](Magnetorquer& magtorque) {
            magtorque.setCommandPollInterval(0.1f);
            magtorque.setMaxVoltage(24.0f);
            magtorque.setResistance(100.0f);
            magtorque.setInductance(1.0f);
            magtorque.setDipolePerAmp(100.0f);
        };

        applyWheelConfig(0, rwheelX);
        applyWheelConfig(1, rwheelY);
        applyWheelConfig(2, rwheelZ);

        applyMagtorqueConfig(magtorqueX);
        applyMagtorqueConfig(magtorqueY);
        applyMagtorqueConfig(magtorqueZ);
    }


    void step (
        const float absoluteTime,
        const float deltaTime
    ) {
        if (deltaTime == 0.0f) return;

        // Find craft intermediate axis
        std::vector<std::pair<float, size_t>> JSorted = {
            {mRotationalInertia.x(), 0u},
            {mRotationalInertia.y(), 1u},
            {mRotationalInertia.z(), 2u}
        };
        std::sort(JSorted.begin(), JSorted.end());
        const size_t intermediateAxis = JSorted[1].second;

        // Calculate craft ang momentum error (goal is to zero non-intermediate axes)
        Vector3f craftAngMomentumError = -angularMomentum();
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
        const Vector3f earthFieldCraft = mRotation.conjugate().matrix() * earthField;

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

        setAngularMomentum(angularMomentum() + -rwheelDeltaAngularMomentum + magtorqueImpulse);
        Rigidbody::step(deltaTime, rwheelPostStepAngularMomentum);
    }
};

} // export namespace adcs
