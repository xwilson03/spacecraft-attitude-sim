module;
#include <Eigen/Dense>
#include <format>
#include <iostream>
#include <source_location>
export module adcs.ReactionWheel;

import adcs.Actuator;
import adcs.PDController;
import adcs.Rigidbody;

using Eigen::Vector3f;
using adcs::Actuator;
using adcs::PDController;
using adcs::Rigidbody;


export namespace adcs {

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

    void setNextAngularMomentumError (const auto angularMomentumError) {
        mNextAngularMomentumError = angularMomentumError;
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

    void setActuatorControllerKP (const auto kP) {
        mActuatorController.setKP(kP);
    }

    void setActuatorControllerKD (const auto kD) {
        mActuatorController.setKD(kD);
    }

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

} // export namespace adcs
