module;
#include <algorithm>
#include <cmath>
#include <format>
#include <iostream>
#include <source_location>
export module adcs.Actuator;


export namespace adcs {

class Actuator {

    // Properties
    float mCommandPollInterval = 0.0f;
    float mMaxVoltage = 0.0f;
    float mMaxCurrent = 0.0f;
    float mResistance = 0.0f;
    float mMotorConstant = 0.0f; // Nm/Amp

    // State
    float mVoltage = 0.0f;
    float mNextVoltage = 0.0f;
    float mTimeToNextPoll = 0.0f;

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

} // export namespace adcs
