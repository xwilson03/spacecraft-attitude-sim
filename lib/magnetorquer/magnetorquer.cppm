module;
#include <algorithm>
#include <cmath>
#include <format>
#include <iostream>
#include <source_location>
#include <Eigen/Dense>
export module adcs.Magnetorquer;


export namespace adcs {

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

} // export namespace adcs
