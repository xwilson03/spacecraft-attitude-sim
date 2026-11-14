module;
#include <Eigen/Dense>
export module adcs.Rigidbody;

import adcs.Transform;

using Eigen::AngleAxisf;
using Eigen::Quaternionf;
using Eigen::Vector3f;
using adcs::Transform;


export namespace adcs {

class Rigidbody : public Transform {

protected:

    Vector3f mAngularVelocity   = Vector3f::Zero();
    Vector3f mTorque            = Vector3f::Zero();
    Vector3f mRotationalInertia;

private:

    Vector3f computeAngularAcceleration(const Vector3f& initialVelocity, const Vector3f& internalMomentum) const {
        const Vector3f L = initialVelocity.cwiseProduct(mRotationalInertia) + internalMomentum;
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

    void step (const float deltaTime, const Vector3f internalMomentum = Vector3f::Zero()) {

        if (deltaTime == 0.0f) return;

        // Update angular velocity (RK4)
        const Vector3f k1 = computeAngularAcceleration(mAngularVelocity, internalMomentum);
        const Vector3f k2 = computeAngularAcceleration(mAngularVelocity + 0.5f * deltaTime * k1, internalMomentum);
        const Vector3f k3 = computeAngularAcceleration(mAngularVelocity + 0.5f * deltaTime * k2, internalMomentum);
        const Vector3f k4 = computeAngularAcceleration(mAngularVelocity + deltaTime * k3, internalMomentum);

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

} // export namespace adcs
