module;
#include <Eigen/Dense>
export module adcs.Transform;

using Eigen::Affine3f;
using Eigen::AngleAxisf;
using Eigen::Quaternionf;
using Eigen::Vector3f;


export namespace adcs {

class Transform {

protected:

    Vector3f    mPosition = Vector3f::Zero();
    Quaternionf mRotation = Quaternionf::Identity();
    Vector3f    mScale    = Vector3f(1.0f, 1.0f, 1.0f);

public:

    const auto& position () const { return mPosition; };
    const auto& rotation () const { return mRotation; };
    const auto& scale    () const { return mScale;    };

    auto affine() const {
        auto transform = Affine3f::Identity();
        transform.translate(mPosition);
        transform.rotate(mRotation);
        transform.scale(mScale);
        return transform;
    }

    void setPosition (const Vector3f& aPosition) { mPosition = aPosition; };
    void setScale    (const Vector3f& aScale)    { mScale = aScale; };

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
            << "Position: " << instance.mPosition << "\n"
            << "Rotation: " << instance.mRotation.coeffs() << "\n"
            << "Scale: " << instance.mScale << "\n";
    }
};

} // export namespace adcs
