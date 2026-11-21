#define EIGEN_DEFAULT_IO_FORMAT Eigen::IOFormat(4, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]")

#include <Eigen/Dense>
#include "simsnooper.hpp"

import adcs.Rigidbody;
import adcs.Spacecraft;

using Eigen::Vector3f;
using adcs::Rigidbody;
using adcs::SimSnooper;
using adcs::Spacecraft;


int main(int argc, char* argv[]) {

    // State
    const float& pi = std::numbers::pi_v<float>;

    Spacecraft craft;
    craft.setScale(Vector3f(1,1,1) / 64);
    craft.setRotationalInertia( Vector3f(4454.7f, 4061.2f, 5967.6f) ); // kgm^2. Galileo's pre-launch values (https://arxiv.org/pdf/gr-qc/0104064 section F.1)
    craft.setAngularVelocity( Vector3f(pi / 10.0f, 0.01f, 0.01f) );    // rad/s. Galileo spun at 3rpm around its major axis; we'll spin around the intermediate.

    Rigidbody planet;
    planet.setRotation( Vector3f(-45.0f, 0.0f, 15.0f) * pi / 180.0f );
    planet.setPosition( Vector3f(5.5f, 4.0f, 4.5f) );
    planet.setAngularVelocity( Vector3f(0.f, -0.01f, 0.0f) );

    // App Setup
    rclcpp::init(argc, argv);
    auto snooper = std::make_shared<SimSnooper>(craft);

    float absoluteTime = 0.0f;

    QTimer physics;
    QObject::connect(&physics, &QTimer::timeout, [&]() {
        const float deltaTime = 1e-3f;
        absoluteTime += deltaTime;
        craft.step(absoluteTime, deltaTime);
        planet.step(deltaTime);
        snooper->publish();
    });
    physics.start(1);

    rclcpp::shutdown();
    return status;
}
