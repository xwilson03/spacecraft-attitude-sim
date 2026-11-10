#define EIGEN_DEFAULT_IO_FORMAT Eigen::IOFormat(4, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]")

#include <Eigen/Dense>
#include <QApplication>
#include <QTimer>
#include "simsnooper.hpp"

import adcs.GLSimWidget;
import adcs.Spacecraft;
import adcs.Transform;

using Eigen::Vector3f;
using adcs::GLSimWidget;
using adcs::Scene;
using adcs::SimSnooper;
using adcs::Spacecraft;
using adcs::Transform;


int main(int argc, char* argv[]) {

    // State
    const float& pi = std::numbers::pi_v<float>;

    Transform camera;
    camera.setPosition( Vector3f(5.0f, 5.0f, 5.0f) );
    camera.setRotation( Vector3f(-36.0f, 45.0f, 0.0f) * pi / 180.0f );

    Spacecraft craft;
    craft.setScale( Vector3f(1.0f, 3.0f, 5.0f) );
    craft.setAngularVelocity( Vector3f(30.0f, 90.0f, 30.0f) * pi / 180.0f );

    Scene scene {
        .camera = camera,
        .model = craft
    };

    // App Setup
    rclcpp::init(argc, argv);
    QApplication app (argc, argv);
    GLSimWidget window (scene);
    auto snooper = std::make_shared<SimSnooper>(craft);

    float absoluteTime = 0.0f;

    QTimer physics;
    QObject::connect(&physics, &QTimer::timeout, [&]() {
        const float deltaTime = 1e-3f;
        absoluteTime += deltaTime;
        craft.step(absoluteTime, deltaTime);
        snooper->publish();
    });
    physics.start(1);

    QTimer display;
    QObject::connect(&display, &QTimer::timeout, [&]() { window.update(); });
    display.start(static_cast<int>(1.0f / 120.0f * 1000.0f));

    window.show();
    int status = app.exec();
    rclcpp::shutdown();
    return status;
}
