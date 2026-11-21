#define EIGEN_DEFAULT_IO_FORMAT Eigen::IOFormat(4, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]")

#include <Eigen/Dense>
#include <QApplication>
#include <QTimer>

import adcs.GLSimWidget;
import adcs.Spacecraft;
import adcs.Transform;

using Eigen::Vector3f;
using adcs::Scene;
using adcs::Spacecraft;
using adcs::Transform;


int main(int argc, char* argv[]) {

    // State
    const float& pi = std::numbers::pi_v<float>;

    Transform camera;
    camera.setPosition( Vector3f(5.0f, 5.0f, 5.0f) );
    camera.setRotation( Vector3f(-36.0f, 45.0f, 0.0f) * pi / 180.0f );

    // App Setup
    rclcpp::init(argc, argv);
    QApplication app (argc, argv);
    GLSimWidget window (scene);

    QTimer display;
    QObject::connect(&display, &QTimer::timeout, [&]() { window.update(); });
    display.start(static_cast<int>(1.0f / 120.0f * 1000.0f));

    window.show();
    int status = app.exec();
    rclcpp::shutdown();
    return status;
}
