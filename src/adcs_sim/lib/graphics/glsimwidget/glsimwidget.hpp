#include <Eigen/Dense>
#include <QOpenGLWidget>
#include <array>
#include <cmath>
#include <numbers>

import adcs.Transform;

using Eigen::Matrix4f;
using Eigen::Vector3f;
using adcs::Transform;


namespace adcs {

struct Scene {
    Transform& camera;
    Transform& model;
};

struct Vertex {
    float x, y, z;
};

class GLSimWidget : public QOpenGLWidget {

    Q_OBJECT

    std::array<Vertex, 8> mVertices;
    std::array<unsigned, 36> mIndices;
    unsigned int mVAO;
    unsigned int mVBO;
    unsigned int mEBO;
    unsigned int shaderProgram;

    const Scene& mScene;
    Matrix4f mModelMatrix = Matrix4f::Zero();
    Matrix4f mViewMatrix = Matrix4f::Zero();
    Matrix4f mProjectionMatrix = Matrix4f::Zero();
    Matrix4f mvp = Matrix4f::Zero();

    const float cameraVertFovDeg = 70.0f;
    const float cameraNearPlane = 0.1f;
    const float cameraFarPlane = 100.0f;
    // use vertical FOV to compute distance from camera where screen height = 2 world units (+-1)
    const float focalDistance = 1.0f / std::tan((cameraVertFovDeg * std::numbers::pi_v<float> / 180.0f) / 2.0f);

    void initializeGL() override;
    void paintGL() override;

public:

    GLSimWidget(const Scene& aScene);
};

} // namespace adcs
