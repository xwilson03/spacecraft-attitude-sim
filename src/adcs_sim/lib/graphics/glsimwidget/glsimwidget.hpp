import adcs.Transform;

#include <Eigen/Dense>
#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include <QOpenGLExtraFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <cmath>
#include <filesystem>
#include <numbers>

using Eigen::Matrix4f;
using Eigen::Vector3f;
using adcs::Transform;


namespace adcs {

struct Scene {
    Transform& camera;
    Transform& craft;
};

struct Vertex {
    float x, y, z;
    float nx, ny, nz;
    float r, g, b;
};

class GLSimWidget : public QOpenGLWidget , protected QOpenGLExtraFunctions {

    Q_OBJECT

    // Spacecraft
    std::vector<Vertex> mCraftVertices;
    QOpenGLVertexArrayObject mCraftVAO;
    QOpenGLBuffer mCraftVBO {QOpenGLBuffer::VertexBuffer};

    std::filesystem::path mCraftVertPath;
    std::filesystem::path mCraftFragPath;
    QOpenGLShaderProgram  mCraftProgram;

    Matrix4f mCraftModelMatrix = Matrix4f::Zero();
    QMatrix4x4 mCraftMVPQMatrix;

    // Camera & Scene
    const Scene& mScene;
    Matrix4f mViewMatrix = Matrix4f::Zero();
    Matrix4f mProjectionMatrix = Matrix4f::Zero();

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
