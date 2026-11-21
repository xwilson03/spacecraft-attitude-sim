import adcs.Transform;

#include <Eigen/Dense>
#include <QImage>
#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include <QOpenGLExtraFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <cmath>
#include <filesystem>
#include <numbers>

using Eigen::Matrix4f;
using adcs::Transform;


namespace adcs {

const float pi = std::numbers::pi_v<float>;

struct Scene {
    Transform& camera;
    Transform& craft;
    Transform& planet;
};

struct Vertex {
    float x, y, z;
    float nx, ny, nz;
    float r, g, b;
};

struct TexVertex {
    float x, y, z;
    float nx, ny, nz;
    float u, v;
};

struct Star {
    float x, y, z;
    float size, tint;
};

struct SimpleVertex {
    float x, y, z;
};

class GLSimWidget : public QOpenGLWidget , protected QOpenGLExtraFunctions {

    Q_OBJECT

    // Spacecraft
    std::vector<Vertex> mCraftVertices;
    QOpenGLVertexArrayObject mCraftVAO;
    QOpenGLBuffer mCraftVBO {QOpenGLBuffer::VertexBuffer};
    QMatrix4x4 mCraftMVPMatrix;

    std::filesystem::path mCraftVertPath;
    std::filesystem::path mCraftFragPath;
    QOpenGLShaderProgram  mCraftProgram;

    // Planet
    std::vector<TexVertex> mPlanetVertices;
    QOpenGLVertexArrayObject mPlanetVAO;
    QOpenGLBuffer mPlanetVBO {QOpenGLBuffer::VertexBuffer};
    QOpenGLTexture mPlanetTex {QOpenGLTexture::Target2D};
    QImage mPlanetImage;
    QMatrix4x4 mPlanetMVPMatrix;

    std::filesystem::path mPlanetVertPath;
    std::filesystem::path mPlanetFragPath;
    QOpenGLShaderProgram  mPlanetProgram;

    // Atmosphere
    std::vector<SimpleVertex> mAtmoVertices;
    QOpenGLVertexArrayObject mAtmoVAO;
    QOpenGLBuffer mAtmoVBO {QOpenGLBuffer::VertexBuffer};
    QMatrix4x4 mAtmoMVPMatrix;

    std::filesystem::path mAtmoVertPath;
    std::filesystem::path mAtmoFragPath;
    QOpenGLShaderProgram  mAtmoProgram;

    // Stars
    std::vector<Star> mStars;
    QOpenGLVertexArrayObject mStarsVAO;
    QOpenGLBuffer mStarsVBO {QOpenGLBuffer::VertexBuffer};
    QMatrix4x4 mStarsMVPMatrix;

    std::filesystem::path mStarsVertPath;
    std::filesystem::path mStarsFragPath;
    QOpenGLShaderProgram  mStarsProgram;

    // Camera & Scene
    const Scene& mScene;
    Matrix4f mProjectionMatrix;

    const float cameraVertFovDeg = 70.0f;
    const float cameraNearPlane = 0.01f;
    const float cameraFarPlane = 100.0f;
    // use vertical FOV to compute distance from camera where screen height = 2 world units (+-1)
    const float focalDistance = 1.0f / std::tan((cameraVertFovDeg * pi / 180.0f) / 2.0f);


    TexVertex createSphereVertex (const float phi, const float theta);
    std::vector<TexVertex> createSphereVertices (const unsigned resolution);
    
    SimpleVertex createSimpleSphereVertex (const float phi, const float theta);
    std::vector<SimpleVertex> createSimpleSphereVertices (const unsigned resolution);
    
    std::vector<Star> createStars (const unsigned count, const unsigned seed);

    void initializeGL() override;
    void paintGL() override;

public:

    GLSimWidget(const Scene& aScene);
};

} // namespace adcs
