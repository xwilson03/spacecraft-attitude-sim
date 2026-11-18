#include "glsimwidget.hpp"

import adcs.Transform;

#include <Eigen/Dense>
#include <QOpenGLBuffer>
#include <QOpenGLContext>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <filesystem>
#include <format>
#include <iostream>
#include <source_location>

using Eigen::Matrix4f;
using Eigen::Vector3f;


namespace adcs {

TexVertex GLSimWidget::createSphereVertex (
    const float phi,
    const float theta
) {
    const float x = std::sin(theta) * std::cos(phi);
    const float y = std::cos(theta);
    const float z = std::sin(theta) * std::sin(phi);
    return {
        x, y, z,
        x, y, z,
        1.0f - phi / (2.0f * adcs::pi),
        theta / adcs::pi
    };
}

std::vector<TexVertex> GLSimWidget::createSphereVertices (
    const unsigned resolution
) {
    const float deltaPhi = 2 * pi / resolution;
    const float deltaTheta = pi / resolution;

    std::vector<TexVertex> vertices;
    vertices.reserve(resolution * resolution * 6);
    for (unsigned x = 0; x < resolution; x++) {
        for (unsigned y = 0; y < resolution; y++) {

            const float phi1 = deltaPhi * x;
            const float phi2 = deltaPhi * (x + 1);
            const float theta1 = deltaTheta * y;
            const float theta2 = deltaTheta * (y + 1);

            const auto a = createSphereVertex(phi1, theta1);
            const auto b = createSphereVertex(phi1, theta2);
            const auto c = createSphereVertex(phi2, theta2);
            const auto d = createSphereVertex(phi2, theta1);

            vertices.insert(vertices.end(), {a, b, c, a, c, d});
        }
    }

    return vertices;
}

void GLSimWidget::initializeGL() {

    // Initialize function ptrs, set clear color, depth bit
    initializeOpenGLFunctions();
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    // Initialize craft object
    {
        mCraftVAO.create();
        QOpenGLVertexArrayObject::Binder vaoBinder(&mCraftVAO);

        mCraftVBO.create();
        mCraftVBO.bind();
        mCraftVBO.allocate(mCraftVertices.data(), mCraftVertices.size() * sizeof(Vertex));

        // Compile shaders
        if (!mCraftProgram.addShaderFromSourceFile(QOpenGLShader::Vertex, mCraftVertPath.string().c_str())) {
            std::cerr << "Craft vertex shader compilation failed. What: " << mCraftProgram.log().toStdString() << std::endl;
        }
        if (!mCraftProgram.addShaderFromSourceFile(QOpenGLShader::Fragment, mCraftFragPath.string().c_str())) {
            std::cerr << "Craft fragment shader compilation failed. What: " << mCraftProgram.log().toStdString() << std::endl;
        }
        if (!mCraftProgram.link()) {
            std::cerr << "Craft shader program linking failed. What: " << mCraftProgram.log().toStdString() << std::endl;
        }

        mCraftProgram.enableAttributeArray(0);
        mCraftProgram.enableAttributeArray(1);
        mCraftProgram.enableAttributeArray(2);
        mCraftProgram.setAttributeBuffer(0, GL_FLOAT, offsetof(Vertex, x),  3, sizeof(Vertex));
        mCraftProgram.setAttributeBuffer(1, GL_FLOAT, offsetof(Vertex, nx), 3, sizeof(Vertex));
        mCraftProgram.setAttributeBuffer(2, GL_FLOAT, offsetof(Vertex, r),  3, sizeof(Vertex));
    }

    // Initialize planet object
    {
        mPlanetVAO.create();
        QOpenGLVertexArrayObject::Binder vaoBinder(&mPlanetVAO);

        mPlanetVBO.create();
        mPlanetVBO.bind();
        mPlanetVBO.allocate(mPlanetVertices.data(), mPlanetVertices.size() * sizeof(TexVertex));

        // Compile shaders
        if (!mPlanetProgram.addShaderFromSourceFile(QOpenGLShader::Vertex, mPlanetVertPath.string().c_str())) {
            std::cerr << "Planet vertex shader compilation failed. What: " << mPlanetProgram.log().toStdString() << std::endl;
        }
        if (!mPlanetProgram.addShaderFromSourceFile(QOpenGLShader::Fragment, mPlanetFragPath.string().c_str())) {
            std::cerr << "Planet fragment shader compilation failed. What: " << mPlanetProgram.log().toStdString() << std::endl;
        }
        if (!mPlanetProgram.link()) {
            std::cerr << "Planet shader program linking failed. What: " << mPlanetProgram.log().toStdString() << std::endl;
        }

        mPlanetProgram.enableAttributeArray(0);
        mPlanetProgram.enableAttributeArray(1);
        mPlanetProgram.enableAttributeArray(2);
        mPlanetProgram.setAttributeBuffer(0, GL_FLOAT, offsetof(TexVertex, x),  3, sizeof(TexVertex));
        mPlanetProgram.setAttributeBuffer(1, GL_FLOAT, offsetof(TexVertex, nx), 3, sizeof(TexVertex));
        mPlanetProgram.setAttributeBuffer(2, GL_FLOAT, offsetof(TexVertex, u),  2, sizeof(TexVertex));

        mPlanetTex.setData(mPlanetImage, QOpenGLTexture::GenerateMipMaps);
        mPlanetTex.setMinificationFilter(QOpenGLTexture::LinearMipMapLinear);
        mPlanetTex.setMagnificationFilter(QOpenGLTexture::Linear);
        mPlanetTex.setWrapMode(QOpenGLTexture::DirectionS, QOpenGLTexture::Repeat);
        mPlanetTex.setWrapMode(QOpenGLTexture::DirectionT, QOpenGLTexture::ClampToEdge);
    }
}

void GLSimWidget::paintGL() {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Recalculate mvp in case aspect ratio or model transform changed
    const auto aspectRatio = static_cast<float>(width()) / static_cast<float>(height());
    mProjectionMatrix(0, 0) = focalDistance / aspectRatio;

    mCraftModelMatrix = mScene.craft.affine().matrix();
    mPlanetModelMatrix = mScene.planet.affine().matrix();
    mViewMatrix = mScene.camera.affine().inverse().matrix();
    Eigen::Map<Eigen::Matrix4f>(mCraftMVPQMatrix.data()) = mProjectionMatrix * mViewMatrix * mCraftModelMatrix;
    Eigen::Map<Eigen::Matrix4f>(mPlanetMVPQMatrix.data()) = mProjectionMatrix * mViewMatrix * mPlanetModelMatrix;

    // Draw Planet
    {
        // Set shader program and update MVP matrix
        mPlanetProgram.bind();
        mPlanetProgram.setUniformValue("aMVP", mPlanetMVPQMatrix);

        // Bind texture
        mPlanetTex.bind();

        // Bind and render mesh
        QOpenGLVertexArrayObject::Binder vaoBinder(&mPlanetVAO);
        glDrawArrays(GL_TRIANGLES, 0, mPlanetVertices.size());

        // Force planet into background
        glClear(GL_DEPTH_BUFFER_BIT);

        mPlanetTex.release();
        mPlanetProgram.release();
    }

    // Draw Spacecraft
    {
        // Set shader program and update MVP matrix
        mCraftProgram.bind();
        mCraftProgram.setUniformValue("mvp", mCraftMVPQMatrix);

        // Bind and render mesh without culling
        QOpenGLVertexArrayObject::Binder vaoBinder(&mCraftVAO);
        
        glDisable(GL_CULL_FACE);
        glDrawArrays(GL_TRIANGLES, 0, mCraftVertices.size());
        glEnable(GL_CULL_FACE);

        mCraftProgram.release();
    }
}

GLSimWidget::GLSimWidget(const Scene& aScene) 
: QOpenGLWidget()
, mScene(aScene)
{
    resize(800, 600);
    setWindowTitle("Spacecraft Attitude Sim");

    // Projection Matrix
    float aspectRatio = static_cast<float>(width()) / static_cast<float>(height());

    mProjectionMatrix(0, 0) = focalDistance / aspectRatio;                                                          // scale X to +-1 in screen-space
    mProjectionMatrix(1, 1) = focalDistance;                                                                        // scale Y to +-1 in screen-space
    mProjectionMatrix(2, 2) = (cameraNearPlane + cameraFarPlane) / (cameraFarPlane - cameraNearPlane) * -1;         // depth = (Az + B) / (-z); scale
    mProjectionMatrix(2, 3) = cameraFarPlane * 2.0f * (cameraNearPlane / (cameraFarPlane - cameraNearPlane) * -1);  //    near plane to -1 and far to +1
    mProjectionMatrix(3, 2) = -1.0f;                                                                                // divide all by -z for perspective scaling

    // Import Asset Data

    const std::filesystem::path dataDir {APP_RESOURCE_DIR};
    const auto assetsDir = dataDir / "assets";

    const auto craftMeshPath = assetsDir / "Galileo.glb";
    mCraftVertPath           = assetsDir / "galileo.vert";
    mCraftFragPath           = assetsDir / "galileo.frag";

    const auto planetImagePath = assetsDir / "HD_189733_b.jpg";
    mPlanetVertPath            = assetsDir / "HD_189733_b.vert";
    mPlanetFragPath            = assetsDir / "HD_189733_b.frag";

    const auto resourcePaths = {
        craftMeshPath, mCraftVertPath, mCraftFragPath,
        planetImagePath, mPlanetVertPath, mPlanetFragPath
    };

    for (const auto& path : resourcePaths) {

        if (std::filesystem::exists(path)) {
            std::cout << std::format("Found \"{}\".", path.string()) << "\n";
        }

        else {
            std::cerr << std::format("Unable to locate asset: {}.", path.string()) << std::endl;
            std::abort();
        }
    };

    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(
        craftMeshPath,
        aiProcess_Triangulate
        | aiProcess_PreTransformVertices
        | aiProcess_GenSmoothNormals
        | aiProcess_FlipWindingOrder
    );

    if (!scene || !scene->mRootNode) {
        std::cerr << std::format(
            "{}: Failed during import of mesh/texture assets.",
            std::source_location::current().function_name()
        ) << std::endl;
        std::abort();
    }

    for (unsigned m = 0; m < scene->mNumMeshes; m++) {
        const aiMesh* mesh = scene->mMeshes[m];
        aiColor4D color;
        scene->mMaterials[mesh->mMaterialIndex]->Get(AI_MATKEY_COLOR_DIFFUSE, color);
        for (unsigned f = 0; f < mesh->mNumFaces; f++) {
            for (unsigned v = 0; v < 3; v++) {
                const aiVector3D& vertex = mesh->mVertices[mesh->mFaces[f].mIndices[v]];
                const aiVector3D& normal = mesh->mNormals[mesh->mFaces[f].mIndices[v]];
                mCraftVertices.push_back({
                    vertex.x, vertex.y, vertex.z,
                    normal.x, normal.y, normal.z,
                    color.r, color.g, color.b
                });
            }
        }
    }

    mPlanetVertices = createSphereVertices(128);
    mPlanetImage = QImage(planetImagePath.string().c_str());
}

} // namespace adcs
