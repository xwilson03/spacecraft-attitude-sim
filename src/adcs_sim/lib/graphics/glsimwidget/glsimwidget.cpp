#include "glsimwidget.hpp"

import adcs.Transform;

#include <Eigen/Dense>
#include <QOpenGLContext>
#include <QOpenGLExtraFunctions>
#include <QOpenGLShaderProgram>
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

void GLSimWidget::initializeGL() {

    // Initialize function ptrs, set clear color, depth bit
    auto f = context()->extraFunctions();
    f->initializeOpenGLFunctions();
    f->glClearColor(0.0, 0.0, 0.0, 1.0);
    f->glEnable(GL_DEPTH_TEST);


    // Initialize vertex data containers
    f->glGenVertexArrays(1, &mVAO);
    f->glBindVertexArray(mVAO);
    
    f->glGenBuffers(1, &mVBO);
    f->glBindBuffer(GL_ARRAY_BUFFER, mVBO);
    f->glBufferData(GL_ARRAY_BUFFER, mVertices.size() * sizeof(Vertex), mVertices.data(), GL_STATIC_DRAW);

    f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*) offsetof(Vertex, x));
    f->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*) offsetof(Vertex, nx));
    f->glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*) offsetof(Vertex, r));
    f->glEnableVertexAttribArray(0);
    f->glEnableVertexAttribArray(1);
    f->glEnableVertexAttribArray(2);


    // Shader Data

    if (!mCraftProgram.addShaderFromSourceFile(QOpenGLShader::Vertex, mCraftVertPath.string().c_str())) {
        std::cerr << "Vertex shader compilation failed. What: " << mCraftProgram.log().toStdString() << std::endl;
    }
    if (!mCraftProgram.addShaderFromSourceFile(QOpenGLShader::Fragment, mCraftFragPath.string().c_str())) {
        std::cerr << "Fragment shader compilation failed. What: " << mCraftProgram.log().toStdString() << std::endl;
    }
    if (!mCraftProgram.link()) {
        std::cerr << "Shader program linking failed. What: " << mCraftProgram.log().toStdString() << std::endl;
    }
}

void GLSimWidget::paintGL() {

    const auto f = context()->extraFunctions();

    f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Recalculate mvp in case aspect ratio or model transform changed
    const auto aspectRatio = static_cast<float>(width()) / static_cast<float>(height());
    mProjectionMatrix(0, 0) = focalDistance / aspectRatio;

    mModelMatrix = mScene.model.affine().matrix();
    mViewMatrix = mScene.camera.affine().inverse().matrix();
    Eigen::Map<Eigen::Matrix4f>(mMVPQMatrix.data()) = mProjectionMatrix * mViewMatrix * mModelMatrix;

    // Set shader program and update MVP matrix
    mCraftProgram.bind();
    mCraftProgram.setUniformValue("mvp", mMVPQMatrix);

    // Copy vertex data
    f->glBindVertexArray(mVAO);
    f->glDrawArrays(GL_TRIANGLES, 0, mVertices.size());

    mCraftProgram.release();
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

    const auto resourcePaths = {
        craftMeshPath, mCraftVertPath, mCraftFragPath
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
        std::cerr
            << std::format(
                "{}: Failed during import of mesh/texture assets.",
                std::source_location::current().function_name()
            )
        << std::endl;
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
                mVertices.push_back({
                    vertex.x, vertex.y, vertex.z,
                    normal.x, normal.y, normal.z,
                    color.r, color.g, color.b
                });
            }
        }
    }
}

} // namespace adcs
