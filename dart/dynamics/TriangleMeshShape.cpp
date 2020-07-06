/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/dynamics/TriangleMeshShape.hpp"

#include <limits>
#include <string>
#include <stack>

#include <assimp/Importer.hpp>
#include <assimp/cimport.h>
#include <assimp/postprocess.h>

#include "dart/common/Console.hpp"
#include "dart/common/LocalResourceRetriever.hpp"
#include "dart/common/Uri.hpp"
#include "dart/config.hpp"
#include "dart/dynamics/AssimpInputResourceAdaptor.hpp"
#include "dart/dynamics/BoxShape.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
TriangleMeshShape::TriangleMeshShape(
    const Eigen::Vector3d& scale,
    const aiScene* mesh,
    const common::Uri& path,
    common::ResourceRetrieverPtr resourceRetriever)
  : Shape(TRIANGLE_MESH),
    mDisplayList(0),
    mColorMode(MATERIAL_COLOR),
    mAlphaMode(BLEND),
    mColorIndex(0)
{
  mScale = scale;
  setMesh(mesh, path, std::move(resourceRetriever));
  setScale(scale);
}

//==============================================================================
TriangleMeshShape::~TriangleMeshShape()
{
  aiReleaseImport(mMesh);
}

//==============================================================================
const std::string& TriangleMeshShape::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& TriangleMeshShape::getStaticType()
{
  static const std::string type("TriangleMeshShape");
  return type;
}

//==============================================================================
const aiScene* TriangleMeshShape::getMesh() const
{
  return mMesh;
}

//==============================================================================
std::string TriangleMeshShape::getMeshUri() const
{
  return mMeshUri.toString();
}

//==============================================================================
const common::Uri& TriangleMeshShape::getMeshUri2() const
{
  return mMeshUri;
}

//==============================================================================
void TriangleMeshShape::update()
{
  // Do nothing
}

//==============================================================================
const std::string& TriangleMeshShape::getMeshPath() const
{
  return mMeshPath;
}

//==============================================================================
common::ResourceRetrieverPtr TriangleMeshShape::getResourceRetriever()
{
  return mResourceRetriever;
}

//==============================================================================
void TriangleMeshShape::setMesh(
    const aiScene* mesh,
    const std::string& path,
    common::ResourceRetrieverPtr resourceRetriever)
{
  setMesh(mesh, common::Uri(path), std::move(resourceRetriever));
}

//==============================================================================
void TriangleMeshShape::setMesh(
    const aiScene* mesh,
    const common::Uri& uri,
    common::ResourceRetrieverPtr resourceRetriever)
{
  mMesh = mesh;

  if (!mMesh)
  {
    mMeshUri.clear();
    mMeshPath.clear();
    mResourceRetriever = nullptr;
    return;
  }

  mMeshUri = uri;

  if (resourceRetriever)
    mMeshPath = resourceRetriever->getFilePath(uri);
  else
    mMeshPath.clear();

  mResourceRetriever = std::move(resourceRetriever);
  collectMeshData(mMesh);
  incrementVersion();
}

//==============================================================================
void TriangleMeshShape::collectMeshData(const aiScene* mesh)
{
  mVertexData.clear();
  mIndexData.clear();

  std::stack<const aiNode*> dfs_traversal;
  dfs_traversal.push( mesh->mRootNode );
  while( !dfs_traversal.empty() )
  {
    auto assimp_node = dfs_traversal.top();
    dfs_traversal.pop();
    if ( !assimp_node )
      continue;

    for ( ssize_t i = 0; i < assimp_node->mNumMeshes; i++ )
    {
      auto assimp_mesh = mesh->mMeshes[assimp_node->mMeshes[i]];
      for ( ssize_t v = 0; v < assimp_mesh->mNumVertices; v++ )
      {
        mVertexData.push_back( mScale.x() * assimp_mesh->mVertices[v].x );
        mVertexData.push_back( mScale.y() * assimp_mesh->mVertices[v].y );
        mVertexData.push_back( mScale.z() * assimp_mesh->mVertices[v].z );
      }
      for ( ssize_t f = 0; f < assimp_mesh->mNumFaces; f++ )
      {
        auto assimp_face = assimp_mesh->mFaces[f];
        mIndexData.push_back( assimp_face.mIndices[0] );
        mIndexData.push_back( assimp_face.mIndices[1] );
        mIndexData.push_back( assimp_face.mIndices[2] );
      }
    }

    for ( ssize_t i = 0; i < assimp_node->mNumChildren; i++ )
        dfs_traversal.push( assimp_node->mChildren[i] );
  }
}

//==============================================================================
void TriangleMeshShape::setScale(const Eigen::Vector3d& scale)
{
  assert((scale.array() > 0.0).all());

  mScale = scale;
  mIsBoundingBoxDirty = true;
  mIsVolumeDirty = true;

  incrementVersion();
}

//==============================================================================
const Eigen::Vector3d& TriangleMeshShape::getScale() const
{
  return mScale;
}

//==============================================================================
void TriangleMeshShape::setColorMode(ColorMode mode)
{
  mColorMode = mode;
}

//==============================================================================
TriangleMeshShape::ColorMode TriangleMeshShape::getColorMode() const
{
  return mColorMode;
}

//==============================================================================
void TriangleMeshShape::setAlphaMode(TriangleMeshShape::AlphaMode mode)
{
  mAlphaMode = mode;
}

//==============================================================================
TriangleMeshShape::AlphaMode TriangleMeshShape::getAlphaMode() const
{
  return mAlphaMode;
}

//==============================================================================
void TriangleMeshShape::setColorIndex(int index)
{
  mColorIndex = index;
}

//==============================================================================
int TriangleMeshShape::getColorIndex() const
{
  return mColorIndex;
}

//==============================================================================
int TriangleMeshShape::getDisplayList() const
{
  return mDisplayList;
}

//==============================================================================
void TriangleMeshShape::setDisplayList(int index)
{
  mDisplayList = index;
}

//==============================================================================
Eigen::Matrix3d TriangleMeshShape::computeInertia(double _mass) const
{
  // Use bounding box to represent the mesh
  return BoxShape::computeInertia(getBoundingBox().computeFullExtents(), _mass);
}

//==============================================================================
void TriangleMeshShape::updateBoundingBox() const
{
  if (!mMesh)
  {
    mBoundingBox.setMin(Eigen::Vector3d::Zero());
    mBoundingBox.setMax(Eigen::Vector3d::Zero());
    mIsBoundingBoxDirty = false;
    return;
  }

  double max_X = -std::numeric_limits<double>::infinity();
  double max_Y = -std::numeric_limits<double>::infinity();
  double max_Z = -std::numeric_limits<double>::infinity();
  double min_X = std::numeric_limits<double>::infinity();
  double min_Y = std::numeric_limits<double>::infinity();
  double min_Z = std::numeric_limits<double>::infinity();

  for (unsigned int i = 0; i < mMesh->mNumMeshes; i++)
  {
    for (unsigned int j = 0; j < mMesh->mMeshes[i]->mNumVertices; j++)
    {
      if (mMesh->mMeshes[i]->mVertices[j].x > max_X)
        max_X = mMesh->mMeshes[i]->mVertices[j].x;
      if (mMesh->mMeshes[i]->mVertices[j].x < min_X)
        min_X = mMesh->mMeshes[i]->mVertices[j].x;
      if (mMesh->mMeshes[i]->mVertices[j].y > max_Y)
        max_Y = mMesh->mMeshes[i]->mVertices[j].y;
      if (mMesh->mMeshes[i]->mVertices[j].y < min_Y)
        min_Y = mMesh->mMeshes[i]->mVertices[j].y;
      if (mMesh->mMeshes[i]->mVertices[j].z > max_Z)
        max_Z = mMesh->mMeshes[i]->mVertices[j].z;
      if (mMesh->mMeshes[i]->mVertices[j].z < min_Z)
        min_Z = mMesh->mMeshes[i]->mVertices[j].z;
    }
  }
  mBoundingBox.setMin(
      Eigen::Vector3d(min_X * mScale[0], min_Y * mScale[1], min_Z * mScale[2]));
  mBoundingBox.setMax(
      Eigen::Vector3d(max_X * mScale[0], max_Y * mScale[1], max_Z * mScale[2]));

  mIsBoundingBoxDirty = false;
}

//==============================================================================
void TriangleMeshShape::updateVolume() const
{
  const Eigen::Vector3d bounds = getBoundingBox().computeFullExtents();
  mVolume = bounds.x() * bounds.y() * bounds.z();
  mIsVolumeDirty = false;
}

//==============================================================================
const aiScene* TriangleMeshShape::loadMesh(
    const std::string& _uri, const common::ResourceRetrieverPtr& retriever)
{
  // Remove points and lines from the import.
  aiPropertyStore* propertyStore = aiCreatePropertyStore();
  aiSetImportPropertyInteger(
      propertyStore,
      AI_CONFIG_PP_SBP_REMOVE,
      aiPrimitiveType_POINT | aiPrimitiveType_LINE);

  // Wrap ResourceRetriever in an IOSystem from Assimp's C++ API.  Then wrap
  // the IOSystem in an aiFileIO from Assimp's C API. Yes, this API is
  // completely ridiculous...
  AssimpInputResourceRetrieverAdaptor systemIO(retriever);
  aiFileIO fileIO = createFileIO(&systemIO);

  // Import the file.
  const aiScene* scene = aiImportFileExWithProperties(
      _uri.c_str(),
      aiProcess_GenNormals | aiProcess_Triangulate
          | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType
          | aiProcess_OptimizeMeshes,
      &fileIO,
      propertyStore);

  // If succeeded, store the importer in the scene to keep it alive. This is
  // necessary because the importer owns the memory that it allocates.
  if (!scene)
  {
    dtwarn << "[TriangleMeshShape::loadMesh] Failed loading mesh '" << _uri << "'.\n";
    aiReleasePropertyStore(propertyStore);
    return nullptr;
  }

  // Assimp rotates collada files such that the up-axis (specified in the
  // collada file) aligns with assimp's y-axis. Here we are reverting this
  // rotation. We are only catching files with the .dae file ending here. We
  // might miss files with an .xml file ending, which would need to be looked
  // into to figure out whether they are collada files.
  std::string extension;
  const std::size_t extensionIndex = _uri.find_last_of('.');
  if (extensionIndex != std::string::npos)
    extension = _uri.substr(extensionIndex);

  std::transform(
      std::begin(extension),
      std::end(extension),
      std::begin(extension),
      ::tolower);

  if (extension == ".dae" || extension == ".zae")
    scene->mRootNode->mTransformation = aiMatrix4x4();

  // Finally, pre-transform the vertices. We can't do this as part of the
  // import process, because we may have changed mTransformation above.
  scene = aiApplyPostProcessing(scene, aiProcess_PreTransformVertices);
  if (!scene)
    dtwarn << "[TriangleMeshShape::loadMesh] Failed pre-transforming vertices.\n";

  aiReleasePropertyStore(propertyStore);

  return scene;
}

//==============================================================================
const aiScene* TriangleMeshShape::loadMesh(
    const common::Uri& uri, const common::ResourceRetrieverPtr& retriever)
{
  return loadMesh(uri.toString(), retriever);
}

//==============================================================================
const aiScene* TriangleMeshShape::loadMesh(const std::string& filePath)
{
  const auto retriever = std::make_shared<common::LocalResourceRetriever>();
  return loadMesh("file://" + filePath, retriever);
}

} // namespace dynamics
} // namespace dart
