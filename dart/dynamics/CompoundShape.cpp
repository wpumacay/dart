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

#include "dart/dynamics/CompoundShape.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include <cmath>
#include <limits>

namespace dart {
namespace dynamics {

//==============================================================================
CompoundShape::CompoundShape() : Shape(COMPOUND)
{
  // Do nothing
}

//==============================================================================
CompoundShape::~CompoundShape()
{
  mChildren.clear();
  mChildrenTfs.clear();
}

//==============================================================================
const std::string& CompoundShape::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& CompoundShape::getStaticType()
{
  static const std::string type("CompoundShape");
  return type;
}

//==============================================================================

void CompoundShape::addChild( ShapePtr child_shape, const Eigen::Isometry3d& child_tf )
{
  mChildren.push_back(child_shape);
  mChildrenTfs.push_back(child_tf);
}

//==============================================================================

std::vector<const Shape*> CompoundShape::children() const
{
  std::vector<const Shape*> children_shapes;
  for (auto shape_ptr : mChildren)
    children_shapes.push_back(shape_ptr.get());
  return children_shapes;
}

//==============================================================================
Eigen::Matrix3d CompoundShape::computeInertia(double mass) const
{
  return BoxShape::computeInertia(getBoundingBox().computeFullExtents(), mass);
}

//==============================================================================
void CompoundShape::updateBoundingBox() const
{
  double max_X = -std::numeric_limits<double>::infinity();
  double max_Y = -std::numeric_limits<double>::infinity();
  double max_Z = -std::numeric_limits<double>::infinity();
  double min_X = std::numeric_limits<double>::infinity();
  double min_Y = std::numeric_limits<double>::infinity();
  double min_Z = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < mChildren.size(); i++ )
  {
    const auto child_shape = mChildren[i];
    const auto aabb_min = child_shape->getBoundingBox().getMin();
    const auto aabb_max = child_shape->getBoundingBox().getMax();

    if (aabb_max.x() > max_X) max_X = aabb_max.x();
    if (aabb_min.x() < min_X) min_X = aabb_min.x();
    if (aabb_max.y() > max_Y) max_Y = aabb_max.y();
    if (aabb_min.y() < min_Y) min_Y = aabb_min.y();
    if (aabb_max.z() > max_Z) max_Z = aabb_max.z();
    if (aabb_min.z() < min_Z) min_Z = aabb_min.z();
  }

  mBoundingBox.setMin(Eigen::Vector3d(min_X, min_Y, min_Z));
  mBoundingBox.setMax(Eigen::Vector3d(max_X, max_Y, max_Z));
  mIsBoundingBoxDirty = false;
}

//==============================================================================
void CompoundShape::updateVolume() const
{
  mVolume = BoxShape::computeVolume(getBoundingBox().computeFullExtents());
  mIsVolumeDirty = false;
}

} // namespace dynamics
} // namespace dart
