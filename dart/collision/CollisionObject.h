/*
 * Copyright (c) 2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#ifndef DART_COLLISION_COLLISIONOBJECT_H_
#define DART_COLLISION_COLLISIONOBJECT_H_

#include <cstddef>
#include <Eigen/Dense>

#include "dart/collision/Contact.h"
#include "dart/collision/Engine.h"
#include "dart/collision/CollisionObjectData.h"

namespace dart {
namespace collision {

class CollisionObjectData;

class CollisionObject
{
public:

  virtual const Eigen::Isometry3d getTransform() const = 0;

  /// Return shape pointer that associated with this CollisionObject
  dynamics::ShapePtr getShape() const;
  // TODO(JS): Shape should be in common or math

  // virtual void reportCollision();

  EngineType getEngineType() const;

  // virtual void changeEngine(EngineType type) {}

  CollisionObjectData* getEngineData() const;

  /// Update engine data. This function should be called before the collision
  /// detection is performed by the engine in most cases.
  void updateEngineData();

  /// Perform collision detection with other CollisionObject. Return false if
  /// the engine type of the other CollisionObject is different from this
  /// CollisionObject's.
  bool detect(CollisionObject* other,
              const collision::Option& option,
              collision::Result& result);

  // bool detect(CollisionGroup* group,
  //             const collision::Option& option,
  //             collision::Result& result);

protected:

  /// Contructor
  CollisionObject(const dynamics::ShapePtr& shape, EngineType type);

protected:

  /// Shape
  dynamics::ShapePtr mShape;

  /// Collision engine type
  EngineType mEngineType;

  /// Engine specific data
  std::shared_ptr<CollisionObjectData> mEngineData;

};

/// FreeCollisionOjbect is a basic collision object whose transformation can be
/// set freely.
class FreeCollisionObject : public CollisionObject
{
public:

  /// Constructor
  FreeCollisionObject(
      const dynamics::ShapePtr& shape,
      const Eigen::Isometry3d& tf = Eigen::Isometry3d::Identity(),
      EngineType type = FCL);

  /// Set world transformation of this FreeCollisionObject
  void setTransform(const Eigen::Isometry3d& tf);

  /// Set world rotation of this FreeCollisionObject
  void setRotation(const Eigen::Matrix3d& rotation);

  /// Set world translation of this FreeCollisionObject
  void setTranslation(const Eigen::Vector3d& translation);

  /// Return world transformation of this FreeCollisionObject
  const Eigen::Isometry3d getTransform() const override;

protected:

  /// Transformation in world coordinates
  Eigen::Isometry3d mW;

};

using CollisionObjectPtr = std::shared_ptr<CollisionObject>;

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_COLLISIONOBJECT_H_