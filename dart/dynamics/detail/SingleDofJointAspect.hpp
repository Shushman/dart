/*
 * Copyright (c) 2015-2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
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

#ifndef DART_DYNAMICS_DETAIL_SINGLEDOFJOINTASPECT_HPP_
#define DART_DYNAMICS_DETAIL_SINGLEDOFJOINTASPECT_HPP_

#include "dart/common/RequiresAspect.hpp"

#include "dart/common/AspectWithVersion.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/dynamics/Joint.hpp"

namespace dart {
namespace dynamics {

// Forward declare the SingleDofJoint class
class SingleDofJoint;

namespace detail {

//==============================================================================
struct SingleDofJointState
{
  /// Position
  double mPosition;

  /// Generalized velocity
  double mVelocity;

  /// Generalized acceleration
  double mAcceleration;

  /// Generalized force
  double mForce;

  /// Command
  double mCommand;

  SingleDofJointState(double position = 0.0,
                      double velocity = 0.0,
                      double acceleration = 0.0,
                      double force = 0.0,
                      double command = 0.0);
};

//==============================================================================
struct SingleDofJointUniqueProperties
{
  /// Lower limit of position
  double mPositionLowerLimit;

  /// Upper limit of position
  double mPositionUpperLimit;

  /// Initial position
  double mInitialPosition;

  /// Lower limit of velocity
  double mVelocityLowerLimit;

  /// Upper limit of velocity
  double mVelocityUpperLimit;

  /// Initial velocity
  double mInitialVelocity;

  /// Lower limit of acceleration
  double mAccelerationLowerLimit;

  /// Upper limit of acceleration
  double mAccelerationUpperLimit;

  /// Lower limit of force
  double mForceLowerLimit;

  /// Upper limit of force
  double mForceUpperLimit;

  /// Joint spring stiffness
  double mSpringStiffness;

  /// Rest position for joint spring
  double mRestPosition;

  /// Joint damping coefficient
  double mDampingCoefficient;

  /// Coulomb friction force
  double mFriction;

  /// True if the name of this Joint's DOF is not allowed to be overwritten
  bool mPreserveDofName;

  /// The name of the DegreeOfFreedom for this Joint
  std::string mDofName;

  /// Constructor
  SingleDofJointUniqueProperties(
      double _positionLowerLimit = -math::constantsd::inf(),
      double _positionUpperLimit =  math::constantsd::inf(),
      double _initialPosition = 0.0,
      double _velocityLowerLimit = -math::constantsd::inf(),
      double _velocityUpperLimit =  math::constantsd::inf(),
      double _initialVelocity = 0.0,
      double _accelerationLowerLimit = -math::constantsd::inf(),
      double _accelerationUpperLimit =  math::constantsd::inf(),
      double _forceLowerLimit = -math::constantsd::inf(),
      double _forceUpperLimit =  math::constantsd::inf(),
      double _springStiffness = 0.0,
      double _restPosition = 0.0,
      double _dampingCoefficient = 0.0,
      double _coulombFriction = 0.0,
      bool _preserveDofName = false,
      const std::string& _dofName = "");

  virtual ~SingleDofJointUniqueProperties() = default;
};

//==============================================================================
struct SingleDofJointProperties :
    Joint::Properties,
    SingleDofJointUniqueProperties
{
  SingleDofJointProperties(
      const Joint::Properties& _jointProperties = Joint::Properties(),
      const SingleDofJointUniqueProperties& _singleDofProperties =
          SingleDofJointUniqueProperties());

  virtual ~SingleDofJointProperties() = default;
};

//==============================================================================
using SingleDofJointBase = common::EmbedStateAndPropertiesOnTopOf<
    SingleDofJoint, SingleDofJointState, SingleDofJointUniqueProperties, Joint>;

} // namespace detail
} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_SINGLEDOFJOINTASPECT_HPP_
