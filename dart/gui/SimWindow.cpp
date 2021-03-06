/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>
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
 *   * This code incorporates portions of Open Dynamics Engine
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
 *
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

#include "dart/gui/SimWindow.h"

#include <cstdio>
#include <iostream>
#include <string>

#include "dart/simulation/World.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/SoftBodyNode.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/dynamics/EllipsoidShape.h"
#include "dart/dynamics/CylinderShape.h"
#include "dart/dynamics/PlaneShape.h"
#include "dart/dynamics/MeshShape.h"
#include "dart/dynamics/SoftMeshShape.h"
#include "dart/dynamics/LineSegmentShape.h"
#include "dart/constraint/ConstraintSolver.h"
#include "dart/collision/CollisionDetector.h"
#include "dart/gui/LoadGlut.h"
#include "dart/gui/GLFuncs.h"
#include "dart/gui/GraphWindow.h"
#include "dart/utils/FileInfoWorld.h"

namespace dart {
namespace gui {

SimWindow::SimWindow()
  : Win3D() {
  mBackground[0] = 1.0;
  mBackground[1] = 1.0;
  mBackground[2] = 1.0;
  mBackground[3] = 1.0;

  mPlay = false;
  mSimulating = false;
  mPlayFrame = 0;
  mShowPointMasses = false;
  mShowMarkers = true;
  mPersp = 45.f;
  mTrans[1] = 300.f;
}

SimWindow::~SimWindow() {
  for (const auto& graphWindow : mGraphWindows)
    delete graphWindow;
}

void SimWindow::timeStepping() {
  mWorld->step();
}

//==============================================================================
void SimWindow::drawWorld() const
{
  drawSkeletons();

  for (auto i = 0u; i < mWorld->getNumSimpleFrames(); ++i)
    drawShapeFrame(mWorld->getSimpleFrame(i).get());
}

//==============================================================================
void SimWindow::drawSkeletons() const
{
  for (auto i = 0u; i < mWorld->getNumSkeletons(); ++i)
    drawSkeleton(mWorld->getSkeleton(i).get());
}

//==============================================================================
void SimWindow::drawSkels()
{
  drawSkeletons();
}

//==============================================================================
void SimWindow::drawEntities()
{
  for (size_t i = 0; i < mWorld->getNumSimpleFrames(); ++i)
    drawShapeFrame(mWorld->getSimpleFrame(i).get());
}

void SimWindow::displayTimer(int _val) {
  int numIter = mDisplayTimeout / (mWorld->getTimeStep() * 1000);
  if (mPlay) {
    mPlayFrame += 16;
    if (mPlayFrame >= mWorld->getRecording()->getNumFrames())
      mPlayFrame = 0;
  } else if (mSimulating) {
    for (int i = 0; i < numIter; i++) {
      timeStepping();
      mWorld->bake();
    }
  }
  glutPostRedisplay();
  glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

void SimWindow::draw() {
  glDisable(GL_LIGHTING);
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  if (!mSimulating) {
      if (mPlayFrame < mWorld->getRecording()->getNumFrames()) {
      size_t nSkels = mWorld->getNumSkeletons();
      for (size_t i = 0; i < nSkels; i++) {
        // size_t start = mWorld->getIndex(i);
        // size_t size = mWorld->getSkeleton(i)->getNumDofs();
        mWorld->getSkeleton(i)->setPositions(mWorld->getRecording()->getConfig(mPlayFrame, i));
      }
      if (mShowMarkers) {
        // size_t sumDofs = mWorld->getIndex(nSkels);
        int nContact = mWorld->getRecording()->getNumContacts(mPlayFrame);
        for (int i = 0; i < nContact; i++) {
            Eigen::Vector3d v = mWorld->getRecording()->getContactPoint(mPlayFrame, i);
            Eigen::Vector3d f = mWorld->getRecording()->getContactForce(mPlayFrame, i);

          glBegin(GL_LINES);
          glVertex3f(v[0], v[1], v[2]);
          glVertex3f(v[0] + f[0], v[1] + f[1], v[2] + f[2]);
          glEnd();
          mRI->setPenColor(Eigen::Vector3d(0.2, 0.2, 0.8));
          mRI->pushMatrix();
          glTranslated(v[0], v[1], v[2]);
          mRI->drawEllipsoid(Eigen::Vector3d(0.02, 0.02, 0.02));
          mRI->popMatrix();
        }
      }
    }
  } else {
    if (mShowMarkers) {
      collision::CollisionDetector* cd =
          mWorld->getConstraintSolver()->getCollisionDetector();
      for (size_t k = 0; k < cd->getNumContacts(); k++) {
        Eigen::Vector3d v = cd->getContact(k).point;
        Eigen::Vector3d f = cd->getContact(k).force / 10.0;
        glBegin(GL_LINES);
        glVertex3f(v[0], v[1], v[2]);
        glVertex3f(v[0] + f[0], v[1] + f[1], v[2] + f[2]);
        glEnd();
        mRI->setPenColor(Eigen::Vector3d(0.2, 0.2, 0.8));
        mRI->pushMatrix();
        glTranslated(v[0], v[1], v[2]);
        mRI->drawEllipsoid(Eigen::Vector3d(0.02, 0.02, 0.02));
        mRI->popMatrix();
      }
    }
  }

  drawWorld();

  // display the frame count in 2D text
  char buff[64];
  if (!mSimulating)
#ifdef _WIN32
    _snprintf(buff, sizeof(buff), "%d", mPlayFrame);
#else
    std::snprintf(buff, sizeof(buff), "%d", mPlayFrame);
#endif
  else
#ifdef _WIN32
    _snprintf(buff, sizeof(buff), "%d", mWorld->getSimFrames());
#else
    std::snprintf(buff, sizeof(buff), "%d", mWorld->getSimFrames());
#endif
  std::string frame(buff);
  glColor3f(0.0, 0.0, 0.0);
  gui::drawStringOnScreen(0.02f, 0.02f, frame);
  glEnable(GL_LIGHTING);
}

void SimWindow::keyboard(unsigned char _key, int _x, int _y) {
  switch (_key) {
    case ' ':  // use space key to play or stop the motion
      mSimulating = !mSimulating;
      if (mSimulating) {
        mPlay = false;
        glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
      }
      break;
    case 'p':  // playBack
      mPlay = !mPlay;
      if (mPlay) {
        mSimulating = false;
        glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
      }
      break;
    case '[':  // step backward
      if (!mSimulating) {
        mPlayFrame--;
        if (mPlayFrame < 0)
          mPlayFrame = 0;
        glutPostRedisplay();
      }
      break;
    case ']':  // step forwardward
      if (!mSimulating) {
        mPlayFrame++;
        if (mPlayFrame >= mWorld->getRecording()->getNumFrames())
          mPlayFrame = 0;
        glutPostRedisplay();
      }
      break;
    case 'v':  // show or hide markers
      mShowMarkers = !mShowMarkers;
      break;
  case 's':
      saveWorld();
      std::cout << "World Saved in 'tempWorld.txt'" << std::endl;
      break;
    default:
      Win3D::keyboard(_key, _x, _y);
  }
  glutPostRedisplay();
}

void SimWindow::setWorld(simulation::WorldPtr _world) {
  mWorld = _world;
}

void SimWindow::saveWorld() {
  if (!mWorld)
    return;
  dart::utils::FileInfoWorld worldFile;
  worldFile.saveFile("tempWorld.txt", mWorld->getRecording());
}

void SimWindow::plot(Eigen::VectorXd& _data) {
  GraphWindow* figure = new GraphWindow();
  figure->setData(_data);
  figure->initWindow(480, 240, "figure");
  mGraphWindows.push_back(figure);
}

//==============================================================================
void SimWindow::drawSkeleton(const dynamics::Skeleton* skeleton,
                             const Eigen::Vector4d& color,
                             bool useDefaultColor) const
{
  if (!skeleton)
    return;

  for (auto i = 0u; i < skeleton->getNumTrees(); ++i)
    drawBodyNode(skeleton->getRootBodyNode(i), color, useDefaultColor, true);
}

//==============================================================================
void SimWindow::drawEntity(const dynamics::Entity* entity,
                           const Eigen::Vector4d& color,
                           bool useDefaultColor) const
{
  if (!entity)
    return;

  const auto& bodyNode = dynamic_cast<const dynamics::BodyNode*>(entity);
  if (bodyNode)
  {
    drawBodyNode(bodyNode, color, useDefaultColor, true);
    return;
  }

  const auto& shapeFrame = dynamic_cast<const dynamics::ShapeFrame*>(entity);
  if (shapeFrame)
  {
    drawShapeFrame(shapeFrame, color, useDefaultColor);
    return;
  }
}

//==============================================================================
void SimWindow::drawBodyNode(const dynamics::BodyNode* bodyNode,
                             const Eigen::Vector4d& color,
                             bool useDefaultColor,
                             bool recursive) const
{
  if (!bodyNode)
    return;

  if (!mRI)
    return;

  mRI->pushMatrix();

  // Use the relative transform of this Frame. We assume that we are being
  // called from the parent Frame's renderer.
  // TODO(MXG): This can cause trouble if the draw function is originally called
  // on an Entity or Frame which is not a child of the World Frame
  mRI->transform(bodyNode->getRelativeTransform());

  // _ri->pushName(???); TODO(MXG): What should we do about this for Frames?
  auto shapeNodes = bodyNode->getShapeNodesWith<dynamics::VisualAddon>();
  for (const auto& shapeNode : shapeNodes)
    drawShapeFrame(shapeNode, color, useDefaultColor);
  // _ri.popName();

  if (mShowPointMasses)
  {
    const auto& softBodyNode
        = dynamic_cast<const dynamics::SoftBodyNode*>(bodyNode);
    if (softBodyNode)
      drawPointMasses(softBodyNode->getPointMasses(), color);
  }

  if (mShowMarkers)
  {
    for (auto i = 0u; i < bodyNode->getNumMarkers(); ++i)
      drawMarker(bodyNode->getMarker(i));
  }

  // render the subtree
  if (recursive)
  {
    for (const auto& entity : bodyNode->getChildEntities())
      drawEntity(entity, color, useDefaultColor);
  }

  mRI->popMatrix();
}

//==============================================================================
void SimWindow::drawShapeFrame(const dynamics::ShapeFrame* shapeFrame,
                               const Eigen::Vector4d& color,
                               bool useDefaultColor) const
{
  if (!shapeFrame)
    return;

  if (!mRI)
    return;

  const auto& visualAddon = shapeFrame->getVisualAddon();

  if (!visualAddon || visualAddon->isHidden())
    return;

  mRI->pushMatrix();
  mRI->transform(shapeFrame->getRelativeTransform());

  if (useDefaultColor)
    drawShape(shapeFrame->getShape().get(), visualAddon->getRGBA());
  else
    drawShape(shapeFrame->getShape().get(), color);

  mRI->popMatrix();
}

//==============================================================================
void SimWindow::drawShape(const dynamics::Shape* shape,
                          const Eigen::Vector4d& color) const
{
  if (!shape)
    return;

  if (!mRI)
    return;

  glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
  glEnable(GL_COLOR_MATERIAL);

  mRI->setPenColor(color);

  using dynamics::Shape;
  using dynamics::BoxShape;
  using dynamics::EllipsoidShape;
  using dynamics::CylinderShape;
  using dynamics::PlaneShape;
  using dynamics::MeshShape;
  using dynamics::SoftMeshShape;
  using dynamics::LineSegmentShape;

  switch (shape->getShapeType())
  {
    case Shape::BOX:
    {
      const auto& box = static_cast<const BoxShape*>(shape);
      mRI->drawCube(box->getSize());

      break;
    }
    case Shape::ELLIPSOID:
    {
      const auto& ellipsoid = static_cast<const EllipsoidShape*>(shape);
      mRI->drawEllipsoid(ellipsoid->getSize());

      break;
    }
    case Shape::CYLINDER:
    {
      const auto& cylinder = static_cast<const CylinderShape*>(shape);
      mRI->drawCylinder(cylinder->getRadius(), cylinder->getHeight());

      break;
    }
    case Shape::MESH:
    {
      const auto& mesh = static_cast<const MeshShape*>(shape);

      glDisable(GL_COLOR_MATERIAL); // Use mesh colors to draw

      if (mesh->getDisplayList())
        mRI->drawList(mesh->getDisplayList());
      else
        mRI->drawMesh(mesh->getScale(), mesh->getMesh());

      break;
    }
    case Shape::SOFT_MESH:
    {
      const auto& softMesh = static_cast<const SoftMeshShape*>(shape);
      mRI->drawSoftMesh(softMesh->getAssimpMesh());

      break;
    }
    case Shape::LINE_SEGMENT:
    {
      const auto& lineSegmentShape
          = static_cast<const LineSegmentShape*>(shape);
      mRI->drawLineSegments(lineSegmentShape->getVertices(),
                            lineSegmentShape->getConnections());

      break;
    }
    default:
    {
      dterr << "[SimWindow::drawShape] Attempting to draw unsupported shape "
            << "type '" << shape->getShapeType() << "'.\n";
      break;
    }
  }

  glDisable(GL_COLOR_MATERIAL);
}

//==============================================================================
void SimWindow::drawPointMasses(
    const std::vector<dynamics::PointMass*> pointMasses,
    const Eigen::Vector4d& color,
    bool useDefaultColor) const
{
  if (!mRI)
    return;

  for (const auto& pointMass : pointMasses)
  {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    // render point at the current position
    mRI->pushMatrix();
    T.translation() = pointMass->getLocalPosition();
    mRI->transform(T);
    if (useDefaultColor)
      mRI->setPenColor(Eigen::Vector4d(0.8, 0.3, 0.3, 1.0));
    else
      mRI->setPenColor(color);
    mRI->drawEllipsoid(Eigen::Vector3d::Constant(0.01));
    mRI->popMatrix();

    // render point at the resting position
    mRI->pushMatrix();
    T.translation() = pointMass->getRestingPosition();
    mRI->transform(T);
    if (useDefaultColor)
      mRI->setPenColor(Eigen::Vector4d(0.8, 0.3, 0.3, 1.0));
    else
      mRI->setPenColor(color);
    mRI->drawEllipsoid(Eigen::Vector3d::Constant(0.01));
    mRI->popMatrix();
  }
}

//==============================================================================
void SimWindow::drawMarker(const dynamics::Marker* marker,
                           const Eigen::Vector4d& color,
                           bool useDefaultColor) const
{
  if (!marker)
    return;

  if (!mRI)
    return;

  mRI->pushName(marker->getID());

  if (marker->getConstraintType() == dynamics::Marker::HARD)
  {
    mRI->setPenColor(Color::Red(1.0));
  }
  else if (marker->getConstraintType() == dynamics::Marker::SOFT)
  {
    mRI->setPenColor(Color::Green(1.0));
  }
  else
  {
    if (useDefaultColor)
      mRI->setPenColor(marker->getColor());
    else
      mRI->setPenColor(color);
  }

  mRI->pushMatrix();
  mRI->translate(marker->getLocalPosition());
  mRI->drawEllipsoid(Eigen::Vector3d::Constant(0.01));
  mRI->popMatrix();

  mRI->popName();
}

}  // namespace gui
}  // namespace dart
