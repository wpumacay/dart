/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
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

#include "osgDart/DragAndDrop.h"
#include "osgDart/DefaultEventHandler.h"
#include "osgDart/Viewer.h"
#include "osgDart/InteractiveFrame.h"
#include "osgDart/MouseEventHandler.h"

#include "dart/dynamics/SimpleFrame.h"
#include "dart/dynamics/MeshShape.h"

#include <iostream>

namespace osgDart {

DragAndDrop::DragAndDrop(Viewer* viewer, dart::dynamics::Entity* entity)
  : mViewer(viewer),
    mEntity(entity),
    mPickedPosition(Eigen::Vector3d::Zero()),
    mConstraintType(UNCONSTRAINED),
    mAmObstructable(true),
    mAmMoving(false),
    mOption(RotationOption::HOLD_CTRL)
{
  addSubject(mEntity);
  addSubject(mViewer);
}

//==============================================================================
DragAndDrop::~DragAndDrop()
{
  // Do nothing
}

//==============================================================================
dart::dynamics::Entity* DragAndDrop::getEntity() const
{
  return mEntity;
}

//==============================================================================
void DragAndDrop::update()
{
  if(nullptr == mEntity)
    return;

  osgDart::MouseButtonEvent event =
      mViewer->getDefaultEventHandler()->getButtonEvent(LEFT_MOUSE);

  if(mAmMoving)
  {
    if(osgDart::BUTTON_RELEASE == event)
      mAmMoving = false;

    move();
  }
  else // not moving
  {
    if(osgDart::BUTTON_PUSH == event)
    {
      const std::vector<osgDart::PickInfo>& picks =
          mViewer->getDefaultEventHandler()->getButtonPicks(
            osgDart::LEFT_MOUSE, osgDart::BUTTON_PUSH);

      for(const osgDart::PickInfo& pick : picks)
      {
        if(pick.entity == mEntity)
        {
          mAmMoving = true;
          mPickedPosition = pick.position;
          saveState();
          return;
        }

        // The picks are always ordered from closest to furthest. If the closest
        // pick is not our Entity, then something is blocking the way, so if we
        // are obstructable, then we should quit.
        if(mAmObstructable)
          return;
      }
    }
  }
}

//==============================================================================
void DragAndDrop::setObstructable(bool _obstructable)
{
  mAmObstructable = _obstructable;
}

//==============================================================================
bool DragAndDrop::isObstructable() const
{
  return mAmObstructable;
}

//==============================================================================
Eigen::Vector3d DragAndDrop::getConstrainedDx() const
{
  return mViewer->getDefaultEventHandler()->getDeltaCursor(
        mPickedPosition, mConstraintType, mVector);
}

//==============================================================================
Eigen::AngleAxisd DragAndDrop::getConstrainedRotation() const
{
  Eigen::Vector3d v1 = mPickedPosition - mPivot;
  Eigen::Vector3d v2;

  if(LINE_CONSTRAINT == mConstraintType || PLANE_CONSTRAINT == mConstraintType)
  {
    v2 = mViewer->getDefaultEventHandler()->getDeltaCursor(
          mPickedPosition, PLANE_CONSTRAINT, mVector)
         + mPickedPosition - mPivot;
  }
  else
  {
    v2 = mViewer->getDefaultEventHandler()->getDeltaCursor(mPickedPosition)
         + mPickedPosition - mPivot;
  }

  if(v1.norm() == 0 || v2.norm() == 0 || v1.cross(v2).norm() == 0)
    return Eigen::AngleAxisd(0, Eigen::Vector3d(1,0,0));

  v1.normalize();
  v2.normalize();

  Eigen::Vector3d axis = v1.cross(v2);
  if(LINE_CONSTRAINT == mConstraintType || PLANE_CONSTRAINT == mConstraintType)
  {
    if(axis.dot(mVector) == 0)
      return Eigen::AngleAxisd(0, Eigen::Vector3d(1,0,0));

    axis = axis.dot(mVector)*mVector;
  }

  axis.normalize();

  return Eigen::AngleAxisd(acos(v1.dot(v2)), axis);
}

//==============================================================================
void DragAndDrop::unconstrain()
{
  mConstraintType = UNCONSTRAINED;
}

//==============================================================================
void DragAndDrop::constrainToLine(const Eigen::Vector3d& slope)
{
  mConstraintType = LINE_CONSTRAINT;
  mVector = slope;
}

//==============================================================================
void DragAndDrop::constrainToPlane(const Eigen::Vector3d& normal)
{
  mConstraintType = PLANE_CONSTRAINT;
  mVector = normal;
}

//==============================================================================
bool DragAndDrop::isMoving() const
{
  return mAmMoving;
}

//==============================================================================
void DragAndDrop::setRotationOption(RotationOption option)
{
  mOption = option;
}

//==============================================================================
void DragAndDrop::handleDestructionNotification(
    const dart::common::Subject* subscription)
{
  if(mEntity == subscription)
    mViewer->disableDragAndDrop(this);

  if(mViewer == subscription)
    delete this;
}

//==============================================================================
SimpleFrameDnD::SimpleFrameDnD(Viewer* viewer,
                               dart::dynamics::SimpleFrame* frame)
  : DragAndDrop(viewer, frame),
    mFrame(frame)
{

}

//==============================================================================
SimpleFrameDnD::~SimpleFrameDnD()
{
  // Do nothing
}

//==============================================================================
dart::dynamics::SimpleFrame* SimpleFrameDnD::getSimpleFrame() const
{
  return mFrame;
}

//==============================================================================
void SimpleFrameDnD::move()
{
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());

  bool ctrl_down = (mViewer->getDefaultEventHandler()->getModKeyMask()
                    & osgGA::GUIEventAdapter::MODKEY_CTRL);

  if(  ((RotationOption::HOLD_CTRL==mOption) && ctrl_down)
      || RotationOption::ALWAYS_ON==mOption )
  {
    // Rotate

    Eigen::AngleAxisd R = getConstrainedRotation();

    tf.translation() = mPivot;
    tf.linear() = (R * mSavedRotation).matrix();
  }
  else
  {
    // Translate

    Eigen::Vector3d dx = getConstrainedDx();

    tf.translation() = mPivot + dx;
    tf.rotate(mSavedRotation);
  }

  dart::dynamics::Frame* parent = mFrame->getParentFrame();
  if(parent->isWorld())
    mFrame->setRelativeTransform(tf);
  else
    mFrame->setRelativeTransform( parent->getWorldTransform().inverse()*tf );
}

//==============================================================================
void SimpleFrameDnD::saveState()
{
  mPivot = mFrame->getWorldTransform().translation();
  mSavedRotation = mFrame->getWorldTransform().rotation();
}

//==============================================================================
SimpleFrameShapeDnD::SimpleFrameShapeDnD(
    Viewer* viewer,
    dart::dynamics::SimpleFrame* frame,
    dart::dynamics::Shape* shape)
  : SimpleFrameDnD(viewer, frame),
    mShape(shape)
{
  // Do nothing
}

//==============================================================================
SimpleFrameShapeDnD::~SimpleFrameShapeDnD()
{
  // Do nothing
}

//==============================================================================
dart::dynamics::Shape* SimpleFrameShapeDnD::getShape() const
{
  return mShape;
}

//==============================================================================
void SimpleFrameShapeDnD::update()
{
  // This is almost identical to the original DragAndDrop::update() except that
  // it also checks that the picked shape matches

  if(nullptr == mEntity || nullptr == mShape)
    return;

  osgDart::MouseButtonEvent event =
      mViewer->getDefaultEventHandler()->getButtonEvent(LEFT_MOUSE);

  if(mAmMoving)
  {
    if(osgDart::BUTTON_RELEASE == event)
      mAmMoving = false;

    move();
  }
  else
  {
    if(osgDart::BUTTON_PUSH == event)
    {
      const std::vector<osgDart::PickInfo>& picks =
          mViewer->getDefaultEventHandler()->getButtonPicks(
            osgDart::LEFT_MOUSE, osgDart::BUTTON_PUSH);

      for(const osgDart::PickInfo& pick : picks)
      {
        if(pick.entity == mEntity && pick.shape == mShape)
        {
          mAmMoving = true;
          mPickedPosition = pick.position;
          saveState();
          return;
        }

        if(mAmObstructable)
          return;
      }
    }
  }
}

//==============================================================================
void SimpleFrameShapeDnD::handleDestructionNotification(
    const dart::common::Subject* subscription)
{
  DragAndDrop::handleDestructionNotification(subscription);

  if(mShape == subscription)
    mViewer->disableDragAndDrop(this);
}

//==============================================================================
class InteractiveFrameMouseEvent : public MouseEventHandler
{
public:

  InteractiveFrameMouseEvent(InteractiveFrame* frame) :
    mFrame(frame), mHighlighting(false)
  {
    addSubject(mFrame);
  }

  void update() override
  {
    if(!mFrame)
      return;

    if(!mEventHandler)
      return;

    if(mHighlighting)
    {
      MouseButtonEvent event = mEventHandler->getButtonEvent(LEFT_MOUSE);
      bool stop_highlighting = false;

      if(BUTTON_RELEASE == event || BUTTON_NOTHING == event)
      {
        const std::vector<PickInfo>& picks = mEventHandler->getMovePicks();
        if(picks.size() > 0)
        {
          const PickInfo& pick = picks[0];
          if(pick.entity != mFrame->getTool(
               (InteractiveTool::Type)mTool, mCoordinate))
            stop_highlighting = true;
        }
        else
          stop_highlighting = true;
      }

      if(stop_highlighting)
      {
        for(size_t s=0; s < InteractiveTool::NUM_TYPES; ++s)
          for(size_t c=0; c<3; ++c)
            mFrame->getTool((InteractiveTool::Type)s, c)->resetAlpha();
        mHighlighting = false;
      }
    }
    else
    {
      MouseButtonEvent event = mEventHandler->getButtonEvent(LEFT_MOUSE);

      if(BUTTON_NOTHING != event && BUTTON_RELEASE != event)
        return;

      const std::vector<PickInfo> picks = mEventHandler->getMovePicks();
      if(picks.size() == 0)
        return;

      const PickInfo& pick = picks[0];

      for(size_t s=0; s < (size_t)InteractiveTool::NUM_TYPES; ++s)
      {
        for(size_t c=0; c<3; ++c)
        {
          if(mFrame->getTool((InteractiveTool::Type)s, c) == pick.entity)
          {
            mHighlighting = true;
            mTool = s;
            mCoordinate = c;
            break;
          }
        }
        if(mHighlighting)
          break;
      }

      if(mHighlighting)
      {
        for(size_t s=0; s < InteractiveTool::NUM_TYPES; ++s)
        {
          for(size_t c=0; c<3; ++c)
          {
            if(s == (size_t)mTool && c == mCoordinate)
              mFrame->getTool((InteractiveTool::Type)s, c)->setAlpha(1.0);
            else
              mFrame->getTool((InteractiveTool::Type)s, c)->setAlpha(0.3);
          }
        }
      }
    }
  }

protected:

  void handleDestructionNotification(const Subject* _subject) override
  {
    if(_subject == mFrame)
    {
      delete this;
      return;
    }

    MouseEventHandler::handleDestructionNotification(_subject);
  }

  InteractiveFrame* mFrame;

  bool mHighlighting;
  int mTool;
  size_t mCoordinate;
};

//==============================================================================
class InteractiveToolDnD : public SimpleFrameDnD
{
public:

  InteractiveToolDnD(Viewer* viewer, InteractiveFrame* frame,
                     InteractiveTool* tool)
    : SimpleFrameDnD(viewer, frame)
  {
    addSubject(tool);
    mEntity = tool;
  }

protected:

  dart::sub_ptr<InteractiveTool> mTool;
};

//==============================================================================
InteractiveFrameDnD::InteractiveFrameDnD(Viewer* viewer,
                                         InteractiveFrame* frame)
  : DragAndDrop(viewer, frame),
    mInteractiveFrame(frame)
{
  mViewer->getDefaultEventHandler()->addMouseEventHandler(
        new InteractiveFrameMouseEvent(frame));

  for(size_t i=0; i<InteractiveTool::NUM_TYPES; ++i)
    for(size_t j=0; j<3; ++j)
      mDnDs.push_back(new InteractiveToolDnD(viewer, frame,
                            frame->getTool((InteractiveTool::Type)i, j)));

  for(size_t i=0; i<3; ++i)
  {
    DragAndDrop* dnd = mDnDs[i];
    dnd->setRotationOption(SimpleFrameDnD::RotationOption::ALWAYS_OFF);

    dnd = mDnDs[i+3];
    dnd->setRotationOption(SimpleFrameDnD::RotationOption::ALWAYS_ON);

    dnd = mDnDs[i+6];
    dnd->setRotationOption(SimpleFrameDnD::RotationOption::ALWAYS_OFF);
  }
}

//==============================================================================
InteractiveFrameDnD::~InteractiveFrameDnD()
{
  for(size_t i=0; i<mDnDs.size(); ++i)
    delete mDnDs[i];
  mDnDs.clear();
}

//==============================================================================
InteractiveFrame* InteractiveFrameDnD::getFrame() const
{
  return mInteractiveFrame;
}

//==============================================================================
void InteractiveFrameDnD::update()
{
  if(!mAmMoving)
  {
    for(size_t i=0; i<3; ++i)
    {
      DragAndDrop* dnd = mDnDs[i];
      Eigen::Matrix3d R = mInteractiveFrame->getWorldTransform().linear();
      dnd->constrainToLine(R.col(i));

      dnd = mDnDs[i+3];
      dnd->constrainToLine(R.col(i));

      dnd = mDnDs[i+6];
      dnd->constrainToPlane(R.col(i));
    }
  }

  mAmMoving = false;
  for(size_t i=0; i<mDnDs.size(); ++i)
  {
    DragAndDrop* dnd = mDnDs[i];
    dnd->update();
    mAmMoving |= dnd->isMoving();
  }

  if(mAmMoving)
  {
    for(size_t i=0; i<InteractiveTool::NUM_TYPES; ++i)
    {
      for(size_t j=0; j<3; ++j)
      {
        DragAndDrop* dnd = mDnDs[3*i+j];
        InteractiveTool* tool =
            mInteractiveFrame->getTool((InteractiveTool::Type)i,j);
        if(!dnd->isMoving() && tool->getEnabled())
        {
          const std::vector<dart::dynamics::Shape*> shapes =
              tool->getVisualizationShapes();
          for(size_t s=0; s<shapes.size(); ++s)
            shapes[s]->setHidden(true);
        }
      }
    }
  }
  else
  {
    for(size_t i=0; i<InteractiveTool::NUM_TYPES; ++i)
    {
      for(size_t j=0; j<3; ++j)
      {
        InteractiveTool* tool =
            mInteractiveFrame->getTool((InteractiveTool::Type)i,j);
        if(tool->getEnabled())
        {
          const std::vector<dart::dynamics::Shape*> shapes =
              tool->getVisualizationShapes();
          for(size_t s=0; s<shapes.size(); ++s)
            shapes[s]->setHidden(false);
        }
      }
    }
  }
}

//==============================================================================
void InteractiveFrameDnD::move()
{
  // Do nothing
}

//==============================================================================
void InteractiveFrameDnD::saveState()
{
  // Do nothing
}

} // namespace osgDart