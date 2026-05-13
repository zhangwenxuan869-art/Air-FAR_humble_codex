#include "pose_tool.h"

#include <cmath>

#include <OgrePlane.h>
#include <OgreQuaternion.h>

#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/viewport_projection_finder.hpp"

namespace waypoint_rviz_plugin
{

Pose3DTool::Pose3DTool()
: rviz_common::Tool(),
  state_(Position),
  prev_angle_(0.0),
  init_z_(0.0)
{}

Pose3DTool::~Pose3DTool()
{
  arrow_array_.clear();
  arrow_.reset();
}

void Pose3DTool::onInitialize()
{
  projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
  arrow_ = std::make_shared<rviz_rendering::Arrow>(
    scene_manager_, nullptr, 2.0f, 0.2f, 0.5f, 0.35f);
  arrow_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
  arrow_->setScale(Ogre::Vector3::ZERO);
}

void Pose3DTool::activate()
{
  setStatus("Click and drag mouse to set position, orientation, and height.");
  state_ = Position;
}

void Pose3DTool::deactivate()
{
  if (arrow_) {
    arrow_->setScale(Ogre::Vector3::ZERO);
  }
  arrow_array_.clear();
  state_ = Position;
}

int Pose3DTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (!projection_finder_) {
    return 0;
  }

  const auto xy_plane_intersection =
    projection_finder_->getViewportPointProjectionOnXYPlane(
    event.panel->getRenderWindow(), event.x, event.y);

  int flags = 0;
  const double z_scale = 240.0;
  const double z_interval = 0.5;
  const Ogre::Quaternion orient_x(
    Ogre::Radian(Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Z);

  if (event.leftDown()) {
    if (xy_plane_intersection.first) {
      pos_ = xy_plane_intersection.second;
      pos_.z = 0.0;
      arrow_->setPosition(pos_);
      state_ = Orientation;
      flags |= Render;
    }
  } else if (event.left()) {
    if (state_ == Orientation && xy_plane_intersection.first) {
      const Ogre::Vector3 cur_pos = xy_plane_intersection.second;
      prev_angle_ = std::atan2(cur_pos.y - pos_.y, cur_pos.x - pos_.x);
      arrow_->setScale(Ogre::Vector3::UNIT_SCALE);
      arrow_->setOrientation(orient_x);
      if (event.wheel_delta != 0) {
        state_ = Height;
        pos_.z = 0.5;
        init_z_ = 0.0;
      }
      flags |= Render;
    }

    if (state_ == Height) {
      pos_.z += static_cast<double>(event.wheel_delta) / z_scale;
      arrow_->setPosition(pos_);

      arrow_array_.clear();
      const int count = static_cast<int>(std::ceil(std::fabs(init_z_ - pos_.z) / z_interval));
      for (int k = 0; k < count; ++k) {
        auto preview_arrow = std::make_shared<rviz_rendering::Arrow>(
          scene_manager_, nullptr, 0.5f, 0.1f, 0.0f, 0.1f);
        preview_arrow->setColor(0.0f, 1.0f, 0.0f, 1.0f);
        Ogre::Vector3 arrow_pos = pos_;
        arrow_pos.z = init_z_ - ((init_z_ - pos_.z > 0.0) ? 1.0 : -1.0) * k * z_interval;
        preview_arrow->setPosition(arrow_pos);
        preview_arrow->setOrientation(
          Ogre::Quaternion(Ogre::Radian(prev_angle_), Ogre::Vector3::UNIT_Z) * orient_x);
        arrow_array_.push_back(preview_arrow);
      }
      flags |= Render;
    }
  } else if (event.leftUp()) {
    if (state_ == Orientation || state_ == Height) {
      arrow_array_.clear();
      onPoseSet(pos_.x, pos_.y, pos_.z, prev_angle_);
      state_ = Position;
      flags |= (Finished | Render);
    }
  }

  return flags;
}

}  // namespace waypoint_rviz_plugin
