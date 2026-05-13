#ifndef WAYPOINT_RVIZ_PLUGIN__POSE_TOOL_H_
#define WAYPOINT_RVIZ_PLUGIN__POSE_TOOL_H_

#include <memory>
#include <utility>
#include <vector>

#include <OgreVector3.h>

#include "rviz_common/tool.hpp"

namespace Ogre
{
class Quaternion;
}

namespace rviz_rendering
{
class Arrow;
class ViewportProjectionFinder;
}

namespace waypoint_rviz_plugin
{

class Pose3DTool : public rviz_common::Tool
{
public:
  Pose3DTool();
  ~Pose3DTool() override;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

protected:
  enum State
  {
    Position,
    Orientation,
    Height
  };

  virtual void onPoseSet(double x, double y, double z, double theta) = 0;

  std::shared_ptr<rviz_rendering::Arrow> arrow_;
  std::vector<std::shared_ptr<rviz_rendering::Arrow>> arrow_array_;
  std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;
  State state_;
  Ogre::Vector3 pos_;
  double prev_angle_;
  double init_z_;
};

}  // namespace waypoint_rviz_plugin

#endif  // WAYPOINT_RVIZ_PLUGIN__POSE_TOOL_H_
