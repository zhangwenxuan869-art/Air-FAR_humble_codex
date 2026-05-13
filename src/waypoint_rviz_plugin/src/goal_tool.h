#ifndef WAYPOINT_RVIZ_PLUGIN__GOAL_TOOL_H_
#define WAYPOINT_RVIZ_PLUGIN__GOAL_TOOL_H_

#include <QObject>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "pose_tool.h"

namespace waypoint_rviz_plugin
{

class Goal3DTool : public Pose3DTool
{
  Q_OBJECT

public:
  Goal3DTool();
  ~Goal3DTool() override = default;

  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double z, double theta) override;

private:
  void updateTopic();
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  double init_z_offset_;
};

}  // namespace waypoint_rviz_plugin

#endif  // WAYPOINT_RVIZ_PLUGIN__GOAL_TOOL_H_
