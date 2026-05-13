#include "goal_tool.h"

#include <pluginlib/class_list_macros.hpp>

#include <chrono>
#include <thread>

#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace waypoint_rviz_plugin
{

Goal3DTool::Goal3DTool()
: init_z_offset_(0.0)
{
  shortcut_key_ = 'g';
}

void Goal3DTool::onInitialize()
{
  Pose3DTool::onInitialize();
  setName("Waypoint3D");
  auto node_abstraction = context_->getRosNodeAbstraction().lock();
  if (node_abstraction) {
    raw_node_ = node_abstraction->get_raw_node();
  }
  updateTopic();
}

void Goal3DTool::updateTopic()
{
  if (!raw_node_) {
    return;
  }

  waypoint_publisher_ =
    raw_node_->create_publisher<geometry_msgs::msg::PointStamped>("/way_point", 5);
  joy_publisher_ = raw_node_->create_publisher<sensor_msgs::msg::Joy>("/joy", 5);
  odom_subscription_ = raw_node_->create_subscription<nav_msgs::msg::Odometry>(
    "/state_estimation", 5,
    std::bind(&Goal3DTool::odomCallback, this, std::placeholders::_1));
}

void Goal3DTool::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  init_z_offset_ = msg->pose.pose.position.z;
}

void Goal3DTool::onPoseSet(double x, double y, double z, double theta)
{
  (void)theta;
  if (!raw_node_ || !waypoint_publisher_ || !joy_publisher_) {
    return;
  }

  sensor_msgs::msg::Joy joy;
  joy.axes = {0.0F, 0.0F, -1.0F, 0.0F, 1.0F, 1.0F, 0.0F, 0.0F};
  joy.buttons = {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0};
  joy.header.stamp = raw_node_->get_clock()->now();
  joy.header.frame_id = "waypoint_tool";
  joy_publisher_->publish(joy);

  geometry_msgs::msg::PointStamped waypoint;
  waypoint.header.frame_id = "map";
  waypoint.header.stamp = joy.header.stamp;
  waypoint.point.x = x;
  waypoint.point.y = y;
  waypoint.point.z = z + init_z_offset_;

  waypoint_publisher_->publish(waypoint);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  waypoint_publisher_->publish(waypoint);
}

}  // namespace waypoint_rviz_plugin

PLUGINLIB_EXPORT_CLASS(waypoint_rviz_plugin::Goal3DTool, rviz_common::Tool)
