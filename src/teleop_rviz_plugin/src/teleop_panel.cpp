#include "teleop_panel.h"

#include <pluginlib/class_list_macros.hpp>

#include "drive_widget.h"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace teleop_rviz_plugin
{

TeleopPanel::TeleopPanel(QWidget * parent)
: rviz_common::Panel(parent),
  drive_widget_(new DriveWidget(this)),
  resume_button_(new QPushButton("Resume Navigation to Goal", this)),
  output_timer_(new QTimer(this)),
  linear_velocity_(0.0F),
  angular_velocity_(0.0F),
  z_velocity_(0.0F),
  mouse_pressed_(false),
  mouse_pressed_sent_(false)
{
  auto * layout = new QVBoxLayout;
  layout->addWidget(resume_button_);
  layout->addWidget(drive_widget_);
  setLayout(layout);

  connect(resume_button_, &QPushButton::pressed, this, &TeleopPanel::pressButtonResume);
  connect(
    drive_widget_, &DriveWidget::outputVelocity, this,
    &TeleopPanel::setVel);
  connect(output_timer_, &QTimer::timeout, this, &TeleopPanel::sendVel);
  output_timer_->start(100);
}

void TeleopPanel::onInitialize()
{
  auto node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!node_abstraction) {
    return;
  }
  raw_node_ = node_abstraction->get_raw_node();
  ensurePublishers();
  drive_widget_->setEnabled(static_cast<bool>(raw_node_));
}

void TeleopPanel::ensurePublishers()
{
  if (!raw_node_ || velocity_publisher_) {
    return;
  }

  velocity_publisher_ = raw_node_->create_publisher<sensor_msgs::msg::Joy>("/joy", 5);
  attemptable_publisher_ =
    raw_node_->create_publisher<std_msgs::msg::Bool>("/planning_attemptable", 5);
  update_publisher_ =
    raw_node_->create_publisher<std_msgs::msg::Bool>("/update_visibility_graph", 5);
  reset_publisher_ =
    raw_node_->create_publisher<std_msgs::msg::Empty>("/reset_visibility_graph", 5);
  read_publisher_ =
    raw_node_->create_publisher<std_msgs::msg::String>("/read_file_dir", 5);
  save_publisher_ =
    raw_node_->create_publisher<std_msgs::msg::String>("/save_file_dir", 5);
}

void TeleopPanel::pressButtonResume()
{
  ensurePublishers();
  if (!velocity_publisher_) {
    return;
  }

  sensor_msgs::msg::Joy joy;
  joy.axes = {0.0F, 0.0F, -1.0F, 0.0F, 1.0F, 1.0F, 0.0F, 0.0F};
  joy.buttons = {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0};
  joy.header.stamp = raw_node_->get_clock()->now();
  joy.header.frame_id = "teleop_panel";
  velocity_publisher_->publish(joy);
}

void TeleopPanel::setVel(
  float linear_velocity, float angular_velocity, bool mouse_pressed, float z_velocity)
{
  linear_velocity_ = linear_velocity;
  angular_velocity_ = angular_velocity;
  mouse_pressed_ = mouse_pressed;
  z_velocity_ = z_velocity;
}

void TeleopPanel::sendVel()
{
  ensurePublishers();
  if (!velocity_publisher_ || (!mouse_pressed_ && !mouse_pressed_sent_)) {
    return;
  }

  sensor_msgs::msg::Joy joy;
  joy.axes = {0.0F, z_velocity_, 1.0F, angular_velocity_, linear_velocity_, 1.0F, 0.0F, 0.0F};
  joy.buttons = {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0};

  const float denom = linear_velocity_ + 0.0001F;
  const float ratio = angular_velocity_ / denom;
  if (ratio > 1.7F || ratio < -1.7F) {
    joy.axes[0] = angular_velocity_;
    joy.axes[3] = 0.0F;
    joy.axes[4] = 0.0F;
    joy.axes[5] = -1.0F;
  }

  joy.header.stamp = raw_node_->get_clock()->now();
  joy.header.frame_id = "teleop_panel";
  velocity_publisher_->publish(joy);
  mouse_pressed_sent_ = mouse_pressed_;
}

void TeleopPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

void TeleopPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

}  // namespace teleop_rviz_plugin

PLUGINLIB_EXPORT_CLASS(teleop_rviz_plugin::TeleopPanel, rviz_common::Panel)
