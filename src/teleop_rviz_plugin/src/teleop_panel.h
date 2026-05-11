#ifndef TELEOP_RVIZ_PLUGIN__TELEOP_PANEL_H_
#define TELEOP_RVIZ_PLUGIN__TELEOP_PANEL_H_

#include <memory>

#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

namespace teleop_rviz_plugin
{

class DriveWidget;

class TeleopPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit TeleopPanel(QWidget * parent = nullptr);

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

public Q_SLOTS:
  void setVel(float linear_velocity, float angular_velocity, bool mouse_pressed, float z_velocity);

private Q_SLOTS:
  void pressButtonResume();
  void sendVel();

private:
  void ensurePublishers();

  DriveWidget * drive_widget_;
  QPushButton * resume_button_;
  QTimer * output_timer_;

  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr attemptable_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr update_publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reset_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr read_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr save_publisher_;

  float linear_velocity_;
  float angular_velocity_;
  float z_velocity_;
  bool mouse_pressed_;
  bool mouse_pressed_sent_;
};

}  // namespace teleop_rviz_plugin

#endif  // TELEOP_RVIZ_PLUGIN__TELEOP_PANEL_H_
