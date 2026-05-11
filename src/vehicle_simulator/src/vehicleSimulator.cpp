#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <gazebo_msgs/msg/entity_state.hpp>
#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

using std::placeholders::_1;

namespace
{

constexpr double kPi = 3.1415926;
constexpr double kControlRate = 200.0;

std::string sim_name = "gazebo";
std::string sensor_name = "lidar";

double realtimeFactor = 1.0;
double windCoeff = 0.05;
double maxRollPitchRate = 20.0;
double rollPitchSmoothRate = 0.1;
double sensorPitch = 0.0;

double vehicleX = 0.0;
double vehicleY = 0.0;
double vehicleZ = 1.0;

double vehicleVelX = 0.0;
double vehicleVelY = 0.0;
double vehicleVelZ = 0.0;
double vehicleVelXG = 0.0;
double vehicleVelYG = 0.0;
double vehicleVelZG = 0.0;

double vehicleRoll = 0.0;
double vehiclePitch = 0.0;
double vehicleYaw = 0.0;

double vehicleRollCmd = 0.0;
double vehiclePitchCmd = 0.0;
double vehicleYawRate = 0.0;

constexpr int systemDelay = 3;
int systemInitCount = 0;
bool systemInited = false;

rclcpp::Node::SharedPtr node;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubScanPointer;

std::vector<int> scanInd;
pcl::PointCloud<pcl::PointXYZI>::Ptr scanData(new pcl::PointCloud<pcl::PointXYZI>());

geometry_msgs::msg::Quaternion makeQuaternion(double roll, double pitch, double yaw)
{
  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);
  return tf2::toMsg(quat);
}

void declareAndGetDouble(const rclcpp::Node::SharedPtr& nh, const std::string& name, double& value)
{
  nh->declare_parameter<double>(name, value);
  nh->get_parameter(name, value);
}

void declareAndGetString(const rclcpp::Node::SharedPtr& nh, const std::string& name, std::string& value)
{
  nh->declare_parameter<std::string>(name, value);
  nh->get_parameter(name, value);
}

void controlHandler(const geometry_msgs::msg::TwistStamped::ConstSharedPtr controlIn)
{
  vehicleRollCmd = controlIn->twist.linear.x;
  vehiclePitchCmd = controlIn->twist.linear.y;
  vehicleYawRate = controlIn->twist.angular.z;
  vehicleVelZG = controlIn->twist.linear.z;
}

void scanHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanIn)
{
  if (sim_name == "unity") {
    return;
  }

  if (!systemInited) {
    systemInitCount++;
    if (systemInitCount > systemDelay) {
      systemInited = true;
    }
    return;
  }

  scanData->clear();
  pcl::fromROSMsg(*scanIn, *scanData);
  pcl::removeNaNFromPointCloud(*scanData, *scanData, scanInd);

  for (auto& point : scanData->points) {
    if (sensor_name == "lidar") {
      const double pointX1 = point.x * std::cos(vehicleYaw) - point.y * std::sin(vehicleYaw);
      const double pointY1 = point.x * std::sin(vehicleYaw) + point.y * std::cos(vehicleYaw);
      const double pointZ1 = point.z;

      point.x = pointX1 + vehicleX;
      point.y = pointY1 + vehicleY;
      point.z = pointZ1 + vehicleZ;
    } else {
      const double pointX1 = point.z;
      const double pointY1 = -point.x;
      const double pointZ1 = -point.y;

      const double pointX2 = pointX1;
      const double pointY2 = pointY1 * std::cos(vehicleRoll) - pointZ1 * std::sin(vehicleRoll);
      const double pointZ2 = pointY1 * std::sin(vehicleRoll) + pointZ1 * std::cos(vehicleRoll);

      const double pointX3 = pointX2 * std::cos(vehiclePitch) + pointZ2 * std::sin(vehiclePitch);
      const double pointY3 = pointY2;
      const double pointZ3 = -pointX2 * std::sin(vehiclePitch) + pointZ2 * std::cos(vehiclePitch);

      const double pointX4 = pointX3 * std::cos(vehicleYaw) - pointY3 * std::sin(vehicleYaw);
      const double pointY4 = pointX3 * std::sin(vehicleYaw) + pointY3 * std::cos(vehicleYaw);
      const double pointZ4 = pointZ3;

      point.x = pointX4 + vehicleX;
      point.y = pointY4 + vehicleY;
      point.z = pointZ4 + vehicleZ;
    }
  }

  sensor_msgs::msg::PointCloud2 scanData2;
  pcl::toROSMsg(*scanData, scanData2);
  scanData2.header.stamp = node->now();
  scanData2.header.frame_id = "map";
  pubScanPointer->publish(scanData2);
}

void sendEntityState(
  const rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr& client,
  const gazebo_msgs::msg::EntityState& state)
{
  if (!client->service_is_ready()) {
    return;
  }

  auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
  request->state = state;
  client->async_send_request(request);
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("vehicleSimulator");

  declareAndGetDouble(node, "realtimeFactor", realtimeFactor);
  declareAndGetDouble(node, "windCoeff", windCoeff);
  declareAndGetDouble(node, "maxRollPitchRate", maxRollPitchRate);
  declareAndGetDouble(node, "rollPitchSmoothRate", rollPitchSmoothRate);
  declareAndGetDouble(node, "sensorPitch", sensorPitch);
  declareAndGetDouble(node, "vehicleX", vehicleX);
  declareAndGetDouble(node, "vehicleY", vehicleY);
  declareAndGetDouble(node, "vehicleZ", vehicleZ);
  declareAndGetDouble(node, "vehicleYaw", vehicleYaw);
  declareAndGetString(node, "sim_name", sim_name);
  declareAndGetString(node, "sensor_name", sensor_name);

  auto subControl = node->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/attitude_control", 5, controlHandler);

  std::string scanTopic = "/velodyne_points";
  if (sensor_name != "lidar") {
    scanTopic = "/rgbd_camera/depth/points";
  }

  auto subScan = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    scanTopic, rclcpp::SensorDataQoS(), scanHandler);

  auto pubVehicleOdom = node->create_publisher<nav_msgs::msg::Odometry>("/state_estimation", 5);

  nav_msgs::msg::Odometry odomData;
  odomData.header.frame_id = "map";
  odomData.child_frame_id = "vehicle";

  auto tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*node);
  geometry_msgs::msg::TransformStamped odomTrans;
  odomTrans.header.frame_id = "map";
  odomTrans.child_frame_id = "vehicle";

  auto setEntityStateClient =
    node->create_client<gazebo_msgs::srv::SetEntityState>("/set_entity_state");

  gazebo_msgs::msg::EntityState cameraState;
  cameraState.name = "rgbd_camera";
  cameraState.reference_frame = "world";
  gazebo_msgs::msg::EntityState lidarState;
  lidarState.name = "lidar";
  lidarState.reference_frame = "world";
  gazebo_msgs::msg::EntityState robotStateGazebo;
  robotStateGazebo.name = "robot";
  robotStateGazebo.reference_frame = "world";

  pubScanPointer = node->create_publisher<sensor_msgs::msg::PointCloud2>("/registered_scan", 5);
  auto pubTerrain = node->create_publisher<sensor_msgs::msg::PointCloud2>("/terrain", 2);
  (void)pubTerrain;

  RCLCPP_INFO(node->get_logger(), "Simulation started.");

  const double rateHz = std::max(1.0, kControlRate * realtimeFactor);
  rclcpp::Rate rate(rateHz);

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    const double vehicleRecRoll = vehicleRoll;
    const double vehicleRecPitch = vehiclePitch;

    if (vehicleRollCmd - vehicleRoll > maxRollPitchRate / kControlRate) {
      vehicleRoll += maxRollPitchRate / kControlRate;
    } else if (vehicleRollCmd - vehicleRoll < -maxRollPitchRate / kControlRate) {
      vehicleRoll -= maxRollPitchRate / kControlRate;
    } else {
      vehicleRoll = vehicleRollCmd;
    }
    vehicleRoll = rollPitchSmoothRate * vehicleRoll + (1.0 - rollPitchSmoothRate) * vehicleRecRoll;

    if (vehiclePitchCmd - vehiclePitch > maxRollPitchRate / kControlRate) {
      vehiclePitch += maxRollPitchRate / kControlRate;
    } else if (vehiclePitchCmd - vehiclePitch < -maxRollPitchRate / kControlRate) {
      vehiclePitch -= maxRollPitchRate / kControlRate;
    } else {
      vehiclePitch = vehiclePitchCmd;
    }
    vehiclePitch = rollPitchSmoothRate * vehiclePitch + (1.0 - rollPitchSmoothRate) * vehicleRecPitch;

    const double vehicleAccX = 9.8 * std::tan(vehiclePitch);
    const double vehicleAccY = -9.8 * std::tan(vehicleRoll) / std::cos(vehiclePitch);

    if (vehicleVelXG < 0.0) {
      vehicleVelXG += windCoeff * vehicleVelXG * vehicleVelXG / kControlRate;
    } else {
      vehicleVelXG -= windCoeff * vehicleVelXG * vehicleVelXG / kControlRate;
    }
    if (vehicleVelYG < 0.0) {
      vehicleVelYG += windCoeff * vehicleVelYG * vehicleVelYG / kControlRate;
    } else {
      vehicleVelYG -= windCoeff * vehicleVelYG * vehicleVelYG / kControlRate;
    }

    vehicleVelXG += (vehicleAccX * std::cos(vehicleYaw) - vehicleAccY * std::sin(vehicleYaw)) / kControlRate;
    vehicleVelYG += (vehicleAccX * std::sin(vehicleYaw) + vehicleAccY * std::cos(vehicleYaw)) / kControlRate;

    const double velX1 = vehicleVelXG * std::cos(vehicleYaw) + vehicleVelYG * std::sin(vehicleYaw);
    const double velY1 = -vehicleVelXG * std::sin(vehicleYaw) + vehicleVelYG * std::cos(vehicleYaw);
    const double velZ1 = vehicleVelZG;

    const double velX2 = velX1 * std::cos(vehiclePitch) - velZ1 * std::sin(vehiclePitch);
    const double velY2 = velY1;
    const double velZ2 = velX1 * std::sin(vehiclePitch) + velZ1 * std::cos(vehiclePitch);

    vehicleVelX = velX2;
    vehicleVelY = velY2 * std::cos(vehicleRoll) + velZ2 * std::sin(vehicleRoll);
    vehicleVelZ = -velY2 * std::sin(vehicleRoll) + velZ2 * std::cos(vehicleRoll);

    vehicleX += vehicleVelXG / kControlRate;
    vehicleY += vehicleVelYG / kControlRate;
    vehicleZ += vehicleVelZG / kControlRate;
    vehicleYaw += vehicleYawRate / kControlRate;

    if (vehicleYaw > kPi) {
      vehicleYaw -= 2.0 * kPi;
    } else if (vehicleYaw < -kPi) {
      vehicleYaw += 2.0 * kPi;
    }

    const rclcpp::Time timeNow = node->now();
    auto geoQuat = makeQuaternion(vehicleRoll, vehiclePitch, vehicleYaw);

    odomData.header.stamp = timeNow;
    odomData.pose.pose.orientation = geoQuat;
    odomData.pose.pose.position.x = vehicleX;
    odomData.pose.pose.position.y = vehicleY;
    odomData.pose.pose.position.z = vehicleZ;
    odomData.twist.twist.angular.x = kControlRate * (vehicleRoll - vehicleRecRoll);
    odomData.twist.twist.angular.y = kControlRate * (vehiclePitch - vehicleRecPitch);
    odomData.twist.twist.angular.z = vehicleYawRate;
    odomData.twist.twist.linear.x = vehicleVelX;
    odomData.twist.twist.linear.y = vehicleVelY;
    odomData.twist.twist.linear.z = vehicleVelZ;
    pubVehicleOdom->publish(odomData);

    odomTrans.header.stamp = timeNow;
    odomTrans.transform.translation.x = vehicleX;
    odomTrans.transform.translation.y = vehicleY;
    odomTrans.transform.translation.z = vehicleZ;
    odomTrans.transform.rotation = geoQuat;
    tfBroadcaster->sendTransform(odomTrans);

    geoQuat = makeQuaternion(vehicleRoll, sensorPitch + vehiclePitch, vehicleYaw);
    const auto geoQuatLidar = makeQuaternion(0.0, 0.0, vehicleYaw);

    cameraState.pose.orientation = geoQuat;
    cameraState.pose.position.x = vehicleX;
    cameraState.pose.position.y = vehicleY;
    cameraState.pose.position.z = vehicleZ;
    sendEntityState(setEntityStateClient, cameraState);

    robotStateGazebo.pose.orientation = geoQuat;
    robotStateGazebo.pose.position.x = vehicleX;
    robotStateGazebo.pose.position.y = vehicleY;
    robotStateGazebo.pose.position.z = vehicleZ;
    sendEntityState(setEntityStateClient, robotStateGazebo);

    lidarState.pose.orientation = geoQuatLidar;
    lidarState.pose.position.x = vehicleX;
    lidarState.pose.position.y = vehicleY;
    lidarState.pose.position.z = vehicleZ;
    sendEntityState(setEntityStateClient, lidarState);

    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
