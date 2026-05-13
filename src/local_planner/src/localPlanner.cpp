#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>


#define PLOTPATHSET 1 // set to 0 to save processing and 1 to plot path set

using namespace std;

rclcpp::Node::SharedPtr nh;

template <typename T>
void declareAndGet(const std::string& name, T& value)
{
  nh->declare_parameter<T>(name, value);
  nh->get_parameter(name, value);
}

rclcpp::Time timeFromSec(double seconds)
{
  return rclcpp::Time(static_cast<int64_t>(seconds * 1e9));
}

geometry_msgs::msg::Quaternion makeQuaternionMsg(double roll, double pitch, double yaw)
{
  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);
  return tf2::toMsg(quat);
}

const double PI = 3.1415926;

string sim_name = "unity";
string sensor_name = "lidar";

// general parameters
string pathFolder;
string stateEstimationTopic = "/state_estimation";
string depthCloudTopic = "/rgbd_camera/depth/points";
double depthCloudDelay = 0;
double depthCamPitchOffset = 0;
double depthCamXOffset = 0;
double depthCamYOffset = 0;
double depthCamZOffset = 0;
bool trackingCamBackward = false;
double trackingCamXOffset = 0;
double trackingCamYOffset = 0;
double trackingCamZOffset = 0;
double trackingCamScale = 1.0;
double scanVoxelSize = 0.1;
const int laserCloudStackNum = 1;
int laserCloudCount = 0;
int pointPerPathThre = 2;
double maxRange = 4.0;
double maxElev = 5.0;
double minObstacleRange = 0.5;
bool keepSurrCloud = true;
double keepHoriDis = 1.0;
double keepVertDis = 0.5;
double lowerBoundZ = -1.2;
double upperBoundZ = 1.2;
double pitchDiffLimit = 35.0;
double pitchWeight = 0.03;
double sensorMaxPitch = 25.0;
double sensorMaxYaw = 40.0;
double yawDiffLimit = 60.0;
double yawWeight = 0.015;
double pathScale = 0.5;
double minPathScale = 0.25;
double pathScaleStep = 0.125;
bool pathScaleBySpeed = true;
double stopDis = 0.5;
bool shiftGoalAtStart = false;
double goalX = 0;
double goalY = 0;
double goalZ = 1.0;
bool enableVerticalEscape = false;
double verticalEscapeMinObstacleHeight = 0.6;
double verticalEscapeLookAhead = 5.0;
double verticalEscapeCorridorWidth = 1.5;
double verticalEscapeClearance = 0.6;
double verticalEscapeMaxStep = 2.0;
double verticalEscapeRadius = 0.8;
double verticalEscapeLateralStep = 1.0;
double verticalEscapeMaxLateralStep = 3.0;
double verticalEscapeRelaxedRadius = 0.35;
double verticalEscapeInfluenceRadius = 2.0;
double verticalEscapeMinCheckZ = -0.3;
int verticalEscapeFailFrames = 8;
double verticalEscapeFallbackStep = 1.0;
int failedPathCount = 0;

// path parameters, set according to path files
const int pathNum = 4375;
const int groupNum = 25;
float gridVoxelSize = 0.2;
float searchRadiusHori = 1.2;
float searchRadiusVert = 0.8;
float gridVoxelOffsetX = 6.4;
float gridVoxelOffsetY = 9.0;
float gridVoxelOffsetZ = 3.3;
const int gridVoxelNumX = 33;
const int gridVoxelNumY = 91;
const int gridVoxelNumZ = 34;
const int gridVoxelNum = gridVoxelNumX * gridVoxelNumY * gridVoxelNumZ;

float joyFwd = 0;
float joyLeft = 0;
float joyUp = 0;
bool manualMode = true;
bool autonomyMode = false;
bool autoAdjustMode = false;
int systemInitDelay = 5;
int stateInitDelay = 100;

pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudDwz(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudStack[laserCloudStackNum];
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudKeep(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudKeepDwz(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr plannerCloudStack(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr plannerCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr startPaths[groupNum];
#if PLOTPATHSET == 1
pcl::PointCloud<pcl::PointXYZI>::Ptr paths[pathNum];
pcl::PointCloud<pcl::PointXYZI>::Ptr freePaths(new pcl::PointCloud<pcl::PointXYZI>());
#endif

int pathList[pathNum] = {0};
float endPitchPathList[pathNum] = {0};
float endYawPathList[pathNum] = {0};
float endZPathList[pathNum] = {0};
int clearPathList[pathNum] = {0};
float clearPathPerGroupScore[groupNum] = {0};
std::vector<int> correspondences[gridVoxelNum];

double laserTime = 0;
bool newlaserCloud = false;

int odomPointerFront = 0;
int odomPointerLast = -1;
const int odomQueLength = 400;
double odomTime[odomQueLength] = {0};
float odomRoll[odomQueLength] = {0};
float odomPitch[odomQueLength] = {0};
float odomYaw[odomQueLength] = {0};
float odomX[odomQueLength] = {0};
float odomY[odomQueLength] = {0};
float odomZ[odomQueLength] = {0};

float trackX = 0;
float trackY = 0;
float trackZ = 0;
float trackRoll = 0;
float trackPitch = 0;
float trackYaw = 0;

pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter;

float pointToSegmentDistanceSq(const pcl::PointXYZ& point,
                               const pcl::PointXYZ& start,
                               const pcl::PointXYZ& end)
{
  const float vx = end.x - start.x;
  const float vy = end.y - start.y;
  const float vz = end.z - start.z;
  const float wx = point.x - start.x;
  const float wy = point.y - start.y;
  const float wz = point.z - start.z;
  const float segLenSq = vx * vx + vy * vy + vz * vz;
  float t = 0.0;
  if (segLenSq > 0.001) {
    t = (wx * vx + wy * vy + wz * vz) / segLenSq;
    if (t < 0.0) t = 0.0;
    else if (t > 1.0) t = 1.0;
  }

  const float dx = start.x + t * vx - point.x;
  const float dy = start.y + t * vy - point.y;
  const float dz = start.z + t * vz - point.z;
  return dx * dx + dy * dy + dz * dz;
}

bool isEscapeSegmentClear(const pcl::PointXYZ& start,
                          const pcl::PointXYZ& end,
                          const float radius)
{
  const float radiusSq = radius * radius;
  for (const auto& point : plannerCloud->points) {
    if (point.z < verticalEscapeMinCheckZ) continue;
    if (point.x * point.x + point.y * point.y + point.z * point.z <
        minObstacleRange * minObstacleRange) {
      continue;
    }
    if (pointToSegmentDistanceSq(point, start, end) < radiusSq) {
      return false;
    }
  }
  return true;
}

float escapeSegmentClearanceSq(const pcl::PointXYZ& start,
                               const pcl::PointXYZ& end)
{
  float clearanceSq = 1000000.0;
  for (const auto& point : plannerCloud->points) {
    if (point.z < verticalEscapeMinCheckZ) continue;
    if (point.x * point.x + point.y * point.y + point.z * point.z <
        minObstacleRange * minObstacleRange) {
      continue;
    }
    const float disSq = pointToSegmentDistanceSq(point, start, end);
    if (disSq < clearanceSq) clearanceSq = disSq;
  }
  return clearanceSq;
}

void setEscapePath(nav_msgs::msg::Path& path,
                   const std::vector<pcl::PointXYZ>& points,
                   const double stamp)
{
  path.poses.resize(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    path.poses[i].pose.position.x = points[i].x;
    path.poses[i].pose.position.y = points[i].y;
    path.poses[i].pose.position.z = points[i].z;
  }
  path.header.stamp = timeFromSec(stamp);
  path.header.frame_id = "track_point";
}

bool tryVerticalEscapePath(nav_msgs::msg::Path& path,
                           const float relativeGoalX,
                           const float relativeGoalY,
                           const float relativeGoalDis,
                           const bool forceEscape,
                           const double stamp)
{
  if (!enableVerticalEscape || relativeGoalDis <= stopDis || plannerCloud->empty()) {
    return false;
  }

  float dirX = 1.0;
  float dirY = 0.0;
  if (relativeGoalDis > 0.001) {
    dirX = relativeGoalX / relativeGoalDis;
    dirY = relativeGoalY / relativeGoalDis;
  }

  bool hasObstacle = false;
  float obstacleTopZ = 0.0;
  float repelX = 0.0;
  float repelY = 0.0;
  for (const auto& point : plannerCloud->points) {
    if (point.z < verticalEscapeMinCheckZ) continue;
    const float disSq = point.x * point.x + point.y * point.y;
    if (disSq < minObstacleRange * minObstacleRange) continue;

    const float forward = point.x * dirX + point.y * dirY;
    const float lateral = fabs(-point.x * dirY + point.y * dirX);
    if (forward > minObstacleRange && forward < verticalEscapeLookAhead &&
        lateral < verticalEscapeCorridorWidth) {
      hasObstacle = true;
      if (point.z > obstacleTopZ) obstacleTopZ = point.z;
    }

    if (forceEscape && disSq < verticalEscapeInfluenceRadius * verticalEscapeInfluenceRadius) {
      hasObstacle = true;
      if (point.z > obstacleTopZ) obstacleTopZ = point.z;
    }

    if (disSq < verticalEscapeInfluenceRadius * verticalEscapeInfluenceRadius) {
      const float weight = 1.0 / disSq;
      repelX -= point.x * weight;
      repelY -= point.y * weight;
    }
  }

  if (!hasObstacle && !forceEscape) {
    return false;
  }

  float targetZ = obstacleTopZ + verticalEscapeClearance;
  if (targetZ < verticalEscapeFallbackStep && forceEscape) {
    targetZ = verticalEscapeFallbackStep;
  }
  if (targetZ < verticalEscapeMinObstacleHeight && !forceEscape) {
    return false;
  }
  if (targetZ > verticalEscapeMaxStep) targetZ = verticalEscapeMaxStep;
  const float maxAllowedZ = maxElev - trackZ;
  if (targetZ > maxAllowedZ) targetZ = maxAllowedZ;
  if (targetZ <= 0.1) return false;

  const pcl::PointXYZ origin(0.0, 0.0, 0.0);
  const pcl::PointXYZ upPoint(0.0, 0.0, targetZ);
  if (isEscapeSegmentClear(origin, upPoint, verticalEscapeRadius)) {
    setEscapePath(path, {origin, upPoint}, stamp);
    RCLCPP_WARN(nh->get_logger(), "localPlanner: vertical escape path selected, climb %.2fm.", targetZ);
    return true;
  }

  float baseX = repelX;
  float baseY = repelY;
  float baseNorm = sqrt(baseX * baseX + baseY * baseY);
  if (baseNorm < 0.001) {
    baseX = -dirX;
    baseY = -dirY;
    baseNorm = 1.0;
  }
  baseX /= baseNorm;
  baseY /= baseNorm;

  float bestScore = -1.0;
  pcl::PointXYZ bestLateralPoint;
  pcl::PointXYZ bestLateralUpPoint;
  const float angles[] = {0.0, 0.785398, -0.785398, 1.570796, -1.570796, 3.141593};
  for (float lateralStep = verticalEscapeLateralStep;
       lateralStep <= verticalEscapeMaxLateralStep + 0.001;
       lateralStep += verticalEscapeLateralStep) {
    for (const float angle : angles) {
      const float cosA = cos(angle);
      const float sinA = sin(angle);
      const float candX = lateralStep * (baseX * cosA - baseY * sinA);
      const float candY = lateralStep * (baseX * sinA + baseY * cosA);
      const pcl::PointXYZ lateralPoint(candX, candY, 0.0);
      const pcl::PointXYZ lateralUpPoint(candX, candY, targetZ);

      if (isEscapeSegmentClear(origin, lateralPoint, verticalEscapeRadius) &&
          isEscapeSegmentClear(lateralPoint, lateralUpPoint, verticalEscapeRadius)) {
        setEscapePath(path, {origin, lateralPoint, lateralUpPoint}, stamp);
        RCLCPP_WARN(nh->get_logger(),
                    "localPlanner: lateral vertical escape path selected, shift %.2fm %.2fm then climb %.2fm.",
                    candX, candY, targetZ);
        return true;
      }

      if (forceEscape) {
        const float lateralClearance = escapeSegmentClearanceSq(origin, lateralPoint);
        const float climbClearance = escapeSegmentClearanceSq(lateralPoint, lateralUpPoint);
        const float score = std::min(lateralClearance, climbClearance);
        if (score > bestScore) {
          bestScore = score;
          bestLateralPoint = lateralPoint;
          bestLateralUpPoint = lateralUpPoint;
        }
      }
    }
  }

  if (forceEscape && bestScore > verticalEscapeRelaxedRadius * verticalEscapeRelaxedRadius) {
    setEscapePath(path, {origin, bestLateralPoint, bestLateralUpPoint}, stamp);
    RCLCPP_WARN(nh->get_logger(),
                "localPlanner: relaxed lateral vertical escape path selected, shift %.2fm %.2fm then climb %.2fm, clearance %.2fm.",
                bestLateralPoint.x, bestLateralPoint.y, targetZ, sqrt(bestScore));
    return true;
  }

  return false;
}

void stateEstimationHandler(const nav_msgs::msg::Odometry::ConstSharedPtr& odom)
{
  if (stateInitDelay >= 0 && shiftGoalAtStart) {
    if (stateInitDelay == 0) {
      goalX += trackingCamScale * odom->pose.pose.position.x;
      goalY += trackingCamScale * odom->pose.pose.position.y;
      goalZ += trackingCamScale * odom->pose.pose.position.z;
    }
    stateInitDelay--;
    return;
  }

  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  float vehicleX = trackingCamScale * odom->pose.pose.position.x;
  float vehicleY = trackingCamScale * odom->pose.pose.position.y;
  float vehicleZ = trackingCamScale * odom->pose.pose.position.z;

  if (trackingCamBackward) {
    roll = -roll;
    pitch = -pitch;
    vehicleX = -vehicleX;
    vehicleY = -vehicleY;
  }

  float pointX1 = trackingCamXOffset;
  float pointY1 = trackingCamYOffset * cos(roll) - trackingCamZOffset * sin(roll);
  float pointZ1 = trackingCamYOffset * sin(roll) + trackingCamZOffset * cos(roll);

  float pointX2 = pointX1 * cos(pitch) + pointZ1 * sin(pitch);
  float pointY2 = pointY1;
  float pointZ2 = -pointX1 * sin(pitch) + pointZ1 * cos(pitch);

  vehicleX -= pointX2 * cos(yaw) - pointY2 * sin(yaw) - trackingCamXOffset;
  vehicleY -= pointX2 * sin(yaw) + pointY2 * cos(yaw) - trackingCamYOffset;
  vehicleZ -= pointZ2 - trackingCamZOffset;

  odomPointerLast = (odomPointerLast + 1) % odomQueLength;

  odomTime[odomPointerLast] = rclcpp::Time(odom->header.stamp).seconds();
  odomRoll[odomPointerLast] = roll;
  odomPitch[odomPointerLast] = pitch;
  odomYaw[odomPointerLast] = yaw;
  odomX[odomPointerLast] = vehicleX;
  odomY[odomPointerLast] = vehicleY;
  odomZ[odomPointerLast] = vehicleZ;
}

void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& laserCloud2)
{
  if (systemInitDelay > 0) {
    systemInitDelay--;
    return;
  }

  if (odomPointerLast < 0) {
    return;
  }

  //laserTime = rclcpp::Time(laserCloud2->header.stamp).seconds() - depthCloudDelay;
  laserTime = odomTime[odomPointerLast] - depthCloudDelay;

  while (odomPointerFront != odomPointerLast) {
    int odomPointerNext = (odomPointerFront + 1) % odomQueLength;
    if (fabs(laserTime - odomTime[odomPointerFront]) < fabs(laserTime - odomTime[odomPointerNext])) {
      break;
    }
    odomPointerFront = (odomPointerFront + 1) % odomQueLength;
  }

  float vehicleX = odomX[odomPointerFront];
  float vehicleY = odomY[odomPointerFront];
  float vehicleZ = odomZ[odomPointerFront];

  laserCloud->clear();
  pcl::fromROSMsg(*laserCloud2, *laserCloud);
  int laserCloudSize = laserCloud->points.size();

  laserCloudCrop->clear();
  for (int i = 0; i < laserCloudSize; i++) {
    if (laserCloud->points[i].z < maxRange) {
      laserCloudCrop->push_back(laserCloud->points[i]);
    }
  }

  laserCloudDwz->clear();
  downSizeFilter.setInputCloud(laserCloudCrop);
  downSizeFilter.filter(*laserCloudDwz);

  int laserCloudDwzSize = laserCloudDwz->points.size();
  // if (sim_name != "unity") {
  //   RCLCPP_WARN("sim_name and sensor_name are not unity and lidar, respectively.");
  //   for (int i = 0; i < laserCloudDwzSize; i++) {
  //       float pointX1 = laserCloudDwz->points[i].z;
  //       float pointY1 = -laserCloudDwz->points[i].x;
  //       float pointZ1 = -laserCloudDwz->points[i].y;

  //       float pointX2 = pointX1 * cosDepthCamPitch + pointZ1 * sinDepthCamPitch + depthCamXOffset;
  //       float pointY2 = pointY1 + depthCamYOffset;
  //       float pointZ2 = -pointX1 * sinDepthCamPitch + pointZ1 * cosDepthCamPitch + depthCamZOffset;

  //       float pointX3 = pointX2;
  //       float pointY3 = pointY2 * cosVehicleRoll - pointZ2 * sinVehicleRoll;
  //       float pointZ3 = pointY2 * sinVehicleRoll + pointZ2 * cosVehicleRoll;

  //       float pointX4 = pointX3 * cosVehiclePitch + pointZ3 * sinVehiclePitch;
  //       float pointY4 = pointY3;
  //       float pointZ4 = -pointX3 * sinVehiclePitch + pointZ3 * cosVehiclePitch;

  //       laserCloudDwz->points[i].x = pointX4 * cosVehicleYaw - pointY4 * sinVehicleYaw + vehicleX;
  //       laserCloudDwz->points[i].y = pointX4 * sinVehicleYaw + pointY4 * cosVehicleYaw + vehicleY;
  //       laserCloudDwz->points[i].z = pointZ4 + vehicleZ;

  //       // laserCloudDwz->points[i].x = pointX4 * cosVehicleYaw - pointY4 * sinVehicleYaw + vehicleX;
  //       // laserCloudDwz->points[i].y = pointX4 * sinVehicleYaw + pointY4 * cosVehicleYaw + vehicleY;
  //       // laserCloudDwz->points[i].z = pointZ4 + vehicleZ;
  //   }
  // }


  if (keepSurrCloud) {
    for (int i = 0; i < laserCloudDwzSize; i++) {
      float disX = laserCloudDwz->points[i].x - vehicleX;
      float disY = laserCloudDwz->points[i].y - vehicleY;
      float disZ = laserCloudDwz->points[i].z - vehicleZ;

      if (sqrt(disX * disX + disY * disY) < keepHoriDis && fabs(disZ) < keepVertDis) {
        laserCloudKeep->push_back(laserCloudDwz->points[i]);
      }
    }

    *laserCloudDwz += *laserCloudKeepDwz;

    laserCloudKeepDwz->clear();
    downSizeFilter.setInputCloud(laserCloudKeep);
    downSizeFilter.filter(*laserCloudKeepDwz);

    laserCloudKeep->clear();
    int laserCloudKeepDwzSize = laserCloudKeepDwz->points.size();
    for (int i = 0; i < laserCloudKeepDwzSize; i++) {
      float disX = laserCloudKeepDwz->points[i].x - vehicleX;
      float disY = laserCloudKeepDwz->points[i].y - vehicleY;
      float disZ = laserCloudKeepDwz->points[i].z - vehicleZ;

      if (sqrt(disX * disX + disY * disY) < keepHoriDis && fabs(disZ) < keepVertDis) {
        laserCloudKeep->push_back(laserCloudKeepDwz->points[i]);
      }
    }
  }

  newlaserCloud = true;
}

void trackPointHandler(const nav_msgs::msg::Odometry::ConstSharedPtr& odom)
{
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  trackRoll = roll;
  trackPitch = pitch;
  trackYaw = yaw;
  trackX = odom->pose.pose.position.x;
  trackY = odom->pose.pose.position.y;
  trackZ = odom->pose.pose.position.z;
}

int readPlyHeader(FILE *filePtr)
{
  char str[50];
  int val, pointNum;
  string strCur, strLast;
  while (strCur != "end_header") {
    val = fscanf(filePtr, "%s", str);
    if (val != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element") {
      val = fscanf(filePtr, "%d", &pointNum);
      if (val != 1) {
        printf ("\nError reading input files, exit.\n\n");
        exit(1);
      }
    }
  }

  return pointNum;
}

void readStartPaths()
{
  string fileName = pathFolder + "/startPaths.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  int pointNum = readPlyHeader(filePtr);

  pcl::PointXYZ point;
  int val1, val2, val3, val4, groupID;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &groupID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1) {
      printf ("\nError reading input files, exit.\n\n");
        exit(1);
    }

    if (groupID >= 0 && groupID < groupNum) {
      startPaths[groupID]->push_back(point);
    }
  }

  fclose(filePtr);
}

#if PLOTPATHSET == 1
void readPaths()
{
  string fileName = pathFolder + "/paths.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  int pointNum = readPlyHeader(filePtr);

  pcl::PointXYZI point;
  int pointSkipNum = 14;
  int pointSkipCount = 0;
  int val1, val2, val3, val4, val5, pathID;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%f", &point.intensity);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
      printf ("\nError reading input files, exit.\n\n");
        exit(1);
    }

    if (pathID >= 0 && pathID < pathNum) {
      pointSkipCount++;
      if (pointSkipCount > pointSkipNum) {
        paths[pathID]->push_back(point);
        pointSkipCount = 0;
      }
    }
  }

  fclose(filePtr);
}
#endif

void readPathList()
{
  string fileName = pathFolder + "/pathList.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  if (pathNum != readPlyHeader(filePtr)) {
    printf ("\nIncorrect path number, exit.\n\n");
    exit(1);
  }

  int val1, val2, val3, val4, val5, pathID, groupID;
  float endX, endY, endZ;
  for (int i = 0; i < pathNum; i++) {
    val1 = fscanf(filePtr, "%f", &endX);
    val2 = fscanf(filePtr, "%f", &endY);
    val3 = fscanf(filePtr, "%f", &endZ);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%d", &groupID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
      printf ("\nError reading input files, exit.\n\n");
        exit(1);
    }

    if (pathID >= 0 && pathID < pathNum && groupID >= 0 && groupID < groupNum) {
      pathList[pathID] = groupID;
      endPitchPathList[pathID] = -atan2(endZ, sqrt(endX * endX + endY * endY)) * 180.0 / PI;
      endYawPathList[pathID] = atan2(endY, endX) * 180.0 / PI;
      endZPathList[pathID] = endZ;
    }
  }

  fclose(filePtr);
}

void readCorrespondences()
{
  string fileName = pathFolder + "/correspondences.txt";

  FILE *filePtr = fopen(fileName.c_str(), "rb");
  if (filePtr == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  short pathID;
  int val1, gridVoxelID;
  for (int i = 0; i < gridVoxelNum; i++) {
    val1 = fread(&gridVoxelID, 4, 1, filePtr);
    if (val1 != 1) {
      printf ("\nError reading input files, exit.\n\n");
        exit(1);
    }

    while (1) {
      val1 = fread(&pathID, 2, 1, filePtr);
      if (val1 != 1) {
        printf ("\nError reading input files, exit.\n\n");
          exit(1);
      }

      if (pathID != -1) {
        if (gridVoxelID >= 0 && gridVoxelID < gridVoxelNum && pathID >= 0 && pathID < pathNum) {
          correspondences[gridVoxelID].push_back(pathID);
        }
      } else {
        break;
      }
    }
  }

  fclose(filePtr);
}

void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr& joy)
{
  if (joy->axes[2] >= -0.1 || joy->axes[5] < -0.1) {
    joyFwd = joy->axes[4];
    joyLeft = yawDiffLimit * joy->axes[3];
    joyUp = pitchDiffLimit * joy->axes[1];
  }

  if (joy->axes[5] < -0.1) {
    manualMode = true;
    autonomyMode = false;
    autoAdjustMode = false;
  } else {
    manualMode = false;

    if (joy->axes[2] < -0.1) {
      if (!autonomyMode) joyFwd = 1.0;
      autonomyMode = true;
    } else {
      autonomyMode = false;
      autoAdjustMode = false;
    }
  }

  if (joy->buttons[5] > 0.5) {
    laserCloudKeep->clear();
    laserCloudKeepDwz->clear();
  }
}

void goalHandler(const geometry_msgs::msg::PointStamped::ConstSharedPtr& goal)
{
  goalX = goal->point.x;
  goalY = goal->point.y;
  goalZ = goal->point.z;

  if (goalZ > maxElev) goalZ = maxElev;

  laserCloudKeep->clear();
  laserCloudKeepDwz->clear();
}

void autoModeHandler(const std_msgs::msg::Float32::ConstSharedPtr& autoMode)
{
  if (autonomyMode) {
    if (autoMode->data < -0.5) {
      autoAdjustMode = true;
    } else {
      autoAdjustMode = false;
    }

    joyFwd = autoMode->data;
    if (joyFwd < 0) joyFwd = 0;
    else if (joyFwd > 1.0) joyFwd = 1.0;
  }
}

void clearSurrCloudHandler(const std_msgs::msg::Empty::ConstSharedPtr&)
{
  laserCloudKeep->clear();
  laserCloudKeepDwz->clear();
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  nh = rclcpp::Node::make_shared("localPlanner");  declareAndGet("pathFolder", pathFolder);  declareAndGet("stateEstimationTopic", stateEstimationTopic);  declareAndGet("autonomyMode", autonomyMode);  declareAndGet("depthCloudTopic", depthCloudTopic);  declareAndGet("depthCloudDelay", depthCloudDelay);  declareAndGet("depthCamPitchOffset", depthCamPitchOffset);  declareAndGet("depthCamXOffset", depthCamXOffset);  declareAndGet("depthCamYOffset", depthCamYOffset);  declareAndGet("depthCamZOffset", depthCamZOffset);  declareAndGet("trackingCamBackward", trackingCamBackward);  declareAndGet("trackingCamXOffset", trackingCamXOffset);  declareAndGet("trackingCamYOffset", trackingCamYOffset);  declareAndGet("trackingCamZOffset", trackingCamZOffset);  declareAndGet("trackingCamScale", trackingCamScale);  declareAndGet("scanVoxelSize", scanVoxelSize);  declareAndGet("pointPerPathThre", pointPerPathThre);  declareAndGet("maxRange", maxRange);  declareAndGet("maxElev", maxElev);  declareAndGet("minObstacleRange", minObstacleRange);  declareAndGet("keepSurrCloud", keepSurrCloud);  declareAndGet("keepHoriDis", keepHoriDis);  declareAndGet("keepVertDis", keepVertDis);  declareAndGet("lowerBoundZ", lowerBoundZ);  declareAndGet("upperBoundZ", upperBoundZ);  declareAndGet("pitchDiffLimit", pitchDiffLimit);  declareAndGet("pitchWeight", pitchWeight);  declareAndGet("sensorMaxPitch", sensorMaxPitch);  declareAndGet("sensorMaxYaw", sensorMaxYaw);  declareAndGet("yawDiffLimit", yawDiffLimit);  declareAndGet("yawWeight", yawWeight);  declareAndGet("pathScale", pathScale);  declareAndGet("minPathScale", minPathScale);  declareAndGet("pathScaleStep", pathScaleStep);  declareAndGet("pathScaleBySpeed", pathScaleBySpeed);  declareAndGet("stopDis", stopDis);  declareAndGet("shiftGoalAtStart", shiftGoalAtStart);  declareAndGet("goalX", goalX);  declareAndGet("goalY", goalY);  declareAndGet("goalZ", goalZ);  declareAndGet("sim_name", sim_name);  declareAndGet("sensor_name", sensor_name);  declareAndGet("enableVerticalEscape", enableVerticalEscape);  declareAndGet("verticalEscapeMinObstacleHeight", verticalEscapeMinObstacleHeight);  declareAndGet("verticalEscapeLookAhead", verticalEscapeLookAhead);  declareAndGet("verticalEscapeCorridorWidth", verticalEscapeCorridorWidth);  declareAndGet("verticalEscapeClearance", verticalEscapeClearance);  declareAndGet("verticalEscapeMaxStep", verticalEscapeMaxStep);  declareAndGet("verticalEscapeRadius", verticalEscapeRadius);  declareAndGet("verticalEscapeLateralStep", verticalEscapeLateralStep);  declareAndGet("verticalEscapeMaxLateralStep", verticalEscapeMaxLateralStep);  declareAndGet("verticalEscapeRelaxedRadius", verticalEscapeRelaxedRadius);  declareAndGet("verticalEscapeInfluenceRadius", verticalEscapeInfluenceRadius);  declareAndGet("verticalEscapeMinCheckZ", verticalEscapeMinCheckZ);  declareAndGet("verticalEscapeFailFrames", verticalEscapeFailFrames);  declareAndGet("verticalEscapeFallbackStep", verticalEscapeFallbackStep);

  if (goalZ > maxElev) goalZ = maxElev;
  if (autonomyMode) {
    manualMode = false;
    joyFwd = 1.0;
  }

  auto subStateEstimation = nh->create_subscription<nav_msgs::msg::Odometry>(stateEstimationTopic, 5, stateEstimationHandler);

  auto subLaserCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>(depthCloudTopic, 5, laserCloudHandler);

  auto subTrackPoint = nh->create_subscription<nav_msgs::msg::Odometry>("/track_point_odom", 5, trackPointHandler);

  auto subJoystick = nh->create_subscription<sensor_msgs::msg::Joy>("/joy", 5, joystickHandler);

  auto subGoal = nh->create_subscription<geometry_msgs::msg::PointStamped>("/way_point", 5, goalHandler);

  auto subAutoMode = nh->create_subscription<std_msgs::msg::Float32>("/auto_mode", 5, autoModeHandler);

  auto subClearSurrCloud = nh->create_subscription<std_msgs::msg::Empty>("/clear_surr_cloud", 5, clearSurrCloudHandler);

  auto pubPath = nh->create_publisher<nav_msgs::msg::Path>("/path", 5);
  nav_msgs::msg::Path path;

  #if PLOTPATHSET == 1
  auto pubFreePaths = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/free_paths", 2);

  auto pubLaserCloud = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/collision_avoidance_cloud", 2);
  #endif

  printf ("\nReading path files.\n");

  for (int i = 0; i < laserCloudStackNum; i++) {
    laserCloudStack[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
  }
  for (int i = 0; i < groupNum; i++) {
    startPaths[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
  }
  #if PLOTPATHSET == 1
  for (int i = 0; i < pathNum; i++) {
    paths[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  #endif
  for (int i = 0; i < gridVoxelNum; i++) {
    correspondences[i].resize(0);
  }

  downSizeFilter.setLeafSize(scanVoxelSize, scanVoxelSize, scanVoxelSize);

  readStartPaths();
  #if PLOTPATHSET == 1
  readPaths();
  #endif
  readPathList();
  readCorrespondences();

  printf ("\nInitialization complete.\n\n");

  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(nh);

    if (newlaserCloud) {
      newlaserCloud = false;

      laserCloudStack[laserCloudCount]->clear();
      *laserCloudStack[laserCloudCount] = *laserCloudDwz;
      laserCloudCount = (laserCloudCount + 1) % laserCloudStackNum;

      plannerCloudStack->clear();
      for (int i = 0; i < laserCloudStackNum; i++) {
        *plannerCloudStack += *laserCloudStack[i];
      }

      plannerCloud->clear();
      downSizeFilter.setInputCloud(plannerCloudStack);
      downSizeFilter.filter(*plannerCloud);

      #if PLOTPATHSET == 1
      sensor_msgs::msg::PointCloud2 plannerCloud2;
      pcl::toROSMsg(*plannerCloud, plannerCloud2);
      plannerCloud2.header.stamp = timeFromSec(laserTime);
      plannerCloud2.header.frame_id = "map";
      pubLaserCloud->publish(plannerCloud2);
      #endif

      float sinTrackPitch = sin(trackPitch);
      float cosTrackPitch = cos(trackPitch);
      float sinTrackYaw = sin(trackYaw);
      float cosTrackYaw = cos(trackYaw);

      int plannerCloudSize = plannerCloud->points.size();
      for (int i = 0; i < plannerCloudSize; i++) {
        float pointX1 = plannerCloud->points[i].x - trackX;
        float pointY1 = plannerCloud->points[i].y - trackY;
        float pointZ1 = plannerCloud->points[i].z - trackZ;

        float pointX2 = pointX1 * cosTrackYaw + pointY1 * sinTrackYaw;
        float pointY2 = -pointX1 * sinTrackYaw + pointY1 * cosTrackYaw;
        float pointZ2 = pointZ1;

        plannerCloud->points[i].x = pointX2 * cosTrackPitch - pointZ2 * sinTrackPitch;
        plannerCloud->points[i].y = pointY2;
        plannerCloud->points[i].z = pointX2 * sinTrackPitch + pointZ2 * cosTrackPitch;
      }

      float goalX1 = (goalX - trackX) * cosTrackYaw + (goalY - trackY) * sinTrackYaw;
      float goalY1 = -(goalX - trackX) * sinTrackYaw + (goalY - trackY) * cosTrackYaw;
      float goalZ1 = goalZ - trackZ;

      float relativeGoalX = (goalX1 * cosTrackPitch - goalZ1 * sinTrackPitch);
      float relativeGoalY = goalY1;
      float relativeGoalZ = (goalX1 * sinTrackPitch + goalZ1 * cosTrackPitch);

      float relativeGoalDis = sqrt(relativeGoalX * relativeGoalX + relativeGoalY * relativeGoalY);
      float relativeGoalPitch = -atan2(relativeGoalZ, sqrt(relativeGoalX * relativeGoalX
                            + relativeGoalY * relativeGoalY)) * 180.0 / PI;
      float relativeGoalYaw = atan2(relativeGoalY, relativeGoalX) * 180.0 / PI;

      if (relativeGoalPitch < -pitchDiffLimit) relativeGoalPitch = -pitchDiffLimit;
      else if (relativeGoalPitch > pitchDiffLimit) relativeGoalPitch = pitchDiffLimit;
      if (relativeGoalYaw < -yawDiffLimit) relativeGoalYaw = -yawDiffLimit;
      else if (relativeGoalYaw > yawDiffLimit) relativeGoalYaw = yawDiffLimit;

      if (manualMode || (autonomyMode && autoAdjustMode)) {
        relativeGoalDis = 1000.0;
        relativeGoalPitch = 0;
        relativeGoalYaw = 0;
      } else if (!autonomyMode) {
        relativeGoalDis = 1000.0;
        relativeGoalPitch = -joyUp;
        relativeGoalYaw = joyLeft;
      }

      bool pathPublished = false;
      float pathScaleOri = pathScale;
      if (manualMode || (autonomyMode && autoAdjustMode)) pathScale = minPathScale;
      else if (pathScaleBySpeed) pathScale *= joyFwd;
      if (pathScale < minPathScale) pathScale = minPathScale;

      while (pathScale >= minPathScale) {
        for (int i = 0; i < pathNum; i++) {
          clearPathList[i] = 0;
        }
        for (int i = 0; i < groupNum; i++) {
          clearPathPerGroupScore[i] = 0;
        }

        for (int i = 0; i < plannerCloudSize; i++) {
          float x = plannerCloud->points[i].x / pathScale;
          float y = plannerCloud->points[i].y / pathScale;
          float z = plannerCloud->points[i].z / pathScale;
          float dis = sqrt(x * x + y * y);

          if (x > 0 && dis > minObstacleRange / pathScale &&
              (dis <= (relativeGoalDis + stopDis) / pathScale || relativeGoalX < 0) && 
              z > lowerBoundZ / pathScale && z < upperBoundZ / pathScale) {
            float scaleY = x / gridVoxelOffsetX + searchRadiusHori / gridVoxelOffsetY
                         * (gridVoxelOffsetX - x) / gridVoxelOffsetX;
            float scaleZ = x / gridVoxelOffsetX + searchRadiusVert / gridVoxelOffsetZ
                         * (gridVoxelOffsetX - x) / gridVoxelOffsetX;

            int indX = int((gridVoxelOffsetX + gridVoxelSize / 2 - x) / gridVoxelSize);
            int indY = int((gridVoxelOffsetY + gridVoxelSize / 2 - y / scaleY) / gridVoxelSize);
            int indZ = int((gridVoxelOffsetZ + gridVoxelSize / 2 - z / scaleZ) / gridVoxelSize);
            if (indX >= 0 && indX < gridVoxelNumX && indY >= 0 && indY < gridVoxelNumY && 
                indZ >= 0 && indZ < gridVoxelNumZ) {
              int ind = gridVoxelNumY * gridVoxelNumZ * indX + gridVoxelNumZ * indY + indZ;
              int blockedPathByVoxelNum = correspondences[ind].size();
              for (int j = 0; j < blockedPathByVoxelNum; j++) {
                clearPathList[correspondences[ind][j]]++;
              }
            }
          }
        }

        for (int i = 0; i < pathNum; i++) {
          float vehiclePitch = odomPitch[odomPointerFront];
          float vehicleYaw = odomYaw[odomPointerFront];
          float pitch = endPitchPathList[i];
          float yaw = trackYaw * 180.0 / PI + endYawPathList[i];
          float pitchDiff = fabs(pitch + trackPitch * 180.0 / PI - vehiclePitch * 180.0 / PI - depthCamPitchOffset * 180.0 / PI);
          float yawDiff = fabs(yaw - vehicleYaw * 180.0 / PI);
          if (yawDiff > 180.0) yawDiff = 360.0 - yawDiff;
          float elev = trackZ + endZPathList[i];
          if (yawDiff > sensorMaxYaw || pitchDiff > sensorMaxPitch || elev > maxElev) {
            clearPathList[i] += pointPerPathThre;
            continue;
          }
          if (clearPathList[i] < pointPerPathThre) {
            float pitchDiff = fabs(relativeGoalPitch - endPitchPathList[i]);
            if (pitchDiff > pitchDiffLimit) {
              pitchDiff = pitchDiffLimit;
            }
            float yawDiff = fabs(relativeGoalYaw - endYawPathList[i]);
            if (yawDiff > 180.0) {
              yawDiff = 360.0 - yawDiff;
            }

            float score = (1 - pitchWeight * pitchDiff) * (1 - yawWeight * yawDiff);
            if (score < 0) score = 0;
            clearPathPerGroupScore[pathList[i]] += score + 0.000001;
          }
        }

        float maxScore = 0;
        int selectedGroupID = -1;
        for (int i = 0; i < groupNum; i++) {
          if (maxScore < clearPathPerGroupScore[i]) {
            maxScore = clearPathPerGroupScore[i];
            selectedGroupID = i;
          }
        }

        if (selectedGroupID >= 0 && (relativeGoalDis > stopDis || relativeGoalX > 0)) {
          int selectedPathLength = startPaths[selectedGroupID]->points.size();
          path.poses.resize(selectedPathLength);
          for (int i = 0; i < selectedPathLength; i++) {
            float x = startPaths[selectedGroupID]->points[i].x;
            float y = startPaths[selectedGroupID]->points[i].y;
            float z = startPaths[selectedGroupID]->points[i].z;
            float dis = sqrt(x * x + y * y);

            if (dis <= relativeGoalDis / pathScale || relativeGoalX < 0) {
              path.poses[i].pose.position.x = pathScale * x;
              path.poses[i].pose.position.y = pathScale * y;
              path.poses[i].pose.position.z = pathScale * z;
            } else {
              path.poses.resize(i);
              break;
            }
          }

          path.header.stamp = timeFromSec(laserTime);
          path.header.frame_id = "track_point";
          pubPath->publish(path);
          pathPublished = true;

          #if PLOTPATHSET == 1
          freePaths->clear();
          pcl::PointXYZI point;
          for (int i = 0; i < pathNum; i++) {
            if (clearPathList[i] < pointPerPathThre) {
              int freePathLength = paths[i]->points.size();
              for (int j = 0; j < freePathLength; j++) {
                point = paths[i]->points[j];
                float dis = sqrt(point.x * point.x + point.y * point.y);
                if (dis <= (relativeGoalDis + stopDis) / pathScale || relativeGoalX < 0) {
                  point.x *= pathScale;
                  point.y *= pathScale;
                  point.z *= pathScale;

                  freePaths->push_back(point);
                }
              }
            }
          }

          sensor_msgs::msg::PointCloud2 freePaths2;
          pcl::toROSMsg(*freePaths, freePaths2);
          freePaths2.header.stamp = timeFromSec(laserTime);
          freePaths2.header.frame_id = "track_point";
          pubFreePaths->publish(freePaths2);
          #endif
        }

        if (selectedGroupID < 0) {
          pathScale -= pathScaleStep;
        } else {
          break;
        }
      }
      pathScale = pathScaleOri;

      if (pathPublished && path.poses.size() <= 1) {
        pathPublished = false;
      }

      bool verticalEscapePublished = false;
      if (!pathPublished) {
        ++failedPathCount;
        const bool forceEscape = failedPathCount >= verticalEscapeFailFrames;
        verticalEscapePublished = tryVerticalEscapePath(path, relativeGoalX, relativeGoalY,
                                                       relativeGoalDis, forceEscape, laserTime);
        pathPublished = verticalEscapePublished;
      } else {
        failedPathCount = 0;
      }

      if (!pathPublished) {
        path.poses.resize(1);
        path.poses[0].pose.position.x = 0;
        path.poses[0].pose.position.y = 0;
        path.poses[0].pose.position.z = 0;

        path.header.stamp = timeFromSec(laserTime);
        path.header.frame_id = "track_point";
      }

      if (!pathPublished) {
        pubPath->publish(path);

        #if PLOTPATHSET == 1
        freePaths->clear();
        sensor_msgs::msg::PointCloud2 freePaths2;
        pcl::toROSMsg(*freePaths, freePaths2);
        freePaths2.header.stamp = timeFromSec(laserTime);
        freePaths2.header.frame_id = "track_point";
        pubFreePaths->publish(freePaths2);
        #endif
      } else if (verticalEscapePublished) {
        pubPath->publish(path);
      }
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}
