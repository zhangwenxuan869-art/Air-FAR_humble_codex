#ifndef AIRFAR_ROS2_COMPAT_H
#define AIRFAR_ROS2_COMPAT_H

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <nav_graph_msg/msg/graph.hpp>
#include <nav_graph_msg/msg/node.hpp>
#include <route_goal_msg/msg/route_goal.hpp>
#include <visibility_graph_msg/msg/graph.hpp>
#include <visibility_graph_msg/msg/node.hpp>

#define AIRFAR_LOGGER rclcpp::get_logger("airfar_planner")

#define ROS_INFO(...) RCLCPP_INFO(AIRFAR_LOGGER, __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(AIRFAR_LOGGER, __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(AIRFAR_LOGGER, __VA_ARGS__)
#define ROS_DEBUG(...) RCLCPP_DEBUG(AIRFAR_LOGGER, __VA_ARGS__)
#define ROS_INFO_COND(cond, ...) do { if (cond) RCLCPP_INFO(AIRFAR_LOGGER, __VA_ARGS__); } while (0)
#define ROS_WARN_COND(cond, ...) do { if (cond) RCLCPP_WARN(AIRFAR_LOGGER, __VA_ARGS__); } while (0)
#define ROS_ERROR_COND(cond, ...) do { if (cond) RCLCPP_ERROR(AIRFAR_LOGGER, __VA_ARGS__); } while (0)
#define ROS_WARN_ONCE(...) RCLCPP_WARN_ONCE(AIRFAR_LOGGER, __VA_ARGS__)
#define ROS_WARN_THROTTLE(period, ...) RCLCPP_WARN(AIRFAR_LOGGER, __VA_ARGS__)
#define ROS_ERROR_THROTTLE(period, ...) RCLCPP_ERROR(AIRFAR_LOGGER, __VA_ARGS__)

namespace geometry_msgs {
using Point = msg::Point;
using Point32 = msg::Point32;
using PointStamped = msg::PointStamped;
using PolygonStamped = msg::PolygonStamped;
using PoseStamped = msg::PoseStamped;
using Quaternion = msg::Quaternion;
using PointStampedConstPtr = msg::PointStamped::ConstSharedPtr;
}

namespace nav_msgs {
using Odometry = msg::Odometry;
using Path = msg::Path;
using OdometryConstPtr = msg::Odometry::ConstSharedPtr;
}

namespace sensor_msgs {
using Joy = msg::Joy;
using PointCloud2 = msg::PointCloud2;
using JoyConstPtr = msg::Joy::ConstSharedPtr;
using PointCloud2ConstPtr = msg::PointCloud2::ConstSharedPtr;
}

namespace std_msgs {
using Bool = msg::Bool;
using ColorRGBA = msg::ColorRGBA;
using Empty = msg::Empty;
using Float32 = msg::Float32;
using String = msg::String;
using BoolConstPtr = msg::Bool::ConstSharedPtr;
using EmptyConstPtr = msg::Empty::ConstSharedPtr;
}

namespace visualization_msgs {
using Marker = msg::Marker;
using MarkerArray = msg::MarkerArray;
}

namespace nav_graph_msg {
using Graph = msg::Graph;
using Node = msg::Node;
using GraphConstPtr = msg::Graph::ConstSharedPtr;
}

namespace route_goal_msg {
using RouteGoal = msg::RouteGoal;
}

namespace visibility_graph_msg {
using Graph = msg::Graph;
using Node = msg::Node;
using GraphConstPtr = msg::Graph::ConstSharedPtr;
}

namespace std_srvs {
using Trigger = srv::Trigger;
}

#endif
