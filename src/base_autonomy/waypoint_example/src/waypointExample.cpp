#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"



using namespace std;

const double PI = 3.1415926;

double waypointXYRadius = 0.5;
double waypointZBound = 5.0;
double waitTime = 0;
double waitTimeStart = 0;
bool isWaiting = false;
double frameRate = 5.0;
double speed = 1.0;
bool sendSpeed = true;
bool sendBoundary = true;

std::vector<std::pair<double, double>> waypoints = {{3.0, 0.0}, {6.0, 0.0}};
std::vector<double> waypointsRaw = {3.0, 0.0, 6.0, 0.0};

float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
double curTime = 0, waypointTime = 0;

rclcpp::Node::SharedPtr nh;

bool loadWaypointsFromParameter()
{
  if (waypointsRaw.empty()) {
    RCLCPP_ERROR(nh->get_logger(), "Parameter 'waypoints' is empty. Expected [x1, y1, x2, y2, ...].");
    return false;
  }

  if (waypointsRaw.size() % 2 != 0) {
    RCLCPP_ERROR(nh->get_logger(), "Parameter 'waypoints' size must be even. Current size: %zu", waypointsRaw.size());
    return false;
  }

  waypoints.clear();
  for (size_t i = 0; i < waypointsRaw.size(); i += 2) {
    waypoints.emplace_back(waypointsRaw[i], waypointsRaw[i + 1]);
  }

  return true;
}

// Load boundary polygons from parameters boundary_polygon_0, boundary_polygon_1, ...
// Each parameter is a flat [x1, y1, x2, y2, ...] list of 2D points forming one closed polygon.
// Polygons are encoded with z = polygon index so local_planner treats them as independent walls.
bool loadBoundaryPolygonsFromParameter(geometry_msgs::msg::PolygonStamped& boundaryMsgs)
{
  boundaryMsgs.polygon.points.clear();
  int polygonCount = 0;

  for (int idx = 0; ; idx++) {
    std::string paramName = "boundary_polygon_" + std::to_string(idx);
    std::vector<double> rawPoints;
    nh->declare_parameter<std::vector<double>>(paramName, std::vector<double>());
    nh->get_parameter(paramName, rawPoints);

    if (rawPoints.empty()) {
      break;  // No more polygons (must be consecutive starting from 0)
    }

    if (rawPoints.size() % 2 != 0) {
      RCLCPP_ERROR(nh->get_logger(), "Parameter '%s' must have an even number of values (x,y pairs). Got %zu.",
                   paramName.c_str(), rawPoints.size());
      exit(1);
    }

    if (rawPoints.size() < 6) {
      RCLCPP_ERROR(nh->get_logger(), "Parameter '%s' needs at least 3 points (6 values).",
                   paramName.c_str());
      exit(1);
    }

    float z = static_cast<float>(idx);
    size_t nPoints = rawPoints.size() / 2;

    for (size_t i = 0; i < nPoints; i++) {
      geometry_msgs::msg::Point32 p;
      p.x = static_cast<float>(rawPoints[i * 2]);
      p.y = static_cast<float>(rawPoints[i * 2 + 1]);
      p.z = z;
      boundaryMsgs.polygon.points.push_back(p);
    }

    // Close the polygon by repeating its first point (same z)
    geometry_msgs::msg::Point32 closeP;
    closeP.x = static_cast<float>(rawPoints[0]);
    closeP.y = static_cast<float>(rawPoints[1]);
    closeP.z = z;
    boundaryMsgs.polygon.points.push_back(closeP);

    polygonCount++;
    RCLCPP_INFO(nh->get_logger(), "Loaded boundary_polygon_%d with %zu points.", idx, nPoints);
  }

  return polygonCount > 0;
}

// vehicle pose callback function
void poseHandler(const nav_msgs::msg::Odometry::SharedPtr pose)
{
  curTime = rclcpp::Time(pose->header.stamp).seconds(); 
  vehicleX = pose->pose.pose.position.x;
  vehicleY = pose->pose.pose.position.y;
  vehicleZ = pose->pose.pose.position.z;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  nh = rclcpp::Node::make_shared("waypointExample");

  nh->declare_parameter<double>("waypointXYRadius", waypointXYRadius);
  nh->declare_parameter<double>("waypointZBound", waypointZBound);
  nh->declare_parameter<double>("waitTime", waitTime);
  nh->declare_parameter<double>("frameRate", frameRate);
  nh->declare_parameter<double>("speed", speed);
  nh->declare_parameter<bool>("sendSpeed", sendSpeed);
  nh->declare_parameter<bool>("sendBoundary", sendBoundary);
  nh->declare_parameter<std::vector<double>>("waypoints", waypointsRaw);

  nh->get_parameter("waypointXYRadius", waypointXYRadius);
  nh->get_parameter("waypointZBound", waypointZBound);
  nh->get_parameter("waitTime", waitTime);
  nh->get_parameter("frameRate", frameRate);
  nh->get_parameter("speed", speed);
  nh->get_parameter("sendSpeed", sendSpeed);
  nh->get_parameter("sendBoundary", sendBoundary);
  nh->get_parameter("waypoints", waypointsRaw);

  if (!loadWaypointsFromParameter()) {
    exit(1);
  }

  if (frameRate <= 0.0) {
    RCLCPP_ERROR(nh->get_logger(), "Parameter 'frameRate' must be > 0. Current value: %.3f", frameRate);
    exit(1);
  }
  
  auto subPose = nh->create_subscription<nav_msgs::msg::Odometry>("/state_estimation", 5, poseHandler);

  // 发布到 /goal_pose，与 rviz 一致
  auto pubGoalPose = nh->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 5);
  geometry_msgs::msg::PoseStamped goalPoseMsgs;
  goalPoseMsgs.header.frame_id = "map";
  goalPoseMsgs.pose.orientation.w = 1.0;  // 无旋转

  auto pubSpeed = nh->create_publisher<std_msgs::msg::Float32>("/speed", 5);
  std_msgs::msg::Float32 speedMsgs;

  auto pubBoundary = nh->create_publisher<geometry_msgs::msg::PolygonStamped>("/navigation_boundary", 5);
  geometry_msgs::msg::PolygonStamped boundaryMsgs;
  boundaryMsgs.header.frame_id = "map";

  // Load boundary polygons; if none configured, boundary is unlimited.
  if (sendBoundary) {
    if (!loadBoundaryPolygonsFromParameter(boundaryMsgs)) {
      sendBoundary = false;
      RCLCPP_WARN(nh->get_logger(), "No boundary polygons configured. Boundary will be unlimited.");
    }
  }

  int wayPointID = 0;
  int waypointSize = static_cast<int>(waypoints.size());

  if (waypointSize == 0) {
    RCLCPP_INFO(nh->get_logger(), "No waypoint available, exit.");
    exit(1);
  }

  RCLCPP_INFO(nh->get_logger(), "Waypoint example: publishing %d goals to /goal_pose.", waypointSize);

  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(nh);

    double gx = waypoints[wayPointID].first;
    double gy = waypoints[wayPointID].second;
    float disX = vehicleX - gx;
    float disY = vehicleY - gy;
    float disZ = vehicleZ - 0.0;

    // start waiting if the current waypoint is reached
    if (sqrt(disX * disX + disY * disY) < waypointXYRadius && fabs(disZ) < waypointZBound && !isWaiting) {
      waitTimeStart = curTime;
      isWaiting = true;
    }

    // move to the next waypoint after waiting is over
    if (isWaiting && waitTimeStart + waitTime < curTime && wayPointID < waypointSize - 1) {
      wayPointID++;
      isWaiting = false;
    }

    // publish goal_pose, speed, and boundary messages at certain frame rate
    if (curTime - waypointTime > 1.0 / frameRate) {
      if (!isWaiting) {
        goalPoseMsgs.header.stamp = nh->now();
        goalPoseMsgs.pose.position.x = waypoints[wayPointID].first;
        goalPoseMsgs.pose.position.y = waypoints[wayPointID].second;
        goalPoseMsgs.pose.position.z = 0.0;
        pubGoalPose->publish(goalPoseMsgs);
      }

      if (sendSpeed) {
        speedMsgs.data = speed;
        pubSpeed->publish(speedMsgs);
      }

      if (sendBoundary) {
        boundaryMsgs.header.stamp = rclcpp::Time(static_cast<uint64_t>(curTime * 1e9));
        pubBoundary->publish(boundaryMsgs);
      }

      waypointTime = curTime;
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}
