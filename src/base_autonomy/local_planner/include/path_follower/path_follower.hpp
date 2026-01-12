#ifndef PATH_FOLLOWER__PATH_FOLLOWER_HPP_
#define PATH_FOLLOWER__PATH_FOLLOWER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/int8.hpp>

namespace path_follower
{

// Constants
constexpr double PI = 3.14159265358979323846;
constexpr double DEG_TO_RAD = PI / 180.0;

// Configuration parameters structure
struct PathFollowerConfig
{
  // Sensor offsets
  double sensorOffsetX = 0.0;
  double sensorOffsetY = 0.0;

  // Control parameters
  double lookAheadDis = 0.5;
  double yawRateGain = 7.5;
  double stopYawRateGain = 7.5;
  double maxYawRate = 45.0;   // degrees
  double maxSpeed = 1.0;
  double maxAccel = 1.0;

  // Direction switching
  bool twoWayDrive = true;
  double switchTimeThre = 1.0;
  double dirDiffThre = 0.1;
  double omniDirDiffThre = 1.5;

  // Speed control
  double noRotSpeed = 10.0;
  double stopDisThre = 0.2;
  double slowDwnDisThre = 1.0;

  // Incline-based slowdown
  bool useInclRateToSlow = false;
  double inclRateThre = 120.0;  // degrees/sec
  double slowRate1 = 0.25;
  double slowRate2 = 0.5;
  double slowTime1 = 2.0;
  double slowTime2 = 2.0;

  // Incline-based stop
  bool useInclToStop = false;
  double inclThre = 45.0;  // degrees
  double stopTime = 5.0;

  // Rotation control
  bool noRotAtStop = false;
  bool noRotAtGoal = true;

  // Goal parameters
  double goalCloseDis = 1.0;

  // System
  bool is_real_robot = false;
  int pubSkipNum = 1;

  void loadFromParameters(rclcpp::Node* node);
};

// Main PathFollower class
class PathFollower : public rclcpp::Node
{
public:
  explicit PathFollower(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~PathFollower() = default;

  // Delete copy/move
  PathFollower(const PathFollower&) = delete;
  PathFollower& operator=(const PathFollower&) = delete;

private:
  // Configuration
  PathFollowerConfig config_;

  // Vehicle state
  struct VehicleState
  {
    float x = 0, y = 0, z = 0;
    float roll = 0, pitch = 0, yaw = 0;
    float yawRate = 0;
    float speed = 0;
    double odomTime = 0;
  } vehicle_;

  struct VehicleStateRecorded
  {
    float x = 0, y = 0, z = 0;
    float roll = 0, pitch = 0, yaw = 0;
  } vehicleRec_;

  // Path following state
  struct PathFollowState
  {
    nav_msgs::msg::Path path;
    int pointID = 0;
    bool initialized = false;
    bool forward = true;
    double switchTime = 0;
    double slowInitTime = 0;
    double stopInitTime = 0;
    int pubSkipCounter = 0;
    int safetyStop = 0;
  } pathState_;

  // ROS2 interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_stop_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd_vel_;

  // Timer for 100Hz control loop
  rclcpp::TimerBase::SharedPtr timer_;

  // Initialization methods
  void declareParameters();
  void loadParameters();
  void initializeSubscribers();
  void initializePublishers();
  void initializeTimer();

  // Callback methods
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void stopCallback(const std_msgs::msg::Int8::SharedPtr msg);

  // Main control loop (called by timer at 100Hz)
  void timerCallback();

  // Control logic methods
  void updatePathFollowing();
  float normalizeAngle(float angle);
  void applyAccelerationLimits(float targetSpeed, float dirDiff, float dis, float endDis);
  void applySafetyChecks();
  void publishCommand(float linearX, float linearY, float angularZ);
};

}  // namespace path_follower

#endif  // PATH_FOLLOWER__PATH_FOLLOWER_HPP_
