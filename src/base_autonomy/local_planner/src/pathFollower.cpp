#include <cmath>

#include <path_follower/path_follower.hpp>

#include <tf2/transform_datatypes.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace path_follower
{

// ============================================================================
// PathFollowerConfig Implementation
// ============================================================================

void PathFollowerConfig::loadFromParameters(rclcpp::Node* node)
{
  // Sensor offsets
  node->get_parameter("sensorOffsetX", sensorOffsetX);
  node->get_parameter("sensorOffsetY", sensorOffsetY);

  // Control parameters
  node->get_parameter("lookAheadDis", lookAheadDis);
  node->get_parameter("yawRateGain", yawRateGain);
  node->get_parameter("stopYawRateGain", stopYawRateGain);
  node->get_parameter("maxYawRate", maxYawRate);
  node->get_parameter("maxSpeed", maxSpeed);
  node->get_parameter("maxAccel", maxAccel);

  // Direction switching
  node->get_parameter("twoWayDrive", twoWayDrive);
  node->get_parameter("switchTimeThre", switchTimeThre);
  node->get_parameter("dirDiffThre", dirDiffThre);
  node->get_parameter("omniDirDiffThre", omniDirDiffThre);

  // Speed control
  node->get_parameter("noRotSpeed", noRotSpeed);
  node->get_parameter("stopDisThre", stopDisThre);
  node->get_parameter("slowDwnDisThre", slowDwnDisThre);

  // Incline-based slowdown
  node->get_parameter("useInclRateToSlow", useInclRateToSlow);
  node->get_parameter("inclRateThre", inclRateThre);
  node->get_parameter("slowRate1", slowRate1);
  node->get_parameter("slowRate2", slowRate2);
  node->get_parameter("slowTime1", slowTime1);
  node->get_parameter("slowTime2", slowTime2);

  // Incline-based stop
  node->get_parameter("useInclToStop", useInclToStop);
  node->get_parameter("inclThre", inclThre);
  node->get_parameter("stopTime", stopTime);

  // Rotation control
  node->get_parameter("noRotAtStop", noRotAtStop);
  node->get_parameter("noRotAtGoal", noRotAtGoal);

  // Goal parameters
  node->get_parameter("goalCloseDis", goalCloseDis);

  // System
  node->get_parameter("is_real_robot", is_real_robot);
  node->get_parameter("pubSkipNum", pubSkipNum);
}

// ============================================================================
// PathFollower Implementation
// ============================================================================

PathFollower::PathFollower(const rclcpp::NodeOptions& options)
: rclcpp::Node("path_follower", options)
{
  // Declare and load parameters
  declareParameters();
  loadParameters();

  // Initialize ROS2 interfaces
  initializeSubscribers();
  initializePublishers();
  initializeTimer();

  RCLCPP_INFO(get_logger(), "PathFollower node initialized");
}

void PathFollower::declareParameters()
{
  // Sensor offsets
  declare_parameter("sensorOffsetX", config_.sensorOffsetX);
  declare_parameter("sensorOffsetY", config_.sensorOffsetY);

  // Control parameters
  declare_parameter("lookAheadDis", config_.lookAheadDis);
  declare_parameter("yawRateGain", config_.yawRateGain);
  declare_parameter("stopYawRateGain", config_.stopYawRateGain);
  declare_parameter("maxYawRate", config_.maxYawRate);
  declare_parameter("maxSpeed", config_.maxSpeed);
  declare_parameter("maxAccel", config_.maxAccel);

  // Direction switching
  declare_parameter("twoWayDrive", config_.twoWayDrive);
  declare_parameter("switchTimeThre", config_.switchTimeThre);
  declare_parameter("dirDiffThre", config_.dirDiffThre);
  declare_parameter("omniDirDiffThre", config_.omniDirDiffThre);

  // Speed control
  declare_parameter("noRotSpeed", config_.noRotSpeed);
  declare_parameter("stopDisThre", config_.stopDisThre);
  declare_parameter("slowDwnDisThre", config_.slowDwnDisThre);

  // Incline-based slowdown
  declare_parameter("useInclRateToSlow", config_.useInclRateToSlow);
  declare_parameter("inclRateThre", config_.inclRateThre);
  declare_parameter("slowRate1", config_.slowRate1);
  declare_parameter("slowRate2", config_.slowRate2);
  declare_parameter("slowTime1", config_.slowTime1);
  declare_parameter("slowTime2", config_.slowTime2);

  // Incline-based stop
  declare_parameter("useInclToStop", config_.useInclToStop);
  declare_parameter("inclThre", config_.inclThre);
  declare_parameter("stopTime", config_.stopTime);

  // Rotation control
  declare_parameter("noRotAtStop", config_.noRotAtStop);
  declare_parameter("noRotAtGoal", config_.noRotAtGoal);

  // Goal parameters
  declare_parameter("goalCloseDis", config_.goalCloseDis);

  // System
  declare_parameter("is_real_robot", config_.is_real_robot);
  declare_parameter("pubSkipNum", config_.pubSkipNum);
}

void PathFollower::loadParameters()
{
  config_.loadFromParameters(this);
}

void PathFollower::initializeSubscribers()
{
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "/state_estimation", 5,
    std::bind(&PathFollower::odomCallback, this, std::placeholders::_1));

  sub_path_ = create_subscription<nav_msgs::msg::Path>(
    "/autonomy_stack/path", 5,
    std::bind(&PathFollower::pathCallback, this, std::placeholders::_1));

  sub_stop_ = create_subscription<std_msgs::msg::Int8>(
    "/stop", 5,
    std::bind(&PathFollower::stopCallback, this, std::placeholders::_1));
}

void PathFollower::initializePublishers()
{
  pub_cmd_vel_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 5);
}

void PathFollower::initializeTimer()
{
  // 100Hz control loop
  timer_ = create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&PathFollower::timerCallback, this));
}

// ============================================================================
// Callback Methods
// ============================================================================

void PathFollower::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  vehicle_.odomTime = rclcpp::Time(msg->header.stamp).seconds();

  double roll, pitch, yaw;
  const auto& quat = msg->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(quat.x, quat.y, quat.z, quat.w)).getRPY(roll, pitch, yaw);

  vehicle_.roll = static_cast<float>(roll);
  vehicle_.pitch = static_cast<float>(pitch);
  vehicle_.yaw = static_cast<float>(yaw);

  // Compensate for sensor offset
  vehicle_.x = static_cast<float>(msg->pose.pose.position.x
                - cos(yaw) * config_.sensorOffsetX
                + sin(yaw) * config_.sensorOffsetY);
  vehicle_.y = static_cast<float>(msg->pose.pose.position.y
                - sin(yaw) * config_.sensorOffsetX
                - cos(yaw) * config_.sensorOffsetY);
  vehicle_.z = static_cast<float>(msg->pose.pose.position.z);

  // Check incline for stopping
  if (config_.useInclToStop) {
    if (std::abs(vehicle_.roll) > config_.inclThre * DEG_TO_RAD ||
        std::abs(vehicle_.pitch) > config_.inclThre * DEG_TO_RAD) {
      pathState_.stopInitTime = vehicle_.odomTime;
    }
  }

  // Check incline rate for slowdown
  if (config_.useInclRateToSlow) {
    if (std::abs(msg->twist.twist.angular.x) > config_.inclRateThre * DEG_TO_RAD ||
        std::abs(msg->twist.twist.angular.y) > config_.inclRateThre * DEG_TO_RAD) {
      pathState_.slowInitTime = vehicle_.odomTime;
    }
  }
}

void PathFollower::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  const size_t pathSize = msg->poses.size();
  pathState_.path.poses.resize(pathSize);

  for (size_t i = 0; i < pathSize; ++i) {
    pathState_.path.poses[i].pose.position.x = msg->poses[i].pose.position.x;
    pathState_.path.poses[i].pose.position.y = msg->poses[i].pose.position.y;
    pathState_.path.poses[i].pose.position.z = msg->poses[i].pose.position.z;
  }

  // Record vehicle state when path is received
  vehicleRec_.x = vehicle_.x;
  vehicleRec_.y = vehicle_.y;
  vehicleRec_.z = vehicle_.z;
  vehicleRec_.roll = vehicle_.roll;
  vehicleRec_.pitch = vehicle_.pitch;
  vehicleRec_.yaw = vehicle_.yaw;

  pathState_.pointID = 0;
  pathState_.initialized = true;
}

void PathFollower::stopCallback(const std_msgs::msg::Int8::SharedPtr msg)
{
  pathState_.safetyStop = msg->data;
}

// ============================================================================
// Timer Callback - Main Control Loop
// ============================================================================

void PathFollower::timerCallback()
{
  if (pathState_.initialized) {
    updatePathFollowing();
  }
}

// ============================================================================
// Path Following Logic
// ============================================================================

void PathFollower::updatePathFollowing()
{
  // 1. Calculate relative vehicle position
  const float cosYawRec = cos(vehicleRec_.yaw);
  const float sinYawRec = sin(vehicleRec_.yaw);

  const float vehicleXRel = cosYawRec * (vehicle_.x - vehicleRec_.x)
                          + sinYawRec * (vehicle_.y - vehicleRec_.y);
  const float vehicleYRel = -sinYawRec * (vehicle_.x - vehicleRec_.x)
                          + cosYawRec * (vehicle_.y - vehicleRec_.y);

  // 2. Find lookahead point and calculate distance to goal
  const int pathSize = static_cast<int>(pathState_.path.poses.size());

  // Distance to goal
  const float endDisX = pathState_.path.poses[pathSize - 1].pose.position.x - vehicleXRel;
  const float endDisY = pathState_.path.poses[pathSize - 1].pose.position.y - vehicleYRel;
  const float endDis = std::sqrt(endDisX * endDisX + endDisY * endDisY);

  // Advance to lookahead point
  float disX, disY, dis;
  while (pathState_.pointID < pathSize - 1) {
    disX = pathState_.path.poses[pathState_.pointID].pose.position.x - vehicleXRel;
    disY = pathState_.path.poses[pathState_.pointID].pose.position.y - vehicleYRel;
    dis = std::sqrt(disX * disX + disY * disY);
    if (dis < config_.lookAheadDis) {
      pathState_.pointID++;
    } else {
      break;
    }
  }

  // Get lookahead point distance and direction
  disX = pathState_.path.poses[pathState_.pointID].pose.position.x - vehicleXRel;
  disY = pathState_.path.poses[pathState_.pointID].pose.position.y - vehicleYRel;
  dis = std::sqrt(disX * disX + disY * disY);
  const float pathDir = std::atan2(disY, disX);

  // 3. Calculate direction difference
  float dirDiff = vehicle_.yaw - vehicleRec_.yaw - pathDir;
  dirDiff = normalizeAngle(dirDiff);

  // 4. Handle two-way drive
  if (config_.twoWayDrive) {
    const double currentTime = now().seconds();
    if (std::abs(dirDiff) > PI / 2.0f && pathState_.forward &&
        currentTime - pathState_.switchTime > config_.switchTimeThre) {
      pathState_.forward = false;
      pathState_.switchTime = currentTime;
    } else if (std::abs(dirDiff) < PI / 2.0f && !pathState_.forward &&
               currentTime - pathState_.switchTime > config_.switchTimeThre) {
      pathState_.forward = true;
      pathState_.switchTime = currentTime;
    }
  }

  // 5. Calculate target speed based on direction
  float targetSpeed = static_cast<float>(config_.maxSpeed);
  if (!pathState_.forward) {
    dirDiff = normalizeAngle(dirDiff + PI);
    targetSpeed *= -1.0f;
  }

  // 6. Calculate yaw rate
  const float maxAccelPerCycle = static_cast<float>(config_.maxAccel / 100.0);
  if (std::abs(vehicle_.speed) < 2.0f * maxAccelPerCycle) {
    vehicle_.yawRate = -static_cast<float>(config_.stopYawRateGain) * dirDiff;
  } else {
    vehicle_.yawRate = -static_cast<float>(config_.yawRateGain) * dirDiff;
  }

  // Limit yaw rate
  const float maxYawRateRad = static_cast<float>(config_.maxYawRate * DEG_TO_RAD);
  vehicle_.yawRate = std::clamp(vehicle_.yawRate, -maxYawRateRad, maxYawRateRad);

  // Disable rotation at goal
  if (pathSize <= 1 || (dis < config_.stopDisThre && config_.noRotAtGoal)) {
    vehicle_.yawRate = 0.0f;
  }

  // 7. Calculate final speed with goal proximity slowdown
  if (pathSize <= 1) {
    targetSpeed = 0.0f;
  } else if (endDis < config_.slowDwnDisThre) {
    targetSpeed *= (endDis / config_.slowDwnDisThre);
  }

  // Apply incline-based slowdown
  if (vehicle_.odomTime < pathState_.slowInitTime + config_.slowTime1 && pathState_.slowInitTime > 0) {
    targetSpeed *= static_cast<float>(config_.slowRate1);
  } else if (vehicle_.odomTime < pathState_.slowInitTime + config_.slowTime1 + config_.slowTime2 &&
             pathState_.slowInitTime > 0) {
    targetSpeed *= static_cast<float>(config_.slowRate2);
  }

  // 8. Apply acceleration limits
  applyAccelerationLimits(targetSpeed, dirDiff, dis, endDis);

  // 9. Disable rotation at high speed
  if (std::abs(vehicle_.speed) > config_.noRotSpeed) {
    vehicle_.yawRate = 0.0f;
  }

  // 10. Apply incline-based stop
  if (config_.useInclToStop && vehicle_.odomTime < pathState_.stopInitTime + config_.stopTime &&
      pathState_.stopInitTime > 0) {
    vehicle_.speed = 0.0f;
    vehicle_.yawRate = 0.0f;
  }

  // 11. Apply safety checks
  applySafetyChecks();

  // 12. Publish command
  pathState_.pubSkipCounter--;
  if (pathState_.pubSkipCounter < 0) {
    const float maxAccelPerCycle = static_cast<float>(config_.maxAccel / 100.0);
    float linearX, linearY;
    if (std::abs(vehicle_.speed) <= maxAccelPerCycle) {
      linearX = 0.0f;
      linearY = 0.0f;
    } else {
      linearX = cos(dirDiff) * vehicle_.speed;
      linearY = -sin(dirDiff) * vehicle_.speed;
    }
    publishCommand(linearX, linearY, vehicle_.yawRate);
    pathState_.pubSkipCounter = config_.pubSkipNum;
  }
}

float PathFollower::normalizeAngle(float angle)
{
  while (angle > PI) angle -= 2.0f * static_cast<float>(PI);
  while (angle < -PI) angle += 2.0f * static_cast<float>(PI);
  return angle;
}

void PathFollower::applyAccelerationLimits(float targetSpeed, float dirDiff, float dis, float endDis)
{
  const float maxAccelPerCycle = static_cast<float>(config_.maxAccel / 100.0);

  // Check if direction is acceptable for movement
  const bool directionOK = (std::abs(dirDiff) < config_.dirDiffThre) ||
                          (endDis < config_.goalCloseDis && std::abs(dirDiff) < config_.omniDirDiffThre);

  if (directionOK && dis > config_.stopDisThre) {
    // Accelerate or decelerate towards target speed
    if (vehicle_.speed < targetSpeed) {
      vehicle_.speed += maxAccelPerCycle;
    } else if (vehicle_.speed > targetSpeed) {
      vehicle_.speed -= maxAccelPerCycle;
    }
  } else {
    // Decelerate to stop
    if (vehicle_.speed > 0) {
      vehicle_.speed -= maxAccelPerCycle;
    } else if (vehicle_.speed < 0) {
      vehicle_.speed += maxAccelPerCycle;
    }
  }
}

void PathFollower::applySafetyChecks()
{
  // Safety stop bit flags:
  // bit 0 (1): stop forward motion
  // bit 1 (2): stop backward motion
  // bit 2 (4): stop positive rotation
  // bit 3 (8): stop negative rotation

  if ((pathState_.safetyStop & 1) && vehicle_.speed > 0) {
    vehicle_.speed = 0;
  }
  if ((pathState_.safetyStop & 2) && vehicle_.speed < 0) {
    vehicle_.speed = 0;
  }
  if ((pathState_.safetyStop & 4) && vehicle_.yawRate > 0) {
    vehicle_.yawRate = 0;
  }
  if ((pathState_.safetyStop & 8) && vehicle_.yawRate < 0) {
    vehicle_.yawRate = 0;
  }
}

void PathFollower::publishCommand(float linearX, float linearY, float angularZ)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = "vehicle";
  cmd_vel.header.stamp = rclcpp::Time(static_cast<uint64_t>(vehicle_.odomTime * 1e9));
  cmd_vel.twist.linear.x = linearX;
  cmd_vel.twist.linear.y = linearY;
  cmd_vel.twist.angular.z = angularZ;
  pub_cmd_vel_->publish(cmd_vel);

  // Note: Go2 command forwarding is now handled by vel_ctrl_repub node
  // which subscribes to /cmd_vel and forwards to /api/sport/request
}

}  // namespace path_follower

// ============================================================================
// Main Function
// ============================================================================

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<path_follower::PathFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
