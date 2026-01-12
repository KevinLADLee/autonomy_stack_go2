#include "local_planner/local_planner.hpp"
#include "local_planner/path_file_io.hpp"

namespace local_planner
{

using constants::kPi;

// Use PI constant for mathematical calculations
constexpr float PI = constants::kPi;

// ============================================================================
// PlannerConfig Implementation
// ============================================================================

void PlannerConfig::loadFromParameters(rclcpp::Node* node)
{
  node->get_parameter("pathFolder", pathFolder);

  node->get_parameter("vehicleLength", vehicleLength);
  node->get_parameter("vehicleWidth", vehicleWidth);
  node->get_parameter("sensorOffsetX", sensorOffsetX);
  node->get_parameter("sensorOffsetY", sensorOffsetY);
  node->get_parameter("twoWayDrive", twoWayDrive);

  node->get_parameter("laserVoxelSize", laserVoxelSize);
  node->get_parameter("terrainVoxelSize", terrainVoxelSize);
  node->get_parameter("useTerrainAnalysis", useTerrainAnalysis);

  node->get_parameter("checkObstacle", checkObstacle);
  node->get_parameter("checkRotObstacle", checkRotObstacle);
  node->get_parameter("adjacentRange", adjacentRange);
  node->get_parameter("obstacleHeightThre", obstacleHeightThre);
  node->get_parameter("groundHeightThre", groundHeightThre);
  node->get_parameter("costHeightThre", costHeightThre);
  node->get_parameter("costScore", costScore);
  node->get_parameter("useCost", useCost);

  node->get_parameter("pointPerPathThre", pointPerPathThre);
  node->get_parameter("minRelZ", minRelZ);
  node->get_parameter("maxRelZ", maxRelZ);
  node->get_parameter("maxSpeed", maxSpeed);
  node->get_parameter("dirWeight", dirWeight);
  node->get_parameter("dirThre", dirThre);
  node->get_parameter("dirToVehicle", dirToVehicle);
  node->get_parameter("pathScale", pathScale);
  node->get_parameter("minPathScale", minPathScale);
  node->get_parameter("pathScaleStep", pathScaleStep);
  node->get_parameter("pathScaleBySpeed", pathScaleBySpeed);
  node->get_parameter("minPathRange", minPathRange);
  node->get_parameter("pathRangeStep", pathRangeStep);
  node->get_parameter("pathRangeBySpeed", pathRangeBySpeed);
  node->get_parameter("pathCropByGoal", pathCropByGoal);

  node->get_parameter("goalCloseDis", goalCloseDis);
  node->get_parameter("goalClearRange", goalClearRange);
  node->get_parameter("goalX", goalX);
  node->get_parameter("goalY", goalY);
}

// ============================================================================
// PointCloudData Implementation
// ============================================================================

PointCloudData::PointCloudData()
  : laserCloud(new pcl::PointCloud<pcl::PointXYZI>())
  , laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>())
  , laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>())
  , terrainCloud(new pcl::PointCloud<pcl::PointXYZI>())
  , terrainCloudCrop(new pcl::PointCloud<pcl::PointXYZI>())
  , terrainCloudDwz(new pcl::PointCloud<pcl::PointXYZI>())
  , plannerCloud(new pcl::PointCloud<pcl::PointXYZI>())
  , plannerCloudCrop(new pcl::PointCloud<pcl::PointXYZI>())
  , boundaryCloud(new pcl::PointCloud<pcl::PointXYZI>())
  , addedObstacles(new pcl::PointCloud<pcl::PointXYZI>())
{
  laserCloudStack.resize(PathData::laserCloudStackNum);
  for (auto& cloud : laserCloudStack) {
    cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
}

// ============================================================================
// PathData Implementation
// ============================================================================

PathData::PathData()
{
#if PLOTPATHSET == 1
  freePaths.reset(new pcl::PointCloud<pcl::PointXYZI>());
#endif
}

void PathData::resizeArrays()
{
  pathList.resize(pathNum, 0);
  endDirPathList.resize(pathNum, 0.0f);
  clearPathList.resize(constants::kNumRotationDirections * pathNum, 0);
  pathPenaltyList.resize(constants::kNumRotationDirections * pathNum, 0.0f);
  clearPathPerGroupScore.resize(constants::kNumRotationDirections * groupNum, 0.0f);
  correspondences.resize(gridVoxelNum);

  startPaths.resize(groupNum);
  for (auto& path : startPaths) {
    path.reset(new pcl::PointCloud<pcl::PointXYZ>());
  }

  paths.resize(pathNum);
  for (auto& path : paths) {
    path.reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
}

bool PathData::readStartPaths(const std::string& pathFolder)
{
  return io::readStartPaths(pathFolder, startPaths, groupNum);
}

bool PathData::readPaths(const std::string& pathFolder)
{
#if PLOTPATHSET == 1
  return io::readPaths(pathFolder, paths, pathNum, constants::kPointSkipCount);
#else
  return true;
#endif
}

bool PathData::readPathList(const std::string& pathFolder)
{
  return io::readPathList(pathFolder, pathList, endDirPathList, pathNum, groupNum);
}

bool PathData::readCorrespondences(const std::string& pathFolder)
{
  return io::readCorrespondences(pathFolder, correspondences, gridVoxelNum, pathNum);
}

// ============================================================================
// LocalPlanner Implementation
// ============================================================================

LocalPlanner::LocalPlanner(const rclcpp::NodeOptions& options)
  : rclcpp::Node("local_planner", options)
{
  RCLCPP_INFO(get_logger(), "Initializing LocalPlanner...");

  declareParameters();
  loadParameters();

  clouds_ = PointCloudData();
  pathData_ = PathData();
  pathData_.resizeArrays();

  initializeFilters();

  if (!loadPathFiles()) {
    RCLCPP_ERROR(get_logger(), "Failed to load path files");
    return;
  }

  initializeROSInterfaces();

  timer_ = create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&LocalPlanner::timerCallback, this));

  RCLCPP_INFO(get_logger(), "LocalPlanner initialized successfully");
}

void LocalPlanner::declareParameters()
{
  declare_parameter<std::string>("pathFolder", config_.pathFolder);

  declare_parameter<double>("vehicleLength", config_.vehicleLength);
  declare_parameter<double>("vehicleWidth", config_.vehicleWidth);
  declare_parameter<double>("sensorOffsetX", config_.sensorOffsetX);
  declare_parameter<double>("sensorOffsetY", config_.sensorOffsetY);
  declare_parameter<bool>("twoWayDrive", config_.twoWayDrive);

  declare_parameter<double>("laserVoxelSize", config_.laserVoxelSize);
  declare_parameter<double>("terrainVoxelSize", config_.terrainVoxelSize);
  declare_parameter<bool>("useTerrainAnalysis", config_.useTerrainAnalysis);

  declare_parameter<bool>("checkObstacle", config_.checkObstacle);
  declare_parameter<bool>("checkRotObstacle", config_.checkRotObstacle);
  declare_parameter<double>("adjacentRange", config_.adjacentRange);
  declare_parameter<double>("obstacleHeightThre", config_.obstacleHeightThre);
  declare_parameter<double>("groundHeightThre", config_.groundHeightThre);
  declare_parameter<double>("costHeightThre", config_.costHeightThre);
  declare_parameter<double>("costScore", config_.costScore);
  declare_parameter<bool>("useCost", config_.useCost);

  declare_parameter<int>("pointPerPathThre", config_.pointPerPathThre);
  declare_parameter<double>("minRelZ", config_.minRelZ);
  declare_parameter<double>("maxRelZ", config_.maxRelZ);
  declare_parameter<double>("maxSpeed", config_.maxSpeed);
  declare_parameter<double>("dirWeight", config_.dirWeight);
  declare_parameter<double>("dirThre", config_.dirThre);
  declare_parameter<bool>("dirToVehicle", config_.dirToVehicle);
  declare_parameter<double>("pathScale", config_.pathScale);
  declare_parameter<double>("minPathScale", config_.minPathScale);
  declare_parameter<double>("pathScaleStep", config_.pathScaleStep);
  declare_parameter<bool>("pathScaleBySpeed", config_.pathScaleBySpeed);
  declare_parameter<double>("minPathRange", config_.minPathRange);
  declare_parameter<double>("pathRangeStep", config_.pathRangeStep);
  declare_parameter<bool>("pathRangeBySpeed", config_.pathRangeBySpeed);
  declare_parameter<bool>("pathCropByGoal", config_.pathCropByGoal);

  declare_parameter<double>("goalCloseDis", config_.goalCloseDis);
  declare_parameter<double>("goalClearRange", config_.goalClearRange);
  declare_parameter<double>("goalX", config_.goalX);
  declare_parameter<double>("goalY", config_.goalY);
}

void LocalPlanner::loadParameters()
{
  config_.loadFromParameters(this);
  state_.normalizedSpeed = 0.0f;
  state_.targetDirection = 0.0f;
  state_.hasValidGoal = false;
}

void LocalPlanner::initializeFilters()
{
  clouds_.laserDwzFilter.setLeafSize(
    config_.laserVoxelSize, config_.laserVoxelSize, config_.laserVoxelSize);
  clouds_.terrainDwzFilter.setLeafSize(
    config_.terrainVoxelSize, config_.terrainVoxelSize, config_.terrainVoxelSize);
}

bool LocalPlanner::loadPathFiles()
{
  RCLCPP_INFO(get_logger(), "Reading path files from: %s", config_.pathFolder.c_str());

  if (!pathData_.readStartPaths(config_.pathFolder)) {
    RCLCPP_ERROR(get_logger(), "Failed to read start paths");
    return false;
  }

#if PLOTPATHSET == 1
  if (!pathData_.readPaths(config_.pathFolder)) {
    RCLCPP_ERROR(get_logger(), "Failed to read paths");
    return false;
  }
#endif

  if (!pathData_.readPathList(config_.pathFolder)) {
    RCLCPP_ERROR(get_logger(), "Failed to read path list");
    return false;
  }

  if (!pathData_.readCorrespondences(config_.pathFolder)) {
    RCLCPP_ERROR(get_logger(), "Failed to read correspondences");
    return false;
  }

  RCLCPP_INFO(get_logger(), "Path files loaded successfully");
  return true;
}

void LocalPlanner::initializeROSInterfaces()
{
  // Initialize subscribers
  sub_odometry_ = create_subscription<nav_msgs::msg::Odometry>(
    "/state_estimation", 5,
    std::bind(&LocalPlanner::odometryCallback, this, std::placeholders::_1));

  sub_laser_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/registered_scan", 5,
    std::bind(&LocalPlanner::laserCloudCallback, this, std::placeholders::_1));

  sub_terrain_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/terrain_map", 5,
    std::bind(&LocalPlanner::terrainCloudCallback, this, std::placeholders::_1));

  sub_goal_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 1,
    std::bind(&LocalPlanner::goalPoseCallback, this, std::placeholders::_1));

  sub_boundary_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
    "/navigation_boundary", 5,
    std::bind(&LocalPlanner::boundaryCallback, this, std::placeholders::_1));

  sub_added_obstacles_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/added_obstacles", 5,
    std::bind(&LocalPlanner::addedObstaclesCallback, this, std::placeholders::_1));

  sub_check_obstacle_ = create_subscription<std_msgs::msg::Bool>(
    "/check_obstacle", 5,
    std::bind(&LocalPlanner::checkObstacleCallback, this, std::placeholders::_1));

  // Initialize publishers
  pub_path_ = create_publisher<nav_msgs::msg::Path>("/autonomy_stack/path", 5);

#if PLOTPATHSET == 1
  pub_free_paths_ = create_publisher<sensor_msgs::msg::PointCloud2>("/autonomy_stack/free_paths", 2);
#endif
}

// ============================================================================
// ROS2 Callbacks
// ============================================================================

void LocalPlanner::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  state_.odomTime = rclcpp::Time(msg->header.stamp).seconds();

  double roll, pitch, yaw;
  const auto& quat = msg->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(quat.x, quat.y, quat.z, quat.w)).getRPY(roll, pitch, yaw);

  state_.roll = static_cast<float>(roll);
  state_.pitch = static_cast<float>(pitch);
  state_.yaw = static_cast<float>(yaw);
  state_.x = static_cast<float>(
    msg->pose.pose.position.x - std::cos(yaw) * config_.sensorOffsetX +
    std::sin(yaw) * config_.sensorOffsetY);
  state_.y = static_cast<float>(
    msg->pose.pose.position.y - std::sin(yaw) * config_.sensorOffsetX -
    std::cos(yaw) * config_.sensorOffsetY);
  state_.z = static_cast<float>(msg->pose.pose.position.z);
}

void LocalPlanner::laserCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (config_.useTerrainAnalysis) {
    return;
  }

  clouds_.laserCloud->clear();
  pcl::fromROSMsg(*msg, *clouds_.laserCloud);

  clouds_.laserCloudCrop->clear();
  for (const auto& pt : clouds_.laserCloud->points) {
    float dis = std::hypot(pt.x - state_.x, pt.y - state_.y);
    if (dis < config_.adjacentRange) {
      pcl::PointXYZI point;
      point.x = pt.x;
      point.y = pt.y;
      point.z = pt.z;
      point.intensity = pt.intensity;
      clouds_.laserCloudCrop->push_back(point);
    }
  }

  clouds_.laserCloudDwz->clear();
  clouds_.laserDwzFilter.setInputCloud(clouds_.laserCloudCrop);
  clouds_.laserDwzFilter.filter(*clouds_.laserCloudDwz);

  clouds_.newLaserCloud = true;
}

void LocalPlanner::terrainCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!config_.useTerrainAnalysis) {
    return;
  }

  clouds_.terrainCloud->clear();
  pcl::fromROSMsg(*msg, *clouds_.terrainCloud);

  clouds_.terrainCloudCrop->clear();
  for (const auto& pt : clouds_.terrainCloud->points) {
    float dis = std::hypot(pt.x - state_.x, pt.y - state_.y);
    if (dis < config_.adjacentRange && (pt.intensity > config_.obstacleHeightThre || config_.useCost)) {
      pcl::PointXYZI point;
      point.x = pt.x;
      point.y = pt.y;
      point.z = pt.z;
      point.intensity = pt.intensity;
      clouds_.terrainCloudCrop->push_back(point);
    }
  }

  clouds_.terrainCloudDwz->clear();
  clouds_.terrainDwzFilter.setInputCloud(clouds_.terrainCloudCrop);
  clouds_.terrainDwzFilter.filter(*clouds_.terrainCloudDwz);

  clouds_.newTerrainCloud = true;
}

void LocalPlanner::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  config_.goalX = msg->pose.position.x;
  config_.goalY = msg->pose.position.y;
  updateTargetFromGoal();

  RCLCPP_INFO(get_logger(), "Received goal_pose: x=%.2f, y=%.2f", config_.goalX, config_.goalY);
}

void LocalPlanner::boundaryCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  clouds_.boundaryCloud->clear();

  const int boundarySize = static_cast<int>(msg->polygon.points.size());
  if (boundarySize < 1) {
    return;
  }

  pcl::PointXYZI point1;
  pcl::PointXYZI point2;
  point2.x = msg->polygon.points[0].x;
  point2.y = msg->polygon.points[0].y;
  point2.z = msg->polygon.points[0].z;

  for (int i = 0; i < boundarySize; i++) {
    point1 = point2;
    point2.x = msg->polygon.points[i].x;
    point2.y = msg->polygon.points[i].y;
    point2.z = msg->polygon.points[i].z;

    if (point1.z != point2.z) {
      continue;
    }

    float disX = point1.x - point2.x;
    float disY = point1.y - point2.y;
    float dis = std::hypot(disX, disY);

    int pointNum = static_cast<int>(dis / config_.terrainVoxelSize) + 1;
    for (int pointID = 0; pointID < pointNum; pointID++) {
      float ratio = static_cast<float>(pointID) / static_cast<float>(pointNum);
      pcl::PointXYZI point;
      point.x = ratio * point1.x + (1.0f - ratio) * point2.x;
      point.y = ratio * point1.y + (1.0f - ratio) * point2.y;
      point.z = 0.0f;
      point.intensity = 100.0f;

      for (int j = 0; j < config_.pointPerPathThre; j++) {
        clouds_.boundaryCloud->push_back(point);
      }
    }
  }
}

void LocalPlanner::addedObstaclesCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  clouds_.addedObstacles->clear();
  pcl::fromROSMsg(*msg, *clouds_.addedObstacles);

  for (auto& pt : clouds_.addedObstacles->points) {
    pt.intensity = 200.0f;
  }
}

void LocalPlanner::checkObstacleCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  config_.checkObstacle = msg->data;
}

// ============================================================================
// Timer Callback - Main Processing Loop
// ============================================================================

void LocalPlanner::timerCallback()
{
  if (clouds_.newLaserCloud || clouds_.newTerrainCloud) {
    processPointClouds();
  }

  if (clouds_.plannerCloud->points.size() > 0) {
    transformAndCropClouds();
  }

  if (state_.hasValidGoal && clouds_.plannerCloudCrop->points.size() > 0) {
    nav_msgs::msg::Path path;
    bool pathFound = findPath(path);

    if (pathFound && state_.normalizedSpeed > 0.0f) {
      publishPath(path);
    }
    publishFreePaths();
  }
}

// ============================================================================
// Point Cloud Processing
// ============================================================================

void LocalPlanner::processPointClouds()
{
  if (clouds_.newLaserCloud) {
    clouds_.newLaserCloud = false;

    clouds_.laserCloudStack[clouds_.laserCloudCount]->clear();
    *clouds_.laserCloudStack[clouds_.laserCloudCount] = *clouds_.laserCloudDwz;
    clouds_.laserCloudCount = (clouds_.laserCloudCount + 1) % PathData::laserCloudStackNum;

    clouds_.plannerCloud->clear();
    for (const auto& cloud : clouds_.laserCloudStack) {
      *clouds_.plannerCloud += *cloud;
    }
  }

  if (clouds_.newTerrainCloud) {
    clouds_.newTerrainCloud = false;
    pcl::copyPointCloud(*clouds_.terrainCloudDwz, *clouds_.plannerCloud);
  }
}

void LocalPlanner::transformPointCloudToVehicleFrame(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud,
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& outputCloud,
  bool checkRangeAndZ,
  bool clearOutput) const
{
  float sinYaw = std::sin(state_.yaw);
  float cosYaw = std::cos(state_.yaw);

  if (clearOutput) {
    outputCloud->clear();
  }

  for (const auto& pt : inputCloud->points) {
    float dx = pt.x - state_.x;
    float dy = pt.y - state_.y;
    float dz = pt.z - state_.z;

    pcl::PointXYZI point;
    point.x = dx * cosYaw + dy * sinYaw;
    point.y = -dx * sinYaw + dy * cosYaw;
    point.z = dz;
    point.intensity = pt.intensity;

    float dis = std::hypot(point.x, point.y);

    if (checkRangeAndZ) {
      bool inRange = dis < config_.adjacentRange;
      bool inZRange = (point.z > config_.minRelZ && point.z < config_.maxRelZ) || config_.useTerrainAnalysis;
      if (inRange && inZRange) {
        outputCloud->push_back(point);
      }
    } else {
      if (dis < config_.adjacentRange) {
        outputCloud->push_back(point);
      }
    }
  }
}

void LocalPlanner::transformAndCropClouds()
{
  clouds_.plannerCloudCrop->clear();
  transformPointCloudToVehicleFrame(clouds_.plannerCloud, clouds_.plannerCloudCrop, true, true);
  transformPointCloudToVehicleFrame(clouds_.boundaryCloud, clouds_.plannerCloudCrop, false, false);
  transformPointCloudToVehicleFrame(clouds_.addedObstacles, clouds_.plannerCloudCrop, false, false);
}

void LocalPlanner::publishPath(const nav_msgs::msg::Path& path)
{
  pub_path_->publish(path);
}

void LocalPlanner::publishFreePaths()
{
#if PLOTPATHSET == 1
  sensor_msgs::msg::PointCloud2 freePathsMsg;
  pcl::toROSMsg(*pathData_.freePaths, freePathsMsg);
  freePathsMsg.header.stamp = rclcpp::Time(static_cast<uint64_t>(state_.odomTime * 1e9));
  freePathsMsg.header.frame_id = "vehicle";
  pub_free_paths_->publish(freePathsMsg);
#endif
}

// ============================================================================
// 辅助方法
// ============================================================================

void LocalPlanner::updateTargetFromGoal()
{
  // Calculate relative goal position
  float sinYaw = std::sin(state_.yaw);
  float cosYaw = std::cos(state_.yaw);

  float relX = static_cast<float>((config_.goalX - state_.x) * cosYaw + (config_.goalY - state_.y) * sinYaw);
  float relY = static_cast<float>(-(config_.goalX - state_.x) * sinYaw + (config_.goalY - state_.y) * cosYaw);
  float distance = std::hypot(relX, relY);

  // Update goal validity
  state_.hasValidGoal = (distance > 0.1f);

  if (state_.hasValidGoal) {
    // Calculate target direction
    state_.targetDirection = normalizeAngle(static_cast<float>(std::atan2(relY, relX) * 180.0 / PI));

    RCLCPP_DEBUG(this->get_logger(), "updateTargetFromGoal: goalX=%.2f, goalY=%.2f, relX=%.2f, relY=%.2f, targetDirection=%.2f",
                 config_.goalX, config_.goalY, relX, relY, state_.targetDirection);

    // If not two-way drive, limit to forward range
    if (!config_.twoWayDrive) {
      state_.targetDirection = std::clamp(state_.targetDirection, -90.0f, 90.0f);
    }

    // Use full maxSpeed
    state_.normalizedSpeed = 1.0f;
  } else {
    state_.normalizedSpeed = 0.0f;
  }
}

void LocalPlanner::updatePathScaleAndRange(float& pathScale, float& pathRange)
{
  if (config_.pathRangeBySpeed) {
    pathRange = static_cast<float>(config_.adjacentRange * state_.normalizedSpeed);
  }
  if (pathRange < config_.minPathRange) {
    pathRange = static_cast<float>(config_.minPathRange);
  }

  if (config_.pathScaleBySpeed) {
    pathScale = static_cast<float>(config_.pathScale * state_.normalizedSpeed);
  }
  if (pathScale < config_.minPathScale) {
    pathScale = static_cast<float>(config_.minPathScale);
  }
}

// ============================================================================
// Utility Methods
// ============================================================================

float LocalPlanner::computeAngleDifference(float angle1, float angle2) const
{
  float diff = std::fabs(angle1 - angle2);
  if (diff > constants::kMaxAngleDegrees) {
    diff = constants::kFullCircleDegrees - diff;
  }
  return diff;
}

float LocalPlanner::normalizeAngle(float angle) const
{
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

bool LocalPlanner::isDirectionValid(int rotDir) const
{
  // Convert rotation direction index to angle (-180 to 170 degrees)
  float rotAngRaw = 10.0f * rotDir - 180.0f;
  
  // Calculate angular difference between target and this rotation direction
  float angDiff = std::fabs(state_.targetDirection - rotAngRaw);
  if (angDiff > 180.0f) {
    angDiff = 360.0f - angDiff;
  }

  // Mode 1: Direction filtering relative to goal
  // Check if direction is within threshold of target direction
  if (!config_.dirToVehicle) {
    return angDiff <= config_.dirThre;
  }
  
  // Mode 2: Direction filtering relative to vehicle
  // For forward-facing targets (within ±90°), allow forward-ish rotations
  if (std::fabs(state_.targetDirection) <= 90.0f) {
    return std::fabs(rotAngRaw) <= config_.dirThre;
  }
  
  // For backward-facing targets (beyond ±90°), allow backward-ish rotations
  // Check if rotation is in backward range (beyond dirThre from both ends)
  bool isBackwardRotation = (rotAngRaw < -config_.dirThre) || (rotAngRaw > config_.dirThre);
  return isBackwardRotation;
}

bool LocalPlanner::isRotationValid(float rotAng, float rotDeg, float minObsAngCW, float minObsAngCCW) const
{
  bool rotAngOK = (rotAng * 180.0f / PI > minObsAngCW && rotAng * 180.0f / PI < minObsAngCCW);
  bool rotDegOK = (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && config_.twoWayDrive);
  return rotAngOK || rotDegOK || !config_.checkRotObstacle;
}

// ============================================================================
// Goal Calculation
// ============================================================================

void LocalPlanner::calculateRelativeGoal(
  float& relativeGoalX,
  float& relativeGoalY,
  float& relativeGoalDis,
  float& desiredDirection
) const
{
  // Transform goal to vehicle frame
  float sinYaw = std::sin(state_.yaw);
  float cosYaw = std::cos(state_.yaw);

  relativeGoalX = static_cast<float>((config_.goalX - state_.x) * cosYaw + (config_.goalY - state_.y) * sinYaw);
  relativeGoalY = static_cast<float>(-(config_.goalX - state_.x) * sinYaw + (config_.goalY - state_.y) * cosYaw);
  relativeGoalDis = std::hypot(relativeGoalX, relativeGoalY);

  // Calculate desired direction
  if (relativeGoalDis > 0.1f) {
    desiredDirection = normalizeAngle(std::atan2(relativeGoalY, relativeGoalX) * 180.0f / PI);
  } else {
    desiredDirection = 0.0f;
  }

  // Use target direction from state (computed by updateTargetFromGoal)
  if (state_.hasValidGoal) {
    desiredDirection = state_.targetDirection;
  }
}

// ============================================================================
// Obstacle Detection
// ============================================================================

void LocalPlanner::initializeScoringArrays()
{
  // Reset scoring arrays
  std::fill(pathData_.clearPathList.begin(), pathData_.clearPathList.end(), 0);
  std::fill(pathData_.pathPenaltyList.begin(), pathData_.pathPenaltyList.end(), 0.0f);
  std::fill(pathData_.clearPathPerGroupScore.begin(), pathData_.clearPathPerGroupScore.end(), 0.0f);
}

void LocalPlanner::checkPointAgainstPaths(
  const pcl::PointXYZI& point,
  float pathScale,
  float pathRange,
  float relativeGoalDis,
  float& minObsAngCW,
  float& minObsAngCCW
)
{
  // Scale point coordinates
  float x = point.x / pathScale;
  float y = point.y / pathScale;
  float h = point.intensity;
  float dis = std::hypot(x, y);

  // Check if within planning range and goal distance
  bool withinRange = dis < pathRange / pathScale;
  bool withinGoalRange = !config_.pathCropByGoal || dis <= (relativeGoalDis + config_.goalClearRange) / pathScale;

  if (!withinRange || !withinGoalRange || !config_.checkObstacle) {
    return;
  }

  // Check each rotation direction (exactly 36 as in original code)
  for (int rotDir = 0; rotDir < 36; rotDir++) {
    if (!isDirectionValid(rotDir)) {
      continue;
    }

    float rotAngRaw = 10.0f * rotDir - 180.0f;
    float rotAng = rotAngRaw * PI / 180.0f;
    float x2 = std::cos(rotAng) * x + std::sin(rotAng) * y;
    float y2 = -std::sin(rotAng) * x + std::cos(rotAng) * y;

    // Calculate scaleY for grid mapping (from original code)
    float scaleY = x2 / pathData_.gridVoxelOffsetX + pathData_.searchRadius / pathData_.gridVoxelOffsetY
                   * (pathData_.gridVoxelOffsetX - x2) / pathData_.gridVoxelOffsetX;

    // Map to grid voxel (using original indexing: gridVoxelNumY * indX + indY)
    int indX = static_cast<int>((pathData_.gridVoxelOffsetX + pathData_.gridVoxelSize / 2.0f - x2) / pathData_.gridVoxelSize);
    int indY = static_cast<int>((pathData_.gridVoxelOffsetY + pathData_.gridVoxelSize / 2.0f - y2 / scaleY) / pathData_.gridVoxelSize);

    if (indX >= 0 && indX < PathData::gridVoxelNumX && indY >= 0 && indY < PathData::gridVoxelNumY) {
      int ind = PathData::gridVoxelNumY * indX + indY;

      // Mark blocked paths
      for (int pathID : pathData_.correspondences[ind]) {
        int pathInd = PathData::pathNum * rotDir + pathID;
        if (h > config_.obstacleHeightThre || !config_.useTerrainAnalysis) {
          pathData_.clearPathList[pathInd]++;
        } else {
          if (pathData_.pathPenaltyList[pathInd] < h && h > config_.groundHeightThre) {
            pathData_.pathPenaltyList[pathInd] = h;
          }
        }
      }
    }
  }

  // Check for rotational obstacles
  float diameter = std::hypot(static_cast<float>(config_.vehicleLength) / 2.0f, static_cast<float>(config_.vehicleWidth) / 2.0f);
  float angOffset = std::atan2(config_.vehicleWidth, config_.vehicleLength) * 180.0f / PI;

  if (dis < diameter / pathScale && config_.checkRotObstacle &&
      (std::fabs(x) > config_.vehicleLength / pathScale / 2.0f || std::fabs(y) > config_.vehicleWidth / pathScale / 2.0f) &&
      (h > config_.obstacleHeightThre || !config_.useTerrainAnalysis)) {

    float pointAng = std::atan2(y, x) * 180.0f / PI;
    if (pointAng > 0.0f) {
      if (minObsAngCCW > pointAng - angOffset) minObsAngCCW = pointAng - angOffset;
      if (minObsAngCW < pointAng + angOffset - 180.0f) minObsAngCW = pointAng + angOffset - 180.0f;
    } else {
      if (minObsAngCW < pointAng + angOffset) minObsAngCW = pointAng + angOffset;
      if (minObsAngCCW > 180.0f + pointAng - angOffset) minObsAngCCW = 180.0f + pointAng - angOffset;
    }
  }
}

// ============================================================================
// Path Scoring
// ============================================================================

void LocalPlanner::scoreAllPaths(float desiredDir, float relativeGoalDis)
{
  // Score all paths and accumulate by group
  int validCount = 0;
  int blockedCount = 0;
  int directionInvalidCount = 0;

  for (int i = 0; i < 36 * PathData::pathNum; i++) {
    int rotDir = i / PathData::pathNum;

    if (!isDirectionValid(rotDir)) {
      directionInvalidCount++;
      continue;
    }

    // Skip if path is blocked
    if (pathData_.clearPathList[i] >= config_.pointPerPathThre) {
      blockedCount++;
      continue;
    }

    validCount++;

    // Calculate penalty score based on terrain cost
    float penaltyScore = 1.0f - pathData_.pathPenaltyList[i] / static_cast<float>(config_.costHeightThre);
    if (penaltyScore < config_.costScore) penaltyScore = static_cast<float>(config_.costScore);

    // Calculate direction difference (normalized to 0-180 degrees)
    float dirDiff = std::fabs(state_.targetDirection - pathData_.endDirPathList[i % PathData::pathNum] - (10.0f * rotDir - 180.0f));
    if (dirDiff > 360.0f) {
      dirDiff -= 360.0f;
    }
    if (dirDiff > 180.0f) {
      dirDiff = 360.0f - dirDiff;
    }
    
    // Normalize direction difference to [0, 1]
    float normalizedDirDiff = dirDiff / 180.0f;

    // Calculate rotation direction weight (prefer forward directions)
    // Reduced from power of 4 to power of 2 for less extreme weighting
    float rotDirW;
    if (rotDir < 18) rotDirW = std::fabs(std::fabs(rotDir - 9.0f) + 1.0f);
    else rotDirW = std::fabs(std::fabs(rotDir - 27.0f) + 1.0f);

    // Calculate group direction weight (prefer straighter paths)
    float groupDirW = 4.0f - std::fabs(pathData_.pathList[i % PathData::pathNum] - 3.0f);

    // Calculate direction score component with improved weighting
    // Using normalized difference and reduced sqrt nesting
    float dirScore = 1.0f - std::sqrt(config_.dirWeight * normalizedDirDiff);
    dirScore = std::max(0.0f, dirScore);  // Ensure non-negative

    // Calculate final score with reduced rotDirW power (2 instead of 4)
    // This makes path selection less extreme and more balanced
    float score = dirScore * rotDirW * rotDirW * penaltyScore;

    // Adjust for goal proximity - use group weight instead of rotation weight
    if (relativeGoalDis < config_.goalCloseDis) {
      score = dirScore * groupDirW * groupDirW * penaltyScore;
    }

    // Accumulate by group
    if (score > 0.0f) {
      pathData_.clearPathPerGroupScore[PathData::groupNum * rotDir + pathData_.pathList[i % PathData::pathNum]] += score;
    }
  }

  // Log scoring statistics (debug level)
  RCLCPP_DEBUG(this->get_logger(), "scoreAllPaths: total=%d, directionInvalid=%d, blocked=%d, valid=%d",
    36 * PathData::pathNum, directionInvalidCount, blockedCount, validCount);
}

// ============================================================================
// Path Selection
// ============================================================================

int LocalPlanner::selectBestPathGroup(float& maxScore, float minObsAngCW, float minObsAngCCW) const
{
  maxScore = 0.0f;
  int selectedGroupID = -1;

  // Select best path group (exactly from original code)
  for (int i = 0; i < 36 * PathData::groupNum; i++) {
    int rotDir = i / PathData::groupNum;
    float rotAng = (10.0f * rotDir - 180.0f) * PI / 180.0f;
    float rotDeg = 10.0f * rotDir;
    if (rotDeg > 180.0f) rotDeg -= 360.0f;

    if (pathData_.clearPathPerGroupScore[i] > maxScore &&
        isRotationValid(rotAng, rotDeg, minObsAngCW, minObsAngCCW)) {
      maxScore = pathData_.clearPathPerGroupScore[i];
      selectedGroupID = i;
    }
  }

  return selectedGroupID;
}

// ============================================================================
// Path Generation
// ============================================================================

bool LocalPlanner::generateSelectedPath(
  int groupID,
  int rotDir,
  float pathScale,
  float pathRange,
  float relativeGoalDis,
  nav_msgs::msg::Path& path
)
{
  // Get rotation parameters
  float rotAng = (constants::kRotationDegreeIncrement * rotDir - 180.0f) * PI / 180.0f;

  // Pre-allocate path poses (from original code line 870)
  int selectedPathLength = pathData_.startPaths[groupID]->points.size();
  path.poses.resize(selectedPathLength);

  // Transform start path points (from original code line 871-885)
  for (int i = 0; i < selectedPathLength; i++) {
    float x = pathData_.startPaths[groupID]->points[i].x;
    float y = pathData_.startPaths[groupID]->points[i].y;
    float z = pathData_.startPaths[groupID]->points[i].z;
    float dis = std::hypot(x, y);

    // Check if point is within range and goal distance (line 877)
    if (dis <= pathRange / pathScale && dis <= relativeGoalDis / pathScale) {
      path.poses[i].pose.position.x = pathScale * (std::cos(rotAng) * x - std::sin(rotAng) * y);
      path.poses[i].pose.position.y = pathScale * (std::sin(rotAng) * x + std::cos(rotAng) * y);
      path.poses[i].pose.position.z = pathScale * z;
    } else {
      // Resize and break (line 882-883)
      path.poses.resize(i);
      break;
    }
  }

  // Ensure all poses have frame_id set (line 887-889)
  for (size_t i = 0; i < path.poses.size(); i++) {
    path.poses[i].header.frame_id = "vehicle";
  }

  // Set header (line 891-892)
  path.header.stamp = rclcpp::Time(static_cast<uint64_t>(state_.odomTime * 1e9));
  path.header.frame_id = "vehicle";

  return !path.poses.empty();
}

void LocalPlanner::generateFreePathsVisualization(
  float pathScale,
  float pathRange,
  float relativeGoalDis,
  float minObsAngCW,
  float minObsAngCCW
)
{
#if PLOTPATHSET == 1
  pathData_.freePaths->clear();

  // Generate free paths visualization (exactly from original code line 896-933)
  for (int i = 0; i < 36 * PathData::pathNum; i++) {
    int rotDir = i / PathData::pathNum;
    int pathID = i % PathData::pathNum;

    if (!isDirectionValid(rotDir)) {
      continue;
    }

    float rotAng = (10.0f * rotDir - 180.0f) * PI / 180.0f;
    float rotDeg = 10.0f * rotDir;
    if (rotDeg > 180.0f) rotDeg -= 360.0f;

    if (!isRotationValid(rotAng, rotDeg, minObsAngCW, minObsAngCCW)) {
      continue;
    }

    // Check if path is blocked
    if (pathData_.clearPathList[i] >= config_.pointPerPathThre) {
      continue;
    }

    // Transform path points for visualization (exactly from original code line 915-930)
    for (const auto& point : pathData_.paths[pathID]->points) {
      float x = point.x;
      float y = point.y;
      float z = point.z;

      float dis = std::hypot(x, y);
      if (dis <= pathRange / pathScale &&
          (dis <= (relativeGoalDis + config_.goalClearRange) / pathScale || !config_.pathCropByGoal)) {

        pcl::PointXYZI newPoint;
        newPoint.x = pathScale * (std::cos(rotAng) * x - std::sin(rotAng) * y);
        newPoint.y = pathScale * (std::sin(rotAng) * x + std::cos(rotAng) * y);
        newPoint.z = pathScale * z;
        newPoint.intensity = 1.0f;

        pathData_.freePaths->push_back(newPoint);
      }
    }
  }
#endif
}

// ============================================================================
// Main Search Loop
// ============================================================================

bool LocalPlanner::findPathWithMultiScaleSearch(nav_msgs::msg::Path& path)
{
  // Update target state (computes hasValidGoal, targetDirection)
  updateTargetFromGoal();

  if (!state_.hasValidGoal) {
    return false;
  }

  // Calculate path parameters - use full speed (normalizedSpeed = 1.0)
  float pathRange = static_cast<float>(config_.adjacentRange);
  if (config_.pathRangeBySpeed) {
    pathRange *= 1.0f;  // Use full speed
  }
  pathRange = std::max(pathRange, static_cast<float>(config_.minPathRange));

  // Calculate path scale
  float pathScale = static_cast<float>(config_.pathScale);
  float defPathScale = pathScale;
  if (config_.pathScaleBySpeed) {
    pathScale *= 1.0f;  // Use full speed
  }
  pathScale = std::max(pathScale, static_cast<float>(config_.minPathScale));

  bool pathFound = false;
  float relativeGoalDis;
  
  // Store parameters for visualization (use last successful search if no path found)
  float visPathScale = pathScale;
  float visPathRange = pathRange;
  float visRelativeGoalDis = 0.0f;
  float visMinObsAngCW = -180.0f;
  float visMinObsAngCCW = 180.0f;
  
  // Track best result across all scales for fallback
  float bestMaxScore = 0.0f;
  int bestSelectedGroupInd = -1;
  float bestPathScale = pathScale;
  float bestPathRange = pathRange;
  float bestRelativeGoalDis = 0.0f;
  
  // Threshold for early exit - if we find a path with score above this, stop searching
  constexpr float GOOD_SCORE_THRESHOLD = 0.8f;

  // Multi-scale search loop
  int searchIteration = 0;
  while (pathScale >= config_.minPathScale && pathRange >= config_.minPathRange) {
    searchIteration++;
    
    // RECALCULATE relative goal distance INSIDE the loop (to account for robot movement)
    float sinYaw = std::sin(state_.yaw);
    float cosYaw = std::cos(state_.yaw);
    float relGoalX = static_cast<float>((config_.goalX - state_.x) * cosYaw + (config_.goalY - state_.y) * sinYaw);
    float relGoalY = static_cast<float>(-(config_.goalX - state_.x) * sinYaw + (config_.goalY - state_.y) * cosYaw);
    relativeGoalDis = std::hypot(relGoalX, relGoalY);

    // Calculate direction to goal (inside loop to use current robot position)
    float goalDirection = std::atan2(relGoalY, relGoalX) * 180.0f / PI;

    // Apply two-way drive constraints
    if (!config_.twoWayDrive) {
      goalDirection = std::clamp(goalDirection, -90.0f, 90.0f);
    }

    // Update state target direction for scoring
    state_.targetDirection = normalizeAngle(goalDirection);

    // Initialize scoring arrays
    initializeScoringArrays();

    // Detect obstacles
    float minObsAngCW = -180.0f, minObsAngCCW = 180.0f;

    for (const auto& point : clouds_.plannerCloudCrop->points) {
      checkPointAgainstPaths(point, pathScale, pathRange, relativeGoalDis,
                            minObsAngCW, minObsAngCCW);
    }

    // Normalize obstacle angles
    if (minObsAngCW > 0.0f) minObsAngCW = 0.0f;
    if (minObsAngCCW < 0.0f) minObsAngCCW = 0.0f;

    // Score all paths
    scoreAllPaths(goalDirection, relativeGoalDis);

    // Select best path
    float maxScore;
    int selectedGroupInd = selectBestPathGroup(maxScore, minObsAngCW, minObsAngCCW);

    // Save current search parameters for visualization
    visPathScale = pathScale;
    visPathRange = pathRange;
    visRelativeGoalDis = relativeGoalDis;
    visMinObsAngCW = minObsAngCW;
    visMinObsAngCCW = minObsAngCCW;
    
    // Track best result across all iterations
    if (maxScore > bestMaxScore) {
      bestMaxScore = maxScore;
      bestSelectedGroupInd = selectedGroupInd;
      bestPathScale = pathScale;
      bestPathRange = pathRange;
      bestRelativeGoalDis = relativeGoalDis;
    }

    if (selectedGroupInd >= 0 && maxScore > 0.0f) {
      // Extract group ID and rotation direction
      int groupID = selectedGroupInd % PathData::groupNum;
      int rotDir = selectedGroupInd / PathData::groupNum;

      // Generate selected path
      if (generateSelectedPath(groupID, rotDir, pathScale, pathRange, relativeGoalDis, path)) {
        pathFound = true;
        RCLCPP_DEBUG(this->get_logger(), "Path found at iteration %d with score %.3f (scale: %.2f, range: %.2f)",
                     searchIteration, maxScore, pathScale, pathRange);
        
        // Early exit if we found a high-quality path
        if (maxScore >= GOOD_SCORE_THRESHOLD) {
          RCLCPP_DEBUG(this->get_logger(), "Early exit: found high-quality path (score >= %.2f)", GOOD_SCORE_THRESHOLD);
          break;
        }
        
        // Continue searching for better paths at smaller scales
        // but we already have a valid path
      }
    }

    // Decrease scale or range
    if (pathScale >= config_.minPathScale + config_.pathScaleStep) {
      pathScale -= static_cast<float>(config_.pathScaleStep);
      pathRange = static_cast<float>(config_.adjacentRange * pathScale / defPathScale);
    } else {
      pathRange -= static_cast<float>(config_.pathRangeStep);
    }
  }
  
  // If no path was found but we have a best candidate, try to use it
  if (!pathFound && bestSelectedGroupInd >= 0 && bestMaxScore > 0.0f) {
    int groupID = bestSelectedGroupInd % PathData::groupNum;
    int rotDir = bestSelectedGroupInd / PathData::groupNum;
    
    RCLCPP_DEBUG(this->get_logger(), "Using fallback path with score %.3f (scale: %.2f, range: %.2f)",
                 bestMaxScore, bestPathScale, bestPathRange);
    
    if (generateSelectedPath(groupID, rotDir, bestPathScale, bestPathRange, bestRelativeGoalDis, path)) {
      pathFound = true;
    }
  }

  // Always update free_paths visualization with the last search parameters
  // This ensures visualization is always up-to-date, even when no path is found
  generateFreePathsVisualization(visPathScale, visPathRange, visRelativeGoalDis,
                                visMinObsAngCW, visMinObsAngCCW);

  return pathFound;
}

bool LocalPlanner::findPath(nav_msgs::msg::Path& path)
{
  bool found = findPathWithMultiScaleSearch(path);

  if (!found) {
    generateZeroLengthPath(path);
  }

  return found;
}

void LocalPlanner::generateZeroLengthPath(nav_msgs::msg::Path& path)
{
  path.poses.clear();
  path.poses.resize(1);

  path.poses[0].header.frame_id = "vehicle";
  path.poses[0].pose.position.x = 0.0;
  path.poses[0].pose.position.y = 0.0;
  path.poses[0].pose.position.z = 0.0;
  path.poses[0].pose.orientation.w = 1.0;

  path.header.stamp = rclcpp::Time(static_cast<uint64_t>(state_.odomTime * 1e9));
  path.header.frame_id = "vehicle";

  // Note: free_paths is now updated in findPathWithMultiScaleSearch()
  // regardless of whether a path is found, so we don't clear it here
}

}  // namespace local_planner

// ============================================================================
// Main 函数
// ============================================================================

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto planner = std::make_shared<local_planner::LocalPlanner>();

  if (rclcpp::ok()) {
    rclcpp::spin(planner);
  }

  rclcpp::shutdown();
  return 0;
}
