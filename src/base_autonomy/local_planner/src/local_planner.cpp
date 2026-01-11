#include "local_planner/local_planner.hpp"

namespace local_planner
{

// ============================================================================
// PlannerConfig 实现
// ============================================================================

void PlannerConfig::loadFromParameters(rclcpp::Node* node)
{
  // 路径文件夹
  node->get_parameter("pathFolder", pathFolder);

  // 车辆参数
  node->get_parameter("vehicleLength", vehicleLength);
  node->get_parameter("vehicleWidth", vehicleWidth);
  node->get_parameter("sensorOffsetX", sensorOffsetX);
  node->get_parameter("sensorOffsetY", sensorOffsetY);
  node->get_parameter("twoWayDrive", twoWayDrive);

  // 传感器参数
  node->get_parameter("laserVoxelSize", laserVoxelSize);
  node->get_parameter("terrainVoxelSize", terrainVoxelSize);
  node->get_parameter("useTerrainAnalysis", useTerrainAnalysis);

  // 障碍物检测
  node->get_parameter("checkObstacle", checkObstacle);
  node->get_parameter("checkRotObstacle", checkRotObstacle);
  node->get_parameter("adjacentRange", adjacentRange);
  node->get_parameter("obstacleHeightThre", obstacleHeightThre);
  node->get_parameter("groundHeightThre", groundHeightThre);
  node->get_parameter("costHeightThre", costHeightThre);
  node->get_parameter("costScore", costScore);
  node->get_parameter("useCost", useCost);

  // 路径参数
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

  // 自主模式
  node->get_parameter("autonomyMode", autonomyMode);
  node->get_parameter("autonomySpeed", autonomySpeed);
  node->get_parameter("joyToSpeedDelay", joyToSpeedDelay);
  node->get_parameter("joyToCheckObstacleDelay", joyToCheckObstacleDelay);
  node->get_parameter("goalCloseDis", goalCloseDis);
  node->get_parameter("goalClearRange", goalClearRange);
  node->get_parameter("goalX", goalX);
  node->get_parameter("goalY", goalY);
}

// ============================================================================
// PointCloudData 实现
// ============================================================================

PointCloudData::PointCloudData()
: laserCloud(new pcl::PointCloud<pcl::PointXYZI>())
, laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>())
, laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>())
, terrainCloud(new pcl::PointCloud<pcl::PointXYZI>())
, terrainCloudCrop(new pcl::PointCloud<pcl::PointXYZI>())
, terrainCloudDwz(new pcl::PointCloud<pcl::PointXYZI>())
, laserCloudCount(0)
, plannerCloud(new pcl::PointCloud<pcl::PointXYZI>())
, plannerCloudCrop(new pcl::PointCloud<pcl::PointXYZI>())
, boundaryCloud(new pcl::PointCloud<pcl::PointXYZI>())
, addedObstacles(new pcl::PointCloud<pcl::PointXYZI>())
, newLaserCloud(false)
, newTerrainCloud(false)
{
  // 预分配点云栈
  laserCloudStack.resize(PathData::laserCloudStackNum);
  for (auto& cloud : laserCloudStack) {
    cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
}

// ============================================================================
// PathData 实现
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
  clearPathList.resize(36 * pathNum, 0);
  pathPenaltyList.resize(36 * pathNum, 0.0f);
  clearPathPerGroupScore.resize(36 * groupNum, 0.0f);
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

int PathData::readPlyHeader(std::ifstream& file)
{
  std::string strCur, strLast;
  int pointNum = 0;
  while (strCur != "end_header") {
    if (!(file >> strCur)) {
      return -1;
    }

    if (strCur == "vertex" && strLast == "element") {
      if (!(file >> pointNum)) {
        return -1;
      }
    }
    strLast = strCur;
  }

  return pointNum;
}

bool PathData::readStartPaths(const std::string& pathFolder)
{
  std::string fileName = pathFolder + "/startPaths.ply";

  std::ifstream file(fileName);
  if (!file.is_open()) {
    return false;
  }

  int pointNum = readPlyHeader(file);
  if (pointNum < 0) {
    return false;
  }

  pcl::PointXYZ point;
  int groupID;
  for (int i = 0; i < pointNum; i++) {
    if (!(file >> point.x >> point.y >> point.z >> groupID)) {
      return false;
    }

    if (groupID >= 0 && groupID < groupNum) {
      startPaths[groupID]->push_back(point);
    }
  }

  file.close();
  return true;
}

bool PathData::readPaths(const std::string& pathFolder)
{
#if PLOTPATHSET == 1
  std::string fileName = pathFolder + "/paths.ply";

  std::ifstream file(fileName);
  if (!file.is_open()) {
    return false;
  }

  int pointNum = readPlyHeader(file);
  if (pointNum < 0) {
    return false;
  }

  pcl::PointXYZI point;
  int pointSkipNum = 30;
  int pointSkipCount = 0;
  int pathID;
  for (int i = 0; i < pointNum; i++) {
    if (!(file >> point.x >> point.y >> point.z >> pathID >> point.intensity)) {
      return false;
    }

    if (pathID >= 0 && pathID < pathNum) {
      pointSkipCount++;
      if (pointSkipCount > pointSkipNum) {
        paths[pathID]->push_back(point);
        pointSkipCount = 0;
      }
    }
  }

  file.close();
#endif
  return true;
}

bool PathData::readPathList(const std::string& pathFolder)
{
  std::string fileName = pathFolder + "/pathList.ply";

  std::ifstream file(fileName);
  if (!file.is_open()) {
    return false;
  }

  if (pathNum != readPlyHeader(file)) {
    return false;
  }

  int pathID, groupID;
  float endX, endY, endZ;
  for (int i = 0; i < pathNum; i++) {
    if (!(file >> endX >> endY >> endZ >> pathID >> groupID)) {
      return false;
    }

    if (pathID >= 0 && pathID < pathNum && groupID >= 0 && groupID < groupNum) {
      pathList[pathID] = groupID;
      endDirPathList[pathID] = static_cast<float>(2.0 * std::atan2(endY, endX) * 180.0 / PI);
    }
  }

  file.close();
  return true;
}

bool PathData::readCorrespondences(const std::string& pathFolder)
{
  std::string fileName = pathFolder + "/correspondences.txt";

  std::ifstream file(fileName);
  if (!file.is_open()) {
    return false;
  }

  int gridVoxelID, pathID;
  for (int i = 0; i < gridVoxelNum; i++) {
    if (!(file >> gridVoxelID)) {
      return false;
    }

    while (true) {
      if (!(file >> pathID)) {
        return false;
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

  file.close();
  return true;
}

// ============================================================================
// LocalPlanner 类实现
// ============================================================================

LocalPlanner::LocalPlanner(const rclcpp::NodeOptions& options)
: rclcpp::Node("local_planner", options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing LocalPlanner...");

  // 1. 声明参数
  declareParameters();

  // 2. 加载参数
  loadParameters();

  // 3. 初始化点云数据
  clouds_ = PointCloudData();
  pathData_ = PathData();
  pathData_.resizeArrays();

  // 4. 初始化滤波器
  initializeFilters();

  // 5. 加载路径文件
  if (!loadPathFiles()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load path files");
    return;
  }

  // 6. 初始化订阅者
  initializeSubscribers();

  // 7. 初始化发布者
  initializePublishers();

  // 8. 初始化定时器 (100Hz)
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&LocalPlanner::timerCallback, this)
  );

  RCLCPP_INFO(this->get_logger(), "LocalPlanner initialized successfully");
}

void LocalPlanner::declareParameters()
{
  // 路径文件夹
  this->declare_parameter<std::string>("pathFolder", config_.pathFolder);

  // 车辆参数
  this->declare_parameter<double>("vehicleLength", config_.vehicleLength);
  this->declare_parameter<double>("vehicleWidth", config_.vehicleWidth);
  this->declare_parameter<double>("sensorOffsetX", config_.sensorOffsetX);
  this->declare_parameter<double>("sensorOffsetY", config_.sensorOffsetY);
  this->declare_parameter<bool>("twoWayDrive", config_.twoWayDrive);

  // 传感器参数
  this->declare_parameter<double>("laserVoxelSize", config_.laserVoxelSize);
  this->declare_parameter<double>("terrainVoxelSize", config_.terrainVoxelSize);
  this->declare_parameter<bool>("useTerrainAnalysis", config_.useTerrainAnalysis);

  // 障碍物检测
  this->declare_parameter<bool>("checkObstacle", config_.checkObstacle);
  this->declare_parameter<bool>("checkRotObstacle", config_.checkRotObstacle);
  this->declare_parameter<double>("adjacentRange", config_.adjacentRange);
  this->declare_parameter<double>("obstacleHeightThre", config_.obstacleHeightThre);
  this->declare_parameter<double>("groundHeightThre", config_.groundHeightThre);
  this->declare_parameter<double>("costHeightThre", config_.costHeightThre);
  this->declare_parameter<double>("costScore", config_.costScore);
  this->declare_parameter<bool>("useCost", config_.useCost);

  // 路径参数
  this->declare_parameter<int>("pointPerPathThre", config_.pointPerPathThre);
  this->declare_parameter<double>("minRelZ", config_.minRelZ);
  this->declare_parameter<double>("maxRelZ", config_.maxRelZ);
  this->declare_parameter<double>("maxSpeed", config_.maxSpeed);
  this->declare_parameter<double>("dirWeight", config_.dirWeight);
  this->declare_parameter<double>("dirThre", config_.dirThre);
  this->declare_parameter<bool>("dirToVehicle", config_.dirToVehicle);
  this->declare_parameter<double>("pathScale", config_.pathScale);
  this->declare_parameter<double>("minPathScale", config_.minPathScale);
  this->declare_parameter<double>("pathScaleStep", config_.pathScaleStep);
  this->declare_parameter<bool>("pathScaleBySpeed", config_.pathScaleBySpeed);
  this->declare_parameter<double>("minPathRange", config_.minPathRange);
  this->declare_parameter<double>("pathRangeStep", config_.pathRangeStep);
  this->declare_parameter<bool>("pathRangeBySpeed", config_.pathRangeBySpeed);
  this->declare_parameter<bool>("pathCropByGoal", config_.pathCropByGoal);

  // 自主模式
  this->declare_parameter<bool>("autonomyMode", config_.autonomyMode);
  this->declare_parameter<double>("autonomySpeed", config_.autonomySpeed);
  this->declare_parameter<double>("joyToSpeedDelay", config_.joyToSpeedDelay);
  this->declare_parameter<double>("joyToCheckObstacleDelay", config_.joyToCheckObstacleDelay);
  this->declare_parameter<double>("goalCloseDis", config_.goalCloseDis);
  this->declare_parameter<double>("goalClearRange", config_.goalClearRange);
  this->declare_parameter<double>("goalX", config_.goalX);
  this->declare_parameter<double>("goalY", config_.goalY);
}

void LocalPlanner::loadParameters()
{
  config_.loadFromParameters(this);

  // Initialize speed to 0 - robot waits for goal_pose to start moving
  state_.joySpeed = 0.0f;
  state_.joyDir = 0.0f;
}

void LocalPlanner::initializeFilters()
{
  clouds_.laserDwzFilter.setLeafSize(
    config_.laserVoxelSize,
    config_.laserVoxelSize,
    config_.laserVoxelSize
  );

  clouds_.terrainDwzFilter.setLeafSize(
    config_.terrainVoxelSize,
    config_.terrainVoxelSize,
    config_.terrainVoxelSize
  );
}

bool LocalPlanner::loadPathFiles()
{
  RCLCPP_INFO(this->get_logger(), "Reading path files from: %s", config_.pathFolder.c_str());

  if (!pathData_.readStartPaths(config_.pathFolder)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to read start paths");
    return false;
  }

#if PLOTPATHSET == 1
  if (!pathData_.readPaths(config_.pathFolder)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to read paths");
    return false;
  }
#endif

  if (!pathData_.readPathList(config_.pathFolder)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to read path list");
    return false;
  }

  if (!pathData_.readCorrespondences(config_.pathFolder)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to read correspondences");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Path files loaded successfully");
  return true;
}

void LocalPlanner::initializeSubscribers()
{
  sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/state_estimation", 5,
    std::bind(&LocalPlanner::odometryCallback, this, std::placeholders::_1)
  );

  sub_laser_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/registered_scan", 5,
    std::bind(&LocalPlanner::laserCloudCallback, this, std::placeholders::_1)
  );

  sub_terrain_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/terrain_map", 5,
    std::bind(&LocalPlanner::terrainCloudCallback, this, std::placeholders::_1)
  );

  // Disable joystick subscription since we're not using joystick input
  // sub_joystick_ = this->create_subscription<sensor_msgs::msg::Joy>(
  //   "/joy", 5,
  //   std::bind(&LocalPlanner::joystickCallback, this, std::placeholders::_1)
  // );

  sub_goal_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 1,
    std::bind(&LocalPlanner::goalPoseCallback, this, std::placeholders::_1)
  );

  // Disable speed subscription - speed is controlled by goal_pose only
  // sub_speed_ = this->create_subscription<std_msgs::msg::Float32>("/speed", 5, std::bind(&LocalPlanner::speedCallback, this, std::placeholders::_1));

  sub_boundary_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
    "/navigation_boundary", 5,
    std::bind(&LocalPlanner::boundaryCallback, this, std::placeholders::_1)
  );

  sub_added_obstacles_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/added_obstacles", 5,
    std::bind(&LocalPlanner::addedObstaclesCallback, this, std::placeholders::_1)
  );

  sub_check_obstacle_ = this->create_subscription<std_msgs::msg::Bool>(
    "/check_obstacle", 5,
    std::bind(&LocalPlanner::checkObstacleCallback, this, std::placeholders::_1)
  );
}

void LocalPlanner::initializePublishers()
{
  pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/autonomy_stack/path", 5);

#if PLOTPATHSET == 1
  pub_free_paths_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/autonomy_stack/free_paths", 2);
#endif
}

void LocalPlanner::initializeTimer()
{
  // Timer 已在构造函数中创建
}

// ============================================================================
// ROS2 回调函数
// ============================================================================

void LocalPlanner::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  state_.odomTime = rclcpp::Time(msg->header.stamp).seconds();

  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = msg->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  state_.roll = static_cast<float>(roll);
  state_.pitch = static_cast<float>(pitch);
  state_.yaw = static_cast<float>(yaw);
  state_.x = static_cast<float>(msg->pose.pose.position.x - std::cos(yaw) * config_.sensorOffsetX + std::sin(yaw) * config_.sensorOffsetY);
  state_.y = static_cast<float>(msg->pose.pose.position.y - std::sin(yaw) * config_.sensorOffsetX - std::cos(yaw) * config_.sensorOffsetY);
  state_.z = static_cast<float>(msg->pose.pose.position.z);
}

void LocalPlanner::laserCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!config_.useTerrainAnalysis) {
    clouds_.laserCloud->clear();
    pcl::fromROSMsg(*msg, *clouds_.laserCloud);

    pcl::PointXYZI point;
    clouds_.laserCloudCrop->clear();
    for (const auto& pt : clouds_.laserCloud->points) {
      float pointX = pt.x;
      float pointY = pt.y;
      float pointZ = pt.z;

      float dis = std::hypot(pointX - state_.x, pointY - state_.y);
      if (dis < config_.adjacentRange) {
        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        clouds_.laserCloudCrop->push_back(point);
      }
    }

    clouds_.laserCloudDwz->clear();
    clouds_.laserDwzFilter.setInputCloud(clouds_.laserCloudCrop);
    clouds_.laserDwzFilter.filter(*clouds_.laserCloudDwz);

    clouds_.newLaserCloud = true;
  }
}

void LocalPlanner::terrainCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (config_.useTerrainAnalysis) {
    clouds_.terrainCloud->clear();
    pcl::fromROSMsg(*msg, *clouds_.terrainCloud);

    pcl::PointXYZI point;
    clouds_.terrainCloudCrop->clear();
    for (const auto& pt : clouds_.terrainCloud->points) {
      float pointX = pt.x;
      float pointY = pt.y;
      float pointZ = pt.z;

      float dis = std::hypot(pointX - state_.x, pointY - state_.y);
      if (dis < config_.adjacentRange && (pt.intensity > config_.obstacleHeightThre || config_.useCost)) {
        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        point.intensity = pt.intensity;
        clouds_.terrainCloudCrop->push_back(point);
      }
    }

    clouds_.terrainCloudDwz->clear();
    clouds_.terrainDwzFilter.setInputCloud(clouds_.terrainCloudCrop);
    clouds_.terrainDwzFilter.filter(*clouds_.terrainCloudDwz);

    clouds_.newTerrainCloud = true;
  }
}

void LocalPlanner::joystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  state_.joyTime = this->now().seconds();
  state_.joySpeedRaw = std::hypot(msg->axes[3], msg->axes[4]);
  state_.joySpeed = state_.joySpeedRaw;
  if (state_.joySpeed > 1.0f) state_.joySpeed = 1.0f;
  if (msg->axes[4] == 0.0f) state_.joySpeed = 0.0f;

  if (state_.joySpeed > 0.0f) {
    state_.joyDir = static_cast<float>(std::atan2(msg->axes[3], msg->axes[4]) * 180.0 / PI);
    if (msg->axes[4] < 0.0f) state_.joyDir *= -1.0f;
  }

  if (msg->axes[4] < 0.0f && !config_.twoWayDrive) state_.joySpeed = 0.0f;

  if (msg->axes[2] > -0.1f) {
    config_.autonomyMode = false;
  } else {
    config_.autonomyMode = true;
  }

  if (msg->axes[5] > -0.1f) {
    config_.checkObstacle = true;
  } else {
    config_.checkObstacle = false;
  }
}

void LocalPlanner::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  config_.goalX = msg->pose.position.x;
  config_.goalY = msg->pose.position.y;

  // Switch to autonomy mode when goal is received
  config_.autonomyMode = true;

  // Set speed based on autonomySpeed
  state_.joySpeed = static_cast<float>(config_.autonomySpeed / config_.maxSpeed);
  if (state_.joySpeed < 0.0f) state_.joySpeed = 0.0f;
  else if (state_.joySpeed > 1.0f) state_.joySpeed = 1.0f;

  RCLCPP_INFO(this->get_logger(), "Received goal_pose: x=%.2f, y=%.2f, frame_id=%s, autonomyMode=ENABLED",
    config_.goalX, config_.goalY, msg->header.frame_id.c_str());
}

void LocalPlanner::speedCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  // Check if we have a valid goal before applying speed
  float sinVehicleYaw = std::sin(state_.yaw);
  float cosVehicleYaw = std::cos(state_.yaw);
  float relativeGoalX = static_cast<float>((config_.goalX - state_.x) * cosVehicleYaw + (config_.goalY - state_.y) * sinVehicleYaw);
  float relativeGoalY = static_cast<float>(-(config_.goalX - state_.x) * sinVehicleYaw + (config_.goalY - state_.y) * cosVehicleYaw);
  float goalDistance = std::hypot(relativeGoalX, relativeGoalY);
  bool hasValidGoal = (goalDistance > 0.1f);

  double speedTime = this->now().seconds();
  if (config_.autonomyMode && hasValidGoal && speedTime - state_.joyTime > config_.joyToSpeedDelay && state_.joySpeedRaw == 0.0f) {
    state_.joySpeed = msg->data / static_cast<float>(config_.maxSpeed);

    if (state_.joySpeed < 0.0f) state_.joySpeed = 0.0f;
    else if (state_.joySpeed > 1.0f) state_.joySpeed = 1.0f;
  }
}

void LocalPlanner::boundaryCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  clouds_.boundaryCloud->clear();
  pcl::PointXYZI point, point1, point2;
  int boundarySize = msg->polygon.points.size();

  if (boundarySize >= 1) {
    point2.x = msg->polygon.points[0].x;
    point2.y = msg->polygon.points[0].y;
    point2.z = msg->polygon.points[0].z;
  }

  for (int i = 0; i < boundarySize; i++) {
    point1 = point2;

    point2.x = msg->polygon.points[i].x;
    point2.y = msg->polygon.points[i].y;
    point2.z = msg->polygon.points[i].z;

    if (point1.z == point2.z) {
      float disX = point1.x - point2.x;
      float disY = point1.y - point2.y;
      float dis = std::hypot(disX, disY);

      int pointNum = static_cast<int>(dis / config_.terrainVoxelSize) + 1;
      for (int pointID = 0; pointID < pointNum; pointID++) {
        point.x = static_cast<float>(pointID) / static_cast<float>(pointNum) * point1.x +
                  (1.0f - static_cast<float>(pointID) / static_cast<float>(pointNum)) * point2.x;
        point.y = static_cast<float>(pointID) / static_cast<float>(pointNum) * point1.y +
                  (1.0f - static_cast<float>(pointID) / static_cast<float>(pointNum)) * point2.y;
        point.z = 0.0f;
        point.intensity = 100.0f;

        for (int j = 0; j < config_.pointPerPathThre; j++) {
          clouds_.boundaryCloud->push_back(point);
        }
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
  double checkObsTime = this->now().seconds();
  if (config_.autonomyMode && checkObsTime - state_.joyTime > config_.joyToCheckObstacleDelay) {
    config_.checkObstacle = msg->data;
  }
}

// ============================================================================
// 定时器回调 - 主处理循环
// ============================================================================

void LocalPlanner::timerCallback()
{
  // 处理回调以最小化命令延迟
  // rclcpp::spin_some(shared_from_this());

  if (clouds_.newLaserCloud || clouds_.newTerrainCloud) {
    processPointClouds();
    transformAndCropClouds();

    nav_msgs::msg::Path path;
    bool pathFound = findPath(path);

    // Only publish path if we found a valid one
    if (pathFound && state_.joySpeed > 0.0f) {
      publishPath(path);
    }
    publishFreePaths();
  }
}

// ============================================================================
// 核心处理方法
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

    clouds_.plannerCloud->clear();
    *clouds_.plannerCloud = *clouds_.terrainCloudDwz;
  }
}

void LocalPlanner::transformAndCropClouds()
{
  float sinVehicleYaw = std::sin(state_.yaw);
  float cosVehicleYaw = std::cos(state_.yaw);

  pcl::PointXYZI point;
  clouds_.plannerCloudCrop->clear();

  // 变换规划器点云
  for (const auto& pt : clouds_.plannerCloud->points) {
    float pointX1 = pt.x - state_.x;
    float pointY1 = pt.y - state_.y;
    float pointZ1 = pt.z - state_.z;

    point.x = pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
    point.y = -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
    point.z = pointZ1;
    point.intensity = pt.intensity;

    float dis = std::hypot(point.x, point.y);
    if (dis < config_.adjacentRange &&
        ((point.z > config_.minRelZ && point.z < config_.maxRelZ) || config_.useTerrainAnalysis)) {
      clouds_.plannerCloudCrop->push_back(point);
    }
  }

  // 变换边界点云
  for (const auto& pt : clouds_.boundaryCloud->points) {
    float pointX1 = pt.x - state_.x;
    float pointY1 = pt.y - state_.y;

    point.x = pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
    point.y = -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
    point.z = pt.z;
    point.intensity = pt.intensity;

    float dis = std::hypot(point.x, point.y);
    if (dis < config_.adjacentRange) {
      clouds_.plannerCloudCrop->push_back(point);
    }
  }

  // 变换障碍物点云
  for (const auto& pt : clouds_.addedObstacles->points) {
    float pointX1 = pt.x - state_.x;
    float pointY1 = pt.y - state_.y;

    point.x = pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
    point.y = -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
    point.z = pt.z;
    point.intensity = pt.intensity;

    float dis = std::hypot(point.x, point.y);
    if (dis < config_.adjacentRange) {
      clouds_.plannerCloudCrop->push_back(point);
    }
  }
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

void LocalPlanner::updateJoystickSpeed()
{
  // 手柄速度已在回调中更新
}

void LocalPlanner::updatePathScaleAndRange(float& pathScale, float& pathRange)
{
  if (config_.pathRangeBySpeed) {
    pathRange = static_cast<float>(config_.adjacentRange * state_.joySpeed);
  }
  if (pathRange < config_.minPathRange) {
    pathRange = static_cast<float>(config_.minPathRange);
  }

  if (config_.pathScaleBySpeed) {
    pathScale = static_cast<float>(config_.pathScale * state_.joySpeed);
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
  if (diff > PathData::kMaxAngleDegrees) {
    diff = PathData::kFullCircleDegrees - diff;
  }
  return diff;
}

float LocalPlanner::normalizeAngle(float angle) const
{
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
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

  // Handle autonomy mode vs joystick
  if (config_.autonomyMode) {
    // Use goal direction - clamp to forward direction if not two-way drive
    if (!config_.twoWayDrive) {
      if (desiredDirection > 90.0f) desiredDirection = 90.0f;
      else if (desiredDirection < -90.0f) desiredDirection = -90.0f;
    }
  } else {
    // Use joystick direction
    desiredDirection = state_.joyDir;
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
    // ============ Direction constraint check (exactly from original code) ============
    float rotAngRaw = 10.0f * rotDir - 180.0f;
    float angDiff = std::fabs(state_.joyDir - rotAngRaw);
    if (angDiff > 180.0f) {
      angDiff = 360.0f - angDiff;
    }

    if ((angDiff > config_.dirThre && !config_.dirToVehicle) ||
        (std::fabs(rotAngRaw) > config_.dirThre && std::fabs(state_.joyDir) <= 90.0f && config_.dirToVehicle) ||
        ((rotAngRaw > config_.dirThre && 360.0f - rotAngRaw > config_.dirThre) &&
         std::fabs(state_.joyDir) > 90.0f && config_.dirToVehicle)) {
      continue;
    }
    // ==============================================================================

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
  float diameter = std::hypot(static_cast<float>(config_.vehicleLength), static_cast<float>(config_.vehicleWidth));
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

float LocalPlanner::scorePath(int pathIndex, int rotDir, float desiredDir, float relativeGoalDis) const
{
  // Get penalty score
  int pathInd = PathData::pathNum * rotDir + pathIndex;
  float penaltyScore = std::max(static_cast<float>(config_.costScore),
                                1.0f - std::min(pathData_.pathPenaltyList[pathInd] / static_cast<float>(config_.costHeightThre), 1.0f));

  // Calculate direction score
  float endDir = pathData_.endDirPathList[pathIndex];
  float rotAng = PathData::kRotationDegreeIncrement * rotDir - 180.0f;
  float pathDir = normalizeAngle(endDir + rotAng);
  float dirDiff = computeAngleDifference(desiredDir, pathDir);

  // Calculate rotation direction weight
  float rotDirW = (rotDir < 18) ? std::fabs(std::fabs(rotDir - 9.0f) + 1.0f) : std::fabs(std::fabs(rotDir - 27.0f) + 1.0f);

  // Calculate group direction weight
  int groupID = pathData_.pathList[pathIndex];
  float groupDirW = 4.0f - std::fabs(groupID - 3.0f);

  // Calculate final score
  float score = (1.0f - std::sqrt(std::sqrt(config_.dirWeight * dirDiff))) *
                std::pow(rotDirW, 4.0f) * penaltyScore;

  // Adjust for goal proximity
  if (relativeGoalDis < config_.goalCloseDis) {
    score = (1.0f - std::sqrt(std::sqrt(config_.dirWeight * dirDiff))) *
            std::pow(groupDirW, 2.0f) * penaltyScore;
  }

  return score;
}

void LocalPlanner::scoreAllPaths(float desiredDir, float relativeGoalDis)
{
  // Score all paths and accumulate by group (exactly from original code)
  for (int i = 0; i < 36 * PathData::pathNum; i++) {
    int rotDir = i / PathData::pathNum;

    // ============ Direction constraint check (exactly from original code) ============
    float rotAngRaw = 10.0f * rotDir - 180.0f;
    float angDiff = std::fabs(state_.joyDir - rotAngRaw);
    if (angDiff > 180.0f) {
      angDiff = 360.0f - angDiff;
    }

    if ((angDiff > config_.dirThre && !config_.dirToVehicle) ||
        (std::fabs(rotAngRaw) > config_.dirThre && std::fabs(state_.joyDir) <= 90.0f && config_.dirToVehicle) ||
        ((rotAngRaw > config_.dirThre && 360.0f - rotAngRaw > config_.dirThre) &&
         std::fabs(state_.joyDir) > 90.0f && config_.dirToVehicle)) {
      continue;
    }
    // ==============================================================================

    // Skip if path is blocked
    if (pathData_.clearPathList[i] >= config_.pointPerPathThre) {
      continue;
    }

    // Calculate penalty score (from original code)
    float penaltyScore = 1.0f - pathData_.pathPenaltyList[i] / static_cast<float>(config_.costHeightThre);
    if (penaltyScore < config_.costScore) penaltyScore = static_cast<float>(config_.costScore);

    // Calculate direction difference (from original code)
    float dirDiff = std::fabs(state_.joyDir - pathData_.endDirPathList[i % PathData::pathNum] - (10.0f * rotDir - 180.0f));
    if (dirDiff > 360.0f) {
      dirDiff -= 360.0f;
    }
    if (dirDiff > 180.0f) {
      dirDiff = 360.0f - dirDiff;
    }

    // Calculate rotation direction weight (from original code)
    float rotDirW;
    if (rotDir < 18) rotDirW = std::fabs(std::fabs(rotDir - 9.0f) + 1.0f);
    else rotDirW = std::fabs(std::fabs(rotDir - 27.0f) + 1.0f);

    // Calculate group direction weight (from original code)
    float groupDirW = 4.0f - std::fabs(pathData_.pathList[i % PathData::pathNum] - 3.0f);

    // Calculate final score (from original code)
    float score = (1.0f - std::sqrt(std::sqrt(config_.dirWeight * dirDiff))) *
                  rotDirW * rotDirW * rotDirW * rotDirW * penaltyScore;

    // Adjust for goal proximity (from original code)
    if (relativeGoalDis < config_.goalCloseDis) {
      score = (1.0f - std::sqrt(std::sqrt(config_.dirWeight * dirDiff))) *
              groupDirW * groupDirW * penaltyScore;
    }

    // Accumulate by group (from original code)
    if (score > 0.0f) {
      pathData_.clearPathPerGroupScore[PathData::groupNum * rotDir + pathData_.pathList[i % PathData::pathNum]] += score;
    }
  }
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

    // ============ Rotational obstacle constraint check (exactly from original code) ============
    bool rotAngOK = (rotAng * 180.0f / PI > minObsAngCW && rotAng * 180.0f / PI < minObsAngCCW);
    bool rotDegOK = (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && config_.twoWayDrive);
    bool noCheck = !config_.checkRotObstacle;

    if (pathData_.clearPathPerGroupScore[i] > maxScore &&
        (rotAngOK || rotDegOK || noCheck)) {
      maxScore = pathData_.clearPathPerGroupScore[i];
      selectedGroupID = i;
    }
    // ==============================================================================
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
  float rotAng = (PathData::kRotationDegreeIncrement * rotDir - 180.0f) * PI / 180.0f;

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

    float rotAng = (10.0f * rotDir - 180.0f) * PI / 180.0f;
    float rotDeg = 10.0f * rotDir;
    if (rotDeg > 180.0f) rotDeg -= 360.0f;

    // ============ Direction constraint check (exactly from original code) ============
    float angDiff = std::fabs(state_.joyDir - (10.0f * rotDir - 180.0f));
    if (angDiff > 180.0f) {
      angDiff = 360.0f - angDiff;
    }

    if ((angDiff > config_.dirThre && !config_.dirToVehicle) ||
        (std::fabs(10.0f * rotDir - 180.0f) > config_.dirThre && std::fabs(state_.joyDir) <= 90.0f && config_.dirToVehicle) ||
        ((10.0f * rotDir > config_.dirThre && 360.0f - 10.0f * rotDir > config_.dirThre) &&
         std::fabs(state_.joyDir) > 90.0f && config_.dirToVehicle)) {
      continue;
    }
    // ==============================================================================

    // ============ Rotational obstacle constraint check (exactly from original code) ============
    bool rotAngOK = (rotAng * 180.0f / PI > minObsAngCW && rotAng * 180.0f / PI < minObsAngCCW);
    bool rotDegOK = (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && config_.twoWayDrive);
    bool noCheck = !config_.checkRotObstacle;

    if (!(rotAngOK || rotDegOK || noCheck)) {
      continue;
    }
    // ====================================================================================================

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
  // Calculate initial scale and range (exactly from original code)
  float pathRange = static_cast<float>(config_.adjacentRange);
  if (config_.pathRangeBySpeed) pathRange = static_cast<float>(config_.adjacentRange * state_.joySpeed);
  if (pathRange < config_.minPathRange) pathRange = static_cast<float>(config_.minPathRange);
  float relativeGoalDis = static_cast<float>(config_.adjacentRange);

  // ============ Autonomy mode: update joyDir from goal ============
  // Check if we have a valid goal by calculating distance first
  float sinVehicleYaw = std::sin(state_.yaw);
  float cosVehicleYaw = std::cos(state_.yaw);

  float relativeGoalX = static_cast<float>((config_.goalX - state_.x) * cosVehicleYaw + (config_.goalY - state_.y) * sinVehicleYaw);
  float relativeGoalY = static_cast<float>(-(config_.goalX - state_.x) * sinVehicleYaw + (config_.goalY - state_.y) * cosVehicleYaw);

  float goalDistance = std::hypot(relativeGoalX, relativeGoalY);
  // Consider goal valid if distance > 0.5m (to avoid treating (0,0) as valid goal when robot is at origin)
  bool hasValidGoal = (goalDistance > 0.1f);

  // In autonomy mode, require a valid goal to proceed
  if (config_.autonomyMode && !hasValidGoal) {
    // No valid goal yet, set speed to 0 and don't try to find a path
    state_.joySpeed = 0.0f;
    return false;
  }

  if (config_.autonomyMode && hasValidGoal) {
    relativeGoalDis = goalDistance;
    state_.joyDir = std::atan2(relativeGoalY, relativeGoalX) * 180.0f / PI;

    if (!config_.twoWayDrive) {
      if (state_.joyDir > 90.0f) state_.joyDir = 90.0f;
      else if (state_.joyDir < -90.0f) state_.joyDir = -90.0f;
    }

    // Restore speed when we have a valid goal
    state_.joySpeed = static_cast<float>(config_.autonomySpeed / config_.maxSpeed);
    if (state_.joySpeed < 0.0f) state_.joySpeed = 0.0f;
    else if (state_.joySpeed > 1.0f) state_.joySpeed = 1.0f;
  }
  // ================================================================

  float pathScale = static_cast<float>(config_.pathScale);
  float defPathScale = pathScale;
  if (config_.pathScaleBySpeed) pathScale = static_cast<float>(config_.pathScale * state_.joySpeed);
  if (pathScale < config_.minPathScale) pathScale = static_cast<float>(config_.minPathScale);

  bool pathFound = false;

  // Multi-scale search loop (exactly from original code)
  while (pathScale >= config_.minPathScale && pathRange >= config_.minPathRange) {
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
    scoreAllPaths(state_.joyDir, relativeGoalDis);

    // Select best path
    float maxScore;
    int selectedGroupInd = selectBestPathGroup(maxScore, minObsAngCW, minObsAngCCW);

    if (selectedGroupInd >= 0 && maxScore > 0.0f) {
      // Extract group ID and rotation direction
      int groupID = selectedGroupInd % PathData::groupNum;
      int rotDir = selectedGroupInd / PathData::groupNum;

      // Generate selected path
      if (generateSelectedPath(groupID, rotDir, pathScale, pathRange, relativeGoalDis, path)) {
        // ============ Only generate free_paths when path is found (exactly from original code) ============
        generateFreePathsVisualization(pathScale, pathRange, relativeGoalDis,
                                      minObsAngCW, minObsAngCCW);
        // ====================================================================================================
        pathFound = true;
        break;
      }
    }

    // Reduce scale or range (exactly from original code)
    if (pathScale >= config_.minPathScale + config_.pathScaleStep) {
      pathScale -= static_cast<float>(config_.pathScaleStep);
      pathRange = static_cast<float>(config_.adjacentRange * pathScale / defPathScale);
    } else {
      pathRange -= static_cast<float>(config_.pathRangeStep);
    }
  }

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

  // Clear free_paths when no path is found (exactly from original code lines 968-974)
#if PLOTPATHSET == 1
  pathData_.freePaths->clear();
#endif
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
