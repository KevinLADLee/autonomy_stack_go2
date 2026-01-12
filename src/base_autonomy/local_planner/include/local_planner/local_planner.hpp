#ifndef LOCAL_PLANNER__LOCAL_PLANNER_HPP_
#define LOCAL_PLANNER__LOCAL_PLANNER_HPP_

#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace local_planner
{

// Enable free paths visualization for debugging
#define PLOTPATHSET 1

namespace constants
{
constexpr double kPi = 3.14159265358979323846;
constexpr int kNumRotationDirections = 36;
constexpr float kRotationDegreeIncrement = 10.0f;
constexpr float kMaxAngleDegrees = 180.0f;
constexpr float kFullCircleDegrees = 360.0f;
constexpr int kPointSkipCount = 30;
}  // namespace constants

/// Configuration parameters loaded from ROS parameters
struct PlannerConfig
{
  std::string pathFolder;

  // Vehicle dimensions
  double vehicleLength = 0.6;
  double vehicleWidth = 0.6;
  double sensorOffsetX = 0.0;
  double sensorOffsetY = 0.0;
  bool twoWayDrive = true;

  // Sensor processing
  double laserVoxelSize = 0.05;
  double terrainVoxelSize = 0.2;
  bool useTerrainAnalysis = false;

  // Obstacle detection
  bool checkObstacle = true;
  bool checkRotObstacle = false;
  double adjacentRange = 3.5;
  double obstacleHeightThre = 0.2;
  double groundHeightThre = 0.1;
  double costHeightThre = 0.1;
  double costScore = 0.02;
  bool useCost = false;

  // Path parameters
  int pointPerPathThre = 2;
  double minRelZ = -0.5;
  double maxRelZ = 0.25;
  double maxSpeed = 1.0;
  double dirWeight = 0.02;
  double dirThre = 90.0;
  bool dirToVehicle = false;
  double pathScale = 1.0;
  double minPathScale = 0.75;
  double pathScaleStep = 0.25;
  bool pathScaleBySpeed = true;
  double minPathRange = 1.0;
  double pathRangeStep = 0.5;
  bool pathRangeBySpeed = true;
  bool pathCropByGoal = true;

  // Goal parameters
  double goalCloseDis = 1.0;
  double goalClearRange = 0.5;
  double goalX = 0.0;
  double goalY = 0.0;

  void loadFromParameters(rclcpp::Node* node);
};

/// Current vehicle state
struct VehicleState
{
  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  double odomTime = 0.0;

  // Autonomy control
  float normalizedSpeed = 0.0f;
  float targetDirection = 0.0f;
  bool hasValidGoal = false;
};

/// Point cloud data storage and processing
struct PointCloudData
{
  // Laser point clouds
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz;

  // Terrain point clouds
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudCrop;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz;

  // Point cloud stack for temporal aggregation
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> laserCloudStack;
  int laserCloudCount = 0;

  // Planner point clouds
  pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloudCrop;

  // Boundaries and obstacles
  pcl::PointCloud<pcl::PointXYZI>::Ptr boundaryCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr addedObstacles;

  // Voxel filters
  pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter;
  pcl::VoxelGrid<pcl::PointXYZI> terrainDwzFilter;

  // Data availability flags
  bool newLaserCloud = false;
  bool newTerrainCloud = false;

  PointCloudData();
  ~PointCloudData() = default;

  PointCloudData(const PointCloudData&) = delete;
  PointCloudData& operator=(const PointCloudData&) = delete;
  PointCloudData(PointCloudData&&) = default;
  PointCloudData& operator=(PointCloudData&&) = default;
};

/// Pre-computed path data and scoring arrays
struct PathData
{
  // Path grid constants
  static constexpr int pathNum = 343;
  static constexpr int groupNum = 7;
  static constexpr int gridVoxelNumX = 161;
  static constexpr int gridVoxelNumY = 451;
  static constexpr int gridVoxelNum = gridVoxelNumX * gridVoxelNumY;
  static constexpr int laserCloudStackNum = 1;

  // Grid parameters
  float gridVoxelSize = 0.02f;
  float searchRadius = 0.55f;
  float gridVoxelOffsetX = 3.2f;
  float gridVoxelOffsetY = 4.5f;

  // Path metadata
  std::vector<int> pathList;
  std::vector<float> endDirPathList;

  // Scoring arrays
  std::vector<int> clearPathList;
  std::vector<float> pathPenaltyList;
  std::vector<float> clearPathPerGroupScore;

  // Grid-to-path correspondences
  std::vector<std::vector<int>> correspondences;

  // Path point clouds
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> startPaths;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> paths;

#if PLOTPATHSET == 1
  pcl::PointCloud<pcl::PointXYZI>::Ptr freePaths;
#endif

  PathData();
  ~PathData() = default;

  PathData(const PathData&) = delete;
  PathData& operator=(const PathData&) = delete;
  PathData(PathData&&) = default;
  PathData& operator=(PathData&&) = default;

  void resizeArrays();

  bool readStartPaths(const std::string& pathFolder);
  bool readPaths(const std::string& pathFolder);
  bool readPathList(const std::string& pathFolder);
  bool readCorrespondences(const std::string& pathFolder);
};

/// Main local planner ROS2 node
class LocalPlanner : public rclcpp::Node
{
public:
  explicit LocalPlanner(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~LocalPlanner() = default;

  LocalPlanner(const LocalPlanner&) = delete;
  LocalPlanner& operator=(const LocalPlanner&) = delete;
  LocalPlanner(LocalPlanner&&) = delete;
  LocalPlanner& operator=(LocalPlanner&&) = delete;

private:
  // Configuration and state
  PlannerConfig config_;
  VehicleState state_;
  PointCloudData clouds_;
  PathData pathData_;

  // ROS2 subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_laser_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_terrain_cloud_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr sub_boundary_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_added_obstacles_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_check_obstacle_;

  // ROS2 publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
#if PLOTPATHSET == 1
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_free_paths_;
#endif

  // Timer for main control loop
  rclcpp::TimerBase::SharedPtr timer_;

  // Initialization
  void declareParameters();
  void loadParameters();
  void initializeROSInterfaces();
  void initializeFilters();
  bool loadPathFiles();

  // ROS2 callbacks
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void laserCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void terrainCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void boundaryCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
  void addedObstaclesCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void checkObstacleCallback(const std_msgs::msg::Bool::SharedPtr msg);

  // Main processing loop
  void timerCallback();

  // Point cloud processing
  void processPointClouds();
  void transformAndCropClouds();
  void transformPointCloudToVehicleFrame(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& outputCloud,
    bool checkRangeAndZ = true,
    bool clearOutput = true) const;

  // Path finding
  bool findPath(nav_msgs::msg::Path& path);
  bool findPathWithMultiScaleSearch(nav_msgs::msg::Path& path);
  void publishPath(const nav_msgs::msg::Path& path);
  void publishFreePaths();

  // Goal handling
  void updateTargetFromGoal();
  void calculateRelativeGoal(
    float& relativeGoalX,
    float& relativeGoalY,
    float& relativeGoalDis,
    float& desiredDirection) const;

  // Angle utilities
  float computeAngleDifference(float angle1, float angle2) const;
  float normalizeAngle(float angle) const;
  bool isDirectionValid(int rotDir) const;
  bool isRotationValid(float rotAng, float rotDeg, float minObsAngCW, float minObsAngCCW) const;

  // Obstacle detection and path scoring
  void initializeScoringArrays();
  void checkPointAgainstPaths(
    const pcl::PointXYZI& point,
    float pathScale,
    float pathRange,
    float relativeGoalDis,
    float& minObsAngCW,
    float& minObsAngCCW);
  void scoreAllPaths(float desiredDir, float relativeGoalDis);
  int selectBestPathGroup(float& maxScore, float minObsAngCW, float minObsAngCCW) const;

  // Path generation
  bool generateSelectedPath(
    int groupID,
    int rotDir,
    float pathScale,
    float pathRange,
    float relativeGoalDis,
    nav_msgs::msg::Path& path);
  void generateFreePathsVisualization(
    float pathScale,
    float pathRange,
    float relativeGoalDis,
    float minObsAngCW,
    float minObsAngCCW);
  void generateZeroLengthPath(nav_msgs::msg::Path& path);

  // Helper methods
  void updatePathScaleAndRange(float& pathScale, float& pathRange);
};

}  // namespace local_planner

#endif  // LOCAL_PLANNER__LOCAL_PLANNER_HPP_
