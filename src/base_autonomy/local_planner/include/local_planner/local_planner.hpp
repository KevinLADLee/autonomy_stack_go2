#ifndef LOCAL_PLANNER__LOCAL_PLANNER_HPP_
#define LOCAL_PLANNER__LOCAL_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>

// ROS2 消息类型
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

// TF2
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// PCL 类型
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

namespace local_planner
{

// 常量定义
constexpr double PI = 3.14159265358979323846;
#define PLOTPATHSET 1

// 配置参数结构体
struct PlannerConfig
{
  // 路径文件夹
  std::string pathFolder;

  // 车辆参数
  double vehicleLength = 0.6;
  double vehicleWidth = 0.6;
  double sensorOffsetX = 0.0;
  double sensorOffsetY = 0.0;
  bool twoWayDrive = true;

  // 传感器参数
  double laserVoxelSize = 0.05;
  double terrainVoxelSize = 0.2;
  bool useTerrainAnalysis = false;

  // 障碍物检测
  bool checkObstacle = true;
  bool checkRotObstacle = false;
  double adjacentRange = 3.5;
  double obstacleHeightThre = 0.2;
  double groundHeightThre = 0.1;
  double costHeightThre = 0.1;
  double costScore = 0.02;
  bool useCost = false;

  // 路径参数
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

  // 自主模式
  bool autonomyMode = false;
  double autonomySpeed = 1.0;
  double goalCloseDis = 1.0;
  double goalClearRange = 0.5;
  double goalX = 0.0;
  double goalY = 0.0;

  // 从 ROS2 参数加载配置
  void loadFromParameters(rclcpp::Node* node);
};

// 车辆状态
struct VehicleState
{
  // 姿态
  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;

  // 位置
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;

  // 时间戳
  double odomTime = 0.0;

  // 自主控制状态
  float normalizedSpeed = 0.0f;      // 归一化速度 [0, 1]
  float targetDirection = 0.0f;      // 目标方向（度数，-180 到 180）
  bool hasValidGoal = false;         // 是否有有效目标
};

// 点云容器 - 全部使用 shared_ptr
struct PointCloudData
{
  // 激光点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz;

  // 地形点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudCrop;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz;

  // 点云栈
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> laserCloudStack;
  int laserCloudCount = 0;

  // 规划器点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloudCrop;

  // 边界和障碍物
  pcl::PointCloud<pcl::PointXYZI>::Ptr boundaryCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr addedObstacles;

  // 体素滤波器
  pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter;
  pcl::VoxelGrid<pcl::PointXYZI> terrainDwzFilter;

  // 新数据标志
  bool newLaserCloud = false;
  bool newTerrainCloud = false;

  PointCloudData();
  ~PointCloudData() = default;

  // 禁止拷贝
  PointCloudData(const PointCloudData&) = delete;
  PointCloudData& operator=(const PointCloudData&) = delete;

  // 允许移动
  PointCloudData(PointCloudData&&) = default;
  PointCloudData& operator=(PointCloudData&&) = default;
};

// 路径数据 - 路径点云使用 shared_ptr
struct PathData
{
  // 路径网格常量
  static constexpr int pathNum = 343;
  static constexpr int groupNum = 7;
  static constexpr int gridVoxelNumX = 161;
  static constexpr int gridVoxelNumY = 451;
  static constexpr int gridVoxelNum = gridVoxelNumX * gridVoxelNumY;
  static constexpr int laserCloudStackNum = 1;

  // 算法常量
  static constexpr int kNumRotationDirections = 36;
  static constexpr float kRotationDegreeIncrement = 10.0f;
  static constexpr float kMaxAngleDegrees = 180.0f;
  static constexpr float kFullCircleDegrees = 360.0f;
  static constexpr int kPointSkipCount = 30;

  // 网格参数
  float gridVoxelSize = 0.02f;
  float searchRadius = 0.55f;
  float gridVoxelOffsetX = 3.2f;
  float gridVoxelOffsetY = 4.5f;

  // 路径列表
  std::vector<int> pathList;
  std::vector<float> endDirPathList;

  // 清除路径数据
  std::vector<int> clearPathList;
  std::vector<float> pathPenaltyList;
  std::vector<float> clearPathPerGroupScore;

  // 对应关系
  std::vector<std::vector<int>> correspondences;

  // 起始路径
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> startPaths;

  // 预计算路径
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> paths;

#if PLOTPATHSET == 1
  // 自由路径可视化
  pcl::PointCloud<pcl::PointXYZI>::Ptr freePaths;
#endif

  PathData();
  ~PathData() = default;

  void resizeArrays();

  // 禁止拷贝
  PathData(const PathData&) = delete;
  PathData& operator=(const PathData&) = delete;

  // 允许移动
  PathData(PathData&&) = default;
  PathData& operator=(PathData&&) = default;

  // 文件读取方法（委托给 io 命名空间的函数）
  bool readStartPaths(const std::string& pathFolder);
  bool readPaths(const std::string& pathFolder);
  bool readPathList(const std::string& pathFolder);
  bool readCorrespondences(const std::string& pathFolder);
};

// 主 LocalPlanner 类
class LocalPlanner : public rclcpp::Node
{
public:
  explicit LocalPlanner(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~LocalPlanner() = default;

  // 禁止拷贝和移动
  LocalPlanner(const LocalPlanner&) = delete;
  LocalPlanner& operator=(const LocalPlanner&) = delete;
  LocalPlanner(LocalPlanner&&) = delete;
  LocalPlanner& operator=(LocalPlanner&&) = delete;

private:
  // 配置和状态
  PlannerConfig config_;
  VehicleState state_;
  PointCloudData clouds_;
  PathData pathData_;

  // ROS2 订阅者
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_laser_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_terrain_cloud_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr sub_boundary_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_added_obstacles_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_check_obstacle_;

  // ROS2 发布者
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
#if PLOTPATHSET == 1
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_free_paths_;
#endif

  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;

  // 初始化方法
  void declareParameters();
  void loadParameters();
  void initializeSubscribers();
  void initializePublishers();
  void initializeFilters();
  bool loadPathFiles();

  // ROS2 回调函数
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void laserCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void terrainCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void boundaryCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
  void addedObstaclesCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void checkObstacleCallback(const std_msgs::msg::Bool::SharedPtr msg);

  // 定时器回调 - 主处理循环
  void timerCallback();

  // 核心处理方法
  void processPointClouds();
  void transformAndCropClouds();
  void transformPointCloudToVehicleFrame(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& outputCloud,
    bool checkRangeAndZ = true,
    bool clearOutput = true
  ) const;
  bool findPath(nav_msgs::msg::Path& path);
  void publishPath(const nav_msgs::msg::Path& path);
  void publishFreePaths();

  // 辅助方法
  void updatePathScaleAndRange(float& pathScale, float& pathRange);

  // 目标更新方法
  void updateTargetFromGoal();

  // 角度和方向工具方法
  float computeAngleDifference(float angle1, float angle2) const;
  float normalizeAngle(float angle) const;
  bool isDirectionValid(int rotDir) const;
  bool isRotationValid(float rotAng, float rotDeg, float minObsAngCW, float minObsAngCCW) const;

  // 目标计算
  void calculateRelativeGoal(
    float& relativeGoalX,
    float& relativeGoalY,
    float& relativeGoalDis,
    float& desiredDirection
  ) const;

  // 障碍物检测
  void initializeScoringArrays();
  void checkPointAgainstPaths(
    const pcl::PointXYZI& point,
    float pathScale,
    float pathRange,
    float relativeGoalDis,
    float& minObsAngCW,
    float& minObsAngCCW
  );

  // 路径评分
  void scoreAllPaths(float desiredDir, float relativeGoalDis);

  // 路径选择
  int selectBestPathGroup(float& maxScore, float minObsAngCW, float minObsAngCCW) const;

  // 路径生成
  bool generateSelectedPath(
    int groupID,
    int rotDir,
    float pathScale,
    float pathRange,
    float relativeGoalDis,
    nav_msgs::msg::Path& path
  );
  void generateFreePathsVisualization(
    float pathScale,
    float pathRange,
    float relativeGoalDis,
    float minObsAngCW,
    float minObsAngCCW
  );

  // 主搜索循环
  bool findPathWithMultiScaleSearch(nav_msgs::msg::Path& path);
  void generateZeroLengthPath(nav_msgs::msg::Path& path);
};

}  // namespace local_planner

#endif  // LOCAL_PLANNER__LOCAL_PLANNER_HPP_
