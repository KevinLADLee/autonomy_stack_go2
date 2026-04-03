#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{

using json = nlohmann::json;

double normalizeAngle(double angle)
{
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

double distance3d(double ax, double ay, double az, double bx, double by, double bz)
{
  const double dx = ax - bx;
  const double dy = ay - by;
  const double dz = az - bz;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double distance2d(double ax, double ay, double bx, double by)
{
  const double dx = ax - bx;
  const double dy = ay - by;
  return std::sqrt(dx * dx + dy * dy);
}

double slopeWeight(const std::string & slope_type)
{
  if (slope_type == "ramp") {
    return 1.15;
  }
  if (slope_type == "slope_transition") {
    return 1.3;
  }
  return 1.0;
}

double riskWeight(const std::string & risk_level)
{
  if (risk_level == "medium") {
    return 1.2;
  }
  if (risk_level == "high") {
    return 1.6;
  }
  return 1.0;
}

}  // namespace

class RouteNetworkNavigator : public rclcpp::Node
{
public:
  RouteNetworkNavigator()
  : Node("route_network_navigator")
  {
    declare_parameter<std::string>("route_file", "");
    declare_parameter<double>("goal_publish_rate_hz", 5.0);
    declare_parameter<double>("arrival_check_rate_hz", 10.0);
    declare_parameter<bool>("replan_on_goal_update", true);
    declare_parameter<bool>("allow_goal_snap_to_edge", false);
    declare_parameter<double>("max_goal_snap_distance_m", 3.0);
    declare_parameter<double>("max_start_snap_distance_m", 3.0);
    declare_parameter<std::string>("default_frame_id", "map");
    declare_parameter<double>("path_progress_timeout_sec", 20.0);
    declare_parameter<double>("xy_reach_fallback", 0.4);
    declare_parameter<double>("yaw_reach_fallback", 0.35);
    declare_parameter<bool>("enable_visualization", true);
    declare_parameter<std::string>("target_goal_topic", "/route_network_navigator/target_goal");
    declare_parameter<std::string>("state_estimation_topic", "/state_estimation");
    declare_parameter<std::string>("goal_pose_topic", "/goal_pose");
    declare_parameter<std::string>("planned_path_topic", "/route_network_navigator/planned_path");
    declare_parameter<std::string>("status_topic", "/route_network_navigator/status");

    route_file_ = get_parameter("route_file").as_string();
    default_frame_id_ = get_parameter("default_frame_id").as_string();
    max_goal_snap_distance_m_ = get_parameter("max_goal_snap_distance_m").as_double();
    max_start_snap_distance_m_ = get_parameter("max_start_snap_distance_m").as_double();
    path_progress_timeout_sec_ = get_parameter("path_progress_timeout_sec").as_double();
    xy_reach_fallback_ = get_parameter("xy_reach_fallback").as_double();
    yaw_reach_fallback_ = get_parameter("yaw_reach_fallback").as_double();
    replan_on_goal_update_ = get_parameter("replan_on_goal_update").as_bool();

    const auto state_estimation_topic = get_parameter("state_estimation_topic").as_string();
    const auto target_goal_topic = get_parameter("target_goal_topic").as_string();
    const auto goal_pose_topic = get_parameter("goal_pose_topic").as_string();
    const auto planned_path_topic = get_parameter("planned_path_topic").as_string();
    const auto status_topic = get_parameter("status_topic").as_string();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      state_estimation_topic, 10, std::bind(&RouteNetworkNavigator::odometryCallback, this, std::placeholders::_1));
    target_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      target_goal_topic, 10, std::bind(&RouteNetworkNavigator::targetGoalCallback, this, std::placeholders::_1));

    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_pose_topic, 10);
    planned_path_pub_ = create_publisher<nav_msgs::msg::Path>(planned_path_topic, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic, 10);

    const auto tick_hz = std::max(
      get_parameter("goal_publish_rate_hz").as_double(),
      get_parameter("arrival_check_rate_hz").as_double());
    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1.0, tick_hz))),
      std::bind(&RouteNetworkNavigator::tick, this));

    route_loaded_ = loadRoute(route_file_);
    if (route_loaded_) {
      publishStatus("ready: route loaded");
    } else {
      publishStatus("idle: route not loaded");
    }
  }

private:
  struct Waypoint
  {
    std::string id;
    std::string name;
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double yaw{0.0};
    double tol_xy{0.4};
    double tol_yaw{0.35};
  };

  struct Edge
  {
    std::string id;
    std::string from_id;
    std::string to_id;
    bool enabled{true};
    std::string direction{"bidirectional"};
    double length_m{0.0};
    std::string slope_type{"flat"};
    double max_slope_deg{0.0};
    std::string risk_level{"low"};
  };

  struct Neighbor
  {
    std::size_t to_index{0};
    std::size_t edge_index{0};
    double cost{0.0};
  };

  struct ExecutionGoal
  {
    geometry_msgs::msg::PoseStamped pose;
    double tol_xy{0.4};
    double tol_yaw{0.35};
    std::string label;
  };

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_goal_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<Waypoint> waypoints_;
  std::vector<Edge> edges_;
  std::vector<std::vector<Neighbor>> adjacency_;
  std::unordered_map<std::string, std::size_t> waypoint_index_by_id_;

  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  geometry_msgs::msg::PoseStamped current_target_goal_;
  bool route_loaded_{false};
  bool has_active_plan_{false};
  bool replan_on_goal_update_{true};
  bool final_goal_appended_{false};
  std::vector<ExecutionGoal> execution_goals_;
  std::size_t current_goal_index_{0};
  double last_goal_distance_{std::numeric_limits<double>::infinity()};
  rclcpp::Time last_progress_time_;

  std::string route_file_;
  std::string default_frame_id_;
  double max_goal_snap_distance_m_{3.0};
  double max_start_snap_distance_m_{3.0};
  double path_progress_timeout_sec_{20.0};
  double xy_reach_fallback_{0.4};
  double yaw_reach_fallback_{0.35};

  void publishStatus(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "%s", text.c_str());
  }

  bool loadRoute(const std::string & route_file)
  {
    if (route_file.empty()) {
      RCLCPP_WARN(get_logger(), "Parameter 'route_file' is empty.");
      return false;
    }

    std::ifstream input(route_file);
    if (!input.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open route file: %s", route_file.c_str());
      return false;
    }

    json root;
    try {
      input >> root;
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Failed to parse route JSON: %s", ex.what());
      return false;
    }

    if (root.value("format_version", "") != "route_graph_export.v1") {
      RCLCPP_ERROR(get_logger(), "Unsupported route format: %s", root.value("format_version", "").c_str());
      return false;
    }

    waypoints_.clear();
    edges_.clear();
    adjacency_.clear();
    waypoint_index_by_id_.clear();

    default_frame_id_ = root.value("frame_id", default_frame_id_);

    const auto & waypoint_json = root["waypoints"];
    for (const auto & item : waypoint_json) {
      Waypoint wp;
      wp.id = item.at("id").get<std::string>();
      wp.name = item.value("name", wp.id);
      const auto & pose = item.at("pose");
      wp.x = pose.value("x", 0.0);
      wp.y = pose.value("y", 0.0);
      wp.z = pose.value("z", 0.0);
      wp.yaw = pose.value("yaw", 0.0);
      wp.tol_xy = item.value("arrival_tolerance_xy", xy_reach_fallback_);
      wp.tol_yaw = item.value("arrival_tolerance_yaw", yaw_reach_fallback_);
      waypoint_index_by_id_[wp.id] = waypoints_.size();
      waypoints_.push_back(wp);
    }

    adjacency_.assign(waypoints_.size(), {});

    const auto & edge_json = root["edges"];
    for (const auto & item : edge_json) {
      Edge edge;
      edge.id = item.at("id").get<std::string>();
      edge.from_id = item.at("from_waypoint_id").get<std::string>();
      edge.to_id = item.at("to_waypoint_id").get<std::string>();
      edge.enabled = item.value("enabled", true);
      edge.direction = item.value("direction", "bidirectional");
      edge.length_m = item.value("length_m", 0.0);
      edge.slope_type = item.value("slope_type", "flat");
      edge.max_slope_deg = item.value("max_slope_deg", 0.0);
      edge.risk_level = item.value("risk_level", "low");
      edges_.push_back(edge);
    }

    for (std::size_t edge_index = 0; edge_index < edges_.size(); ++edge_index) {
      const auto & edge = edges_[edge_index];
      const auto from_it = waypoint_index_by_id_.find(edge.from_id);
      const auto to_it = waypoint_index_by_id_.find(edge.to_id);
      if (from_it == waypoint_index_by_id_.end() || to_it == waypoint_index_by_id_.end()) {
        continue;
      }
      if (!edge.enabled) {
        continue;
      }

      const double base_cost = edge.length_m > 0.0 ? edge.length_m :
        distance3d(
          waypoints_[from_it->second].x, waypoints_[from_it->second].y, waypoints_[from_it->second].z,
          waypoints_[to_it->second].x, waypoints_[to_it->second].y, waypoints_[to_it->second].z);
      const double cost = base_cost * slopeWeight(edge.slope_type) * riskWeight(edge.risk_level);

      if (edge.direction == "bidirectional" || edge.direction == "forward_only") {
        adjacency_[from_it->second].push_back(Neighbor{to_it->second, edge_index, cost});
      }
      if (edge.direction == "bidirectional" || edge.direction == "reverse_only") {
        adjacency_[to_it->second].push_back(Neighbor{from_it->second, edge_index, cost});
      }
    }

    RCLCPP_INFO(
      get_logger(), "Loaded route graph: %zu waypoints, %zu edges", waypoints_.size(), edges_.size());
    return !waypoints_.empty();
  }

  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_odom_ = msg;
  }

  void targetGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    current_target_goal_ = *msg;
    if (current_target_goal_.header.frame_id.empty()) {
      current_target_goal_.header.frame_id = default_frame_id_;
    }

    if (!route_loaded_) {
      publishStatus("failed: route not loaded");
      return;
    }
    if (!latest_odom_) {
      publishStatus("failed: no odometry yet");
      return;
    }
    if (has_active_plan_ && !replan_on_goal_update_) {
      publishStatus("ignored: active plan in progress");
      return;
    }

    if (planToGoal(current_target_goal_)) {
      publishStatus("running: plan ready");
    }
  }

  std::size_t findNearestWaypoint(double x, double y, double z, double max_distance, double * out_distance = nullptr) const
  {
    std::size_t best_index = std::numeric_limits<std::size_t>::max();
    double best_distance = std::numeric_limits<double>::infinity();

    for (std::size_t index = 0; index < waypoints_.size(); ++index) {
      const auto & wp = waypoints_[index];
      const double dist = distance3d(x, y, z, wp.x, wp.y, wp.z);
      if (dist < best_distance) {
        best_distance = dist;
        best_index = index;
      }
    }

    if (out_distance != nullptr) {
      *out_distance = best_distance;
    }

    if (best_distance > max_distance) {
      return std::numeric_limits<std::size_t>::max();
    }
    return best_index;
  }

  std::vector<std::size_t> runAStar(std::size_t start_index, std::size_t goal_index) const
  {
    struct QueueItem
    {
      double f_score;
      std::size_t index;
      bool operator<(const QueueItem & other) const
      {
        return f_score > other.f_score;
      }
    };

    std::priority_queue<QueueItem> open_set;
    std::vector<double> g_score(waypoints_.size(), std::numeric_limits<double>::infinity());
    std::vector<std::size_t> came_from(waypoints_.size(), std::numeric_limits<std::size_t>::max());
    std::vector<bool> closed(waypoints_.size(), false);

    g_score[start_index] = 0.0;
    open_set.push(QueueItem{heuristic(start_index, goal_index), start_index});

    while (!open_set.empty()) {
      const auto current = open_set.top().index;
      open_set.pop();

      if (closed[current]) {
        continue;
      }
      closed[current] = true;

      if (current == goal_index) {
        std::vector<std::size_t> path;
        std::size_t cursor = goal_index;
        while (cursor != std::numeric_limits<std::size_t>::max()) {
          path.push_back(cursor);
          if (cursor == start_index) {
            break;
          }
          cursor = came_from[cursor];
        }
        std::reverse(path.begin(), path.end());
        if (!path.empty() && path.front() == start_index && path.back() == goal_index) {
          return path;
        }
        return {};
      }

      for (const auto & neighbor : adjacency_[current]) {
        if (closed[neighbor.to_index]) {
          continue;
        }

        const double tentative_g = g_score[current] + neighbor.cost;
        if (tentative_g >= g_score[neighbor.to_index]) {
          continue;
        }

        g_score[neighbor.to_index] = tentative_g;
        came_from[neighbor.to_index] = current;
        open_set.push(QueueItem{tentative_g + heuristic(neighbor.to_index, goal_index), neighbor.to_index});
      }
    }

    return {};
  }

  double heuristic(std::size_t from_index, std::size_t to_index) const
  {
    const auto & from = waypoints_[from_index];
    const auto & to = waypoints_[to_index];
    return distance2d(from.x, from.y, to.x, to.y);
  }

  bool planToGoal(const geometry_msgs::msg::PoseStamped & target_goal)
  {
    const auto & pose = latest_odom_->pose.pose;
    double start_snap_distance = 0.0;
    double goal_snap_distance = 0.0;
    const auto start_index = findNearestWaypoint(
      pose.position.x, pose.position.y, pose.position.z, max_start_snap_distance_m_, &start_snap_distance);
    const auto goal_index = findNearestWaypoint(
      target_goal.pose.position.x, target_goal.pose.position.y, target_goal.pose.position.z,
      max_goal_snap_distance_m_, &goal_snap_distance);

    if (start_index == std::numeric_limits<std::size_t>::max()) {
      publishStatus("failed: robot pose too far from route network");
      return false;
    }
    if (goal_index == std::numeric_limits<std::size_t>::max()) {
      publishStatus("failed: target pose too far from route network");
      return false;
    }

    auto waypoint_path = runAStar(start_index, goal_index);
    if (waypoint_path.empty()) {
      publishStatus("failed: no route path found");
      return false;
    }

    execution_goals_.clear();
    for (const auto waypoint_index : waypoint_path) {
      const auto & wp = waypoints_[waypoint_index];
      ExecutionGoal exec_goal;
      exec_goal.pose.header.frame_id = default_frame_id_;
      exec_goal.pose.pose.position.x = wp.x;
      exec_goal.pose.pose.position.y = wp.y;
      exec_goal.pose.pose.position.z = wp.z;
      exec_goal.pose.pose.orientation = quaternionFromYaw(wp.yaw);
      exec_goal.tol_xy = wp.tol_xy;
      exec_goal.tol_yaw = wp.tol_yaw;
      exec_goal.label = wp.id;
      execution_goals_.push_back(exec_goal);
    }

    final_goal_appended_ = false;
    const auto & snapped_goal_wp = waypoints_[goal_index];
    const double final_leg_distance = distance3d(
      snapped_goal_wp.x, snapped_goal_wp.y, snapped_goal_wp.z,
      target_goal.pose.position.x, target_goal.pose.position.y, target_goal.pose.position.z);
    if (final_leg_distance > std::max(0.05, xy_reach_fallback_ * 0.5)) {
      ExecutionGoal final_goal;
      final_goal.pose = target_goal;
      final_goal.pose.header.frame_id = target_goal.header.frame_id.empty() ? default_frame_id_ : target_goal.header.frame_id;
      final_goal.tol_xy = xy_reach_fallback_;
      final_goal.tol_yaw = yaw_reach_fallback_;
      final_goal.label = "final_goal";
      execution_goals_.push_back(final_goal);
      final_goal_appended_ = true;
    }

    current_goal_index_ = 0;
    has_active_plan_ = !execution_goals_.empty();
    last_goal_distance_ = std::numeric_limits<double>::infinity();
    last_progress_time_ = now();

    publishPlannedPath();

    std::ostringstream ss;
    ss << "planned: " << execution_goals_.size() << " sub-goals"
       << ", start_snap=" << start_snap_distance
       << ", goal_snap=" << goal_snap_distance
       << ", final_goal=" << (final_goal_appended_ ? "yes" : "no");
    publishStatus(ss.str());
    return true;
  }

  void publishPlannedPath()
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = default_frame_id_;
    for (const auto & goal : execution_goals_) {
      auto pose = goal.pose;
      pose.header.stamp = path.header.stamp;
      if (pose.header.frame_id.empty()) {
        pose.header.frame_id = default_frame_id_;
      }
      path.poses.push_back(pose);
    }
    planned_path_pub_->publish(path);
  }

  void tick()
  {
    if (!has_active_plan_ || !latest_odom_) {
      return;
    }
    if (current_goal_index_ >= execution_goals_.size()) {
      has_active_plan_ = false;
      publishStatus("completed: route execution finished");
      return;
    }

    auto & goal = execution_goals_[current_goal_index_];
    goal.pose.header.stamp = now();
    if (goal.pose.header.frame_id.empty()) {
      goal.pose.header.frame_id = default_frame_id_;
    }
    goal_pub_->publish(goal.pose);

    const auto & robot_pose = latest_odom_->pose.pose;
    const double xy_error = distance2d(
      robot_pose.position.x, robot_pose.position.y,
      goal.pose.pose.position.x, goal.pose.pose.position.y);
    const double yaw_error = std::abs(
      normalizeAngle(yawFromQuaternion(robot_pose.orientation) - yawFromQuaternion(goal.pose.pose.orientation)));

    if (xy_error < last_goal_distance_ - 0.02) {
      last_goal_distance_ = xy_error;
      last_progress_time_ = now();
    }

    if (xy_error <= goal.tol_xy && yaw_error <= goal.tol_yaw) {
      std::ostringstream ss;
      ss << "sub-goal reached: " << goal.label;
      publishStatus(ss.str());
      current_goal_index_++;
      last_goal_distance_ = std::numeric_limits<double>::infinity();
      last_progress_time_ = now();
      if (current_goal_index_ >= execution_goals_.size()) {
        has_active_plan_ = false;
        publishStatus("completed: route execution finished");
      }
      return;
    }

    if ((now() - last_progress_time_).seconds() > path_progress_timeout_sec_) {
      has_active_plan_ = false;
      std::ostringstream ss;
      ss << "failed: timeout at sub-goal " << goal.label;
      publishStatus(ss.str());
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RouteNetworkNavigator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
