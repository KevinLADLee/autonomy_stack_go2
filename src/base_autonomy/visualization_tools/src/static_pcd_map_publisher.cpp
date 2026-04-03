#include <algorithm>
#include <chrono>
#include <fstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

class StaticPcdMapPublisher : public rclcpp::Node {
public:
  StaticPcdMapPublisher() : Node("static_pcd_map_publisher") {
    this->declare_parameter<std::string>("pcd_file", "");
    this->declare_parameter<std::string>("source_topic", "");
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<std::string>("topic_name", "/overall_map");
    this->declare_parameter<double>("voxel_size", 0.0);
    this->declare_parameter<int>("publish_delay_ms", 1000);

    this->get_parameter("pcd_file", pcd_file_);
    this->get_parameter("source_topic", source_topic_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("topic_name", topic_name_);
    this->get_parameter("voxel_size", voxel_size_);
    this->get_parameter("publish_delay_ms", publish_delay_ms_);

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name_, qos);

    if (load_map_msg()) {
      const int delay_ms = std::max(1, publish_delay_ms_);
      publish_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(delay_ms),
          std::bind(&StaticPcdMapPublisher::publish_once, this));
      return;
    }

    if (!source_topic_.empty()) {
      source_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          source_topic_,
          rclcpp::SensorDataQoS(),
          std::bind(&StaticPcdMapPublisher::source_cloud_callback, this, std::placeholders::_1));
      RCLCPP_WARN(
          this->get_logger(),
          "Falling back to source topic '%s'. Waiting for first cloud and then latching to %s.",
          source_topic_.c_str(),
          topic_name_.c_str());
      return;
    }

    RCLCPP_ERROR(
        this->get_logger(),
        "No valid map source available. Provide a standard PCD file or set source_topic.");
  }

private:
  static bool has_standard_pcd_header(const std::string &pcd_file) {
    std::ifstream in(pcd_file, std::ios::binary);
    if (!in.is_open()) {
      return false;
    }

    std::string first_line;
    std::getline(in, first_line);
    return first_line.rfind("# .PCD", 0) == 0 || first_line.rfind("VERSION", 0) == 0;
  }

  bool load_map_msg() {
    if (pcd_file_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Parameter 'pcd_file' is empty.");
      return false;
    }

    pcl::PCLPointCloud2 pcl_cloud;
    if (pcl::io::loadPCDFile(pcd_file_, pcl_cloud) < 0) {
      if (!has_standard_pcd_header(pcd_file_)) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to load %s. The file does not look like a standard PCD header.",
            pcd_file_.c_str());
      }
      RCLCPP_ERROR(this->get_logger(), "Failed to load PCD map: %s", pcd_file_.c_str());
      return false;
    }

    if (voxel_size_ > 0.0) {
      pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
      vg.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
      vg.setInputCloud(pcl::make_shared<pcl::PCLPointCloud2>(pcl_cloud));

      pcl::PCLPointCloud2 filtered_cloud;
      vg.filter(filtered_cloud);
      pcl_cloud = filtered_cloud;
    }

    pcl_conversions::fromPCL(pcl_cloud, map_msg_);
    map_msg_.header.frame_id = frame_id_;

    const size_t point_count =
        static_cast<size_t>(map_msg_.width) * static_cast<size_t>(map_msg_.height);
    RCLCPP_INFO(
        this->get_logger(),
        "Loaded PCD map with %zu points from %s (frame: %s).",
        point_count,
        pcd_file_.c_str(),
        frame_id_.c_str());

    return true;
  }

  void source_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (published_ || !map_pub_) {
      return;
    }

    map_msg_ = *msg;
    if (!frame_id_.empty()) {
      map_msg_.header.frame_id = frame_id_;
    }
    map_msg_.header.stamp = this->now();

    map_pub_->publish(map_msg_);
    published_ = true;
    source_sub_.reset();

    RCLCPP_INFO(
        this->get_logger(),
        "Latched first cloud from %s onto %s with transient_local durability.",
        source_topic_.c_str(),
        topic_name_.c_str());
  }

  void publish_once() {
    if (published_ || !map_pub_) {
      return;
    }

    map_msg_.header.stamp = this->now();
    map_pub_->publish(map_msg_);
    published_ = true;

    RCLCPP_INFO(
        this->get_logger(),
        "Published latched map on %s with transient_local durability.",
        topic_name_.c_str());

    publish_timer_->cancel();
  }

  std::string pcd_file_;
  std::string source_topic_;
  std::string frame_id_;
  std::string topic_name_;
  double voxel_size_ = 0.0;
  int publish_delay_ms_ = 1000;
  bool published_ = false;

  sensor_msgs::msg::PointCloud2 map_msg_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr source_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticPcdMapPublisher>());
  rclcpp::shutdown();
  return 0;
}
