#include <memory>
#include <limits>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl/common/transforms.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class OdinFrameAdapter : public rclcpp::Node {
public:
  OdinFrameAdapter()
      : Node("odin_adapter"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {
    raw_odom_topic_ = declare_parameter<std::string>("raw_odom_topic", "/odin1/highodom");
    raw_cloud_topic_ = declare_parameter<std::string>("raw_cloud_topic", "/odin1/cloud_slam");
    output_odom_topic_ = declare_parameter<std::string>("output_odom_topic", "/state_estimation");
    output_cloud_topic_ = declare_parameter<std::string>("output_cloud_topic", "/registered_scan");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    tf_lookup_timeout_sec_ = declare_parameter<double>("tf_lookup_timeout_sec", 0.05);
    publish_fallback_before_tf_ = declare_parameter<bool>("publish_fallback_before_tf", false);
    z_filter_min_ = declare_parameter<double>(
        "z_filter_min", -std::numeric_limits<double>::infinity());
    z_filter_max_ = declare_parameter<double>(
        "z_filter_max", std::numeric_limits<double>::infinity());

    const auto sensor_qos = rclcpp::SensorDataQoS();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        raw_odom_topic_, sensor_qos,
        std::bind(&OdinFrameAdapter::handleOdom, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        raw_cloud_topic_, sensor_qos,
        std::bind(&OdinFrameAdapter::handleCloud, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 10);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, 2);
  }

private:
  bool lookupTransform(const std::string &source_frame, const rclcpp::Time &stamp,
                       geometry_msgs::msg::TransformStamped &transform) {
    try {
      transform = tf_buffer_.lookupTransform(
          map_frame_, source_frame, stamp,
          tf2::durationFromSec(tf_lookup_timeout_sec_));
      return true;
    } catch (const tf2::TransformException &) {
      try {
        transform = tf_buffer_.lookupTransform(
            map_frame_, source_frame, tf2::TimePointZero,
            tf2::durationFromSec(tf_lookup_timeout_sec_));
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                             "Using latest %s <- %s TF because stamped lookup failed.",
                             map_frame_.c_str(), source_frame.c_str());
        return true;
      } catch (const tf2::TransformException &error) {
        const std::string error_text = error.what();
        if (error_text.find("target_frame does not exist") != std::string::npos) {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "Frame %s is not in TF yet. Odin relocalization likely has not succeeded, "
              "so map-frame outputs are being held back.",
              map_frame_.c_str());
        } else {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                               "Waiting for TF %s <- %s: %s", map_frame_.c_str(),
                               source_frame.c_str(), error.what());
        }
        return false;
      }
    }
  }

  void handleOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (msg->header.frame_id == map_frame_) {
      odom_pub_->publish(*msg);
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    if (!lookupTransform(msg->header.frame_id, rclcpp::Time(msg->header.stamp), transform)) {
      if (publish_fallback_before_tf_) {
        odom_pub_->publish(*msg);
      }
      return;
    }

    geometry_msgs::msg::PoseStamped pose_in;
    pose_in.header = msg->header;
    pose_in.pose = msg->pose.pose;

    geometry_msgs::msg::PoseStamped pose_out;
    tf2::doTransform(pose_in, pose_out, transform);

    tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);
    tf2::Matrix3x3 rotation(q);

    tf2::Vector3 linear_in(
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.linear.z);
    tf2::Vector3 angular_in(
        msg->twist.twist.angular.x,
        msg->twist.twist.angular.y,
        msg->twist.twist.angular.z);

    const tf2::Vector3 linear_out = rotation * linear_in;
    const tf2::Vector3 angular_out = rotation * angular_in;

    nav_msgs::msg::Odometry output = *msg;
    output.header = pose_out.header;
    output.header.frame_id = map_frame_;
    output.pose.pose = pose_out.pose;
    output.twist.twist.linear.x = linear_out.x();
    output.twist.twist.linear.y = linear_out.y();
    output.twist.twist.linear.z = linear_out.z();
    output.twist.twist.angular.x = angular_out.x();
    output.twist.twist.angular.y = angular_out.y();
    output.twist.twist.angular.z = angular_out.z();
    odom_pub_->publish(output);
  }

  void handleCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (msg->header.frame_id == map_frame_) {
      cloud_pub_->publish(*msg);
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    if (!lookupTransform(msg->header.frame_id, rclcpp::Time(msg->header.stamp), transform)) {
      if (publish_fallback_before_tf_) {
        cloud_pub_->publish(*msg);
      }
      return;
    }

    pcl::PointCloud<pcl::PointXYZRGB> raw_cloud;
    pcl::fromROSMsg(*msg, raw_cloud);

    pcl::PointCloud<pcl::PointXYZI> cloud_in;
    cloud_in.reserve(raw_cloud.size());
    for (const auto &raw_point : raw_cloud.points) {
      pcl::PointXYZI point;
      point.x = raw_point.x;
      point.y = raw_point.y;
      point.z = raw_point.z;
      point.intensity = 0.0f;
      cloud_in.push_back(point);
    }

    tf2::Transform tf;
    tf2::fromMsg(transform.transform, tf);
    Eigen::Matrix4f tf_matrix = Eigen::Matrix4f::Identity();
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        tf_matrix(row, col) = static_cast<float>(tf.getBasis()[row][col]);
      }
      tf_matrix(row, 3) = static_cast<float>(tf.getOrigin()[row]);
    }

    pcl::PointCloud<pcl::PointXYZI> cloud_out;
    pcl::transformPointCloud(cloud_in, cloud_out, tf_matrix);

    if (std::isfinite(z_filter_min_) || std::isfinite(z_filter_max_)) {
      pcl::PointCloud<pcl::PointXYZI> filtered_cloud;
      filtered_cloud.reserve(cloud_out.size());
      for (const auto &point : cloud_out.points) {
        if (point.z < z_filter_min_ || point.z > z_filter_max_) {
          continue;
        }
        filtered_cloud.push_back(point);
      }
      cloud_out.swap(filtered_cloud);
    }

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(cloud_out, output);
    output.header.stamp = msg->header.stamp;
    output.header.frame_id = map_frame_;
    cloud_pub_->publish(output);
  }

  std::string raw_odom_topic_;
  std::string raw_cloud_topic_;
  std::string output_odom_topic_;
  std::string output_cloud_topic_;
  std::string map_frame_;
  double tf_lookup_timeout_sec_ = 0.05;
  bool publish_fallback_before_tf_ = false;
  double z_filter_min_ = -std::numeric_limits<double>::infinity();
  double z_filter_max_ = std::numeric_limits<double>::infinity();

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdinFrameAdapter>());
  rclcpp::shutdown();
  return 0;
}
