#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

struct MapConfig {
  std::string image_path;
  double resolution = 0.0;
  std::array<double, 3> origin = {0.0, 0.0, 0.0};
  int negate = 0;
  double occupied_thresh = 0.65;
  double free_thresh = 0.196;
};

struct PgmImage {
  int width = 0;
  int height = 0;
  int max_value = 255;
  std::vector<uint8_t> pixels;
};

std::string trim(const std::string &input) {
  const auto begin = input.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }

  const auto end = input.find_last_not_of(" \t\r\n");
  return input.substr(begin, end - begin + 1);
}

std::string strip_quotes(const std::string &input) {
  if (input.size() >= 2 &&
      ((input.front() == '"' && input.back() == '"') ||
       (input.front() == '\'' && input.back() == '\''))) {
    return input.substr(1, input.size() - 2);
  }

  return input;
}

std::string get_directory_name(const std::string &path) {
  const auto slash = path.find_last_of("/");
  if (slash == std::string::npos) {
    return ".";
  }

  if (slash == 0) {
    return "/";
  }

  return path.substr(0, slash);
}

std::string join_path(const std::string &base_dir, const std::string &relative_path) {
  if (relative_path.empty() || relative_path.front() == '/') {
    return relative_path;
  }

  if (base_dir.empty() || base_dir == ".") {
    return relative_path;
  }

  if (base_dir.back() == '/') {
    return base_dir + relative_path;
  }

  return base_dir + "/" + relative_path;
}

std::array<double, 3> parse_origin(const std::string &value) {
  const auto left = value.find('[');
  const auto right = value.find(']');
  if (left == std::string::npos || right == std::string::npos || right <= left) {
    throw std::runtime_error("Invalid origin format in map yaml.");
  }

  std::array<double, 3> origin = {0.0, 0.0, 0.0};
  std::stringstream ss(value.substr(left + 1, right - left - 1));
  std::string token;
  size_t index = 0;
  while (std::getline(ss, token, ',')) {
    if (index >= origin.size()) {
      throw std::runtime_error("Origin must contain exactly 3 values.");
    }

    origin[index++] = std::stod(trim(token));
  }

  if (index != origin.size()) {
    throw std::runtime_error("Origin must contain exactly 3 values.");
  }

  return origin;
}

MapConfig load_map_config(const std::string &yaml_path) {
  std::ifstream yaml_file(yaml_path);
  if (!yaml_file.is_open()) {
    throw std::runtime_error("Failed to open map yaml: " + yaml_path);
  }

  MapConfig config;
  std::string line;
  while (std::getline(yaml_file, line)) {
    const auto comment_pos = line.find('#');
    if (comment_pos != std::string::npos) {
      line = line.substr(0, comment_pos);
    }

    line = trim(line);
    if (line.empty()) {
      continue;
    }

    const auto colon_pos = line.find(':');
    if (colon_pos == std::string::npos) {
      continue;
    }

    const std::string key = trim(line.substr(0, colon_pos));
    const std::string raw_value = trim(line.substr(colon_pos + 1));

    if (key == "image") {
      config.image_path = strip_quotes(raw_value);
    } else if (key == "resolution") {
      config.resolution = std::stod(raw_value);
    } else if (key == "origin") {
      config.origin = parse_origin(raw_value);
    } else if (key == "negate") {
      config.negate = std::stoi(raw_value);
    } else if (key == "occupied_thresh") {
      config.occupied_thresh = std::stod(raw_value);
    } else if (key == "free_thresh") {
      config.free_thresh = std::stod(raw_value);
    }
  }

  if (config.image_path.empty()) {
    throw std::runtime_error("Map yaml is missing 'image'.");
  }
  if (config.resolution <= 0.0) {
    throw std::runtime_error("Map yaml has invalid 'resolution'.");
  }

  config.image_path = join_path(get_directory_name(yaml_path), config.image_path);
  return config;
}

std::string next_pgm_token(std::ifstream &input) {
  std::string token;
  char ch = '\0';

  while (input.get(ch)) {
    if (std::isspace(static_cast<unsigned char>(ch))) {
      continue;
    }

    if (ch == '#') {
      std::string ignored_line;
      std::getline(input, ignored_line);
      continue;
    }

    token.push_back(ch);
    break;
  }

  if (token.empty()) {
    return token;
  }

  while (input.get(ch)) {
    if (std::isspace(static_cast<unsigned char>(ch))) {
      break;
    }

    if (ch == '#') {
      std::string ignored_line;
      std::getline(input, ignored_line);
      break;
    }

    token.push_back(ch);
  }

  return token;
}

PgmImage load_pgm_image(const std::string &image_path) {
  std::ifstream image_file(image_path, std::ios::binary);
  if (!image_file.is_open()) {
    throw std::runtime_error("Failed to open map image: " + image_path);
  }

  const std::string magic = next_pgm_token(image_file);
  if (magic != "P5" && magic != "P2") {
    throw std::runtime_error("Only PGM images in P5 or P2 format are supported: " + image_path);
  }

  PgmImage image;
  image.width = std::stoi(next_pgm_token(image_file));
  image.height = std::stoi(next_pgm_token(image_file));
  image.max_value = std::stoi(next_pgm_token(image_file));

  if (image.width <= 0 || image.height <= 0) {
    throw std::runtime_error("Invalid image dimensions in map image: " + image_path);
  }
  if (image.max_value <= 0 || image.max_value > 255) {
    throw std::runtime_error("Only max_value <= 255 is supported in map image: " + image_path);
  }

  image.pixels.resize(static_cast<size_t>(image.width) * static_cast<size_t>(image.height));

  if (magic == "P5") {
    // next_pgm_token() already consumes the trailing whitespace/comment delimiter
    // after max_value, so binary pixel data begins at the current stream position.
    image_file.read(reinterpret_cast<char *>(image.pixels.data()),
                    static_cast<std::streamsize>(image.pixels.size()));
    if (image_file.gcount() != static_cast<std::streamsize>(image.pixels.size())) {
      throw std::runtime_error("Unexpected end of file while reading map image: " + image_path);
    }
  } else {
    for (size_t index = 0; index < image.pixels.size(); ++index) {
      const std::string token = next_pgm_token(image_file);
      if (token.empty()) {
        throw std::runtime_error("Unexpected end of file while reading ASCII map image: " + image_path);
      }

      image.pixels[index] = static_cast<uint8_t>(std::stoi(token));
    }
  }

  return image;
}

int8_t to_occupancy_value(uint8_t pixel, int max_value, const MapConfig &config) {
  const double normalized = static_cast<double>(pixel) / static_cast<double>(max_value);
  const double occupancy = config.negate != 0 ? normalized : (1.0 - normalized);

  if (occupancy > config.occupied_thresh) {
    return 100;
  }
  if (occupancy < config.free_thresh) {
    return 0;
  }

  return -1;
}

}  // namespace

class StaticOccupancyMapPublisher : public rclcpp::Node {
public:
  StaticOccupancyMapPublisher() : Node("static_occupancy_map_publisher") {
    this->declare_parameter<std::string>("map_yaml_file", "");
    this->declare_parameter<std::string>("topic_name", "/map");
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<int>("publish_delay_ms", 1000);

    this->get_parameter("map_yaml_file", map_yaml_file_);
    this->get_parameter("topic_name", topic_name_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("publish_delay_ms", publish_delay_ms_);

    if (map_yaml_file_.empty()) {
      throw std::runtime_error("Parameter 'map_yaml_file' must not be empty.");
    }

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(topic_name_, qos);

    load_grid();

    const int delay_ms = std::max(1, publish_delay_ms_);
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(delay_ms),
        std::bind(&StaticOccupancyMapPublisher::publish_once, this));
  }

private:
  void load_grid() {
    const MapConfig config = load_map_config(map_yaml_file_);
    const PgmImage image = load_pgm_image(config.image_path);

    grid_.header.frame_id = frame_id_;
    grid_.info.resolution = static_cast<float>(config.resolution);
    grid_.info.width = static_cast<uint32_t>(image.width);
    grid_.info.height = static_cast<uint32_t>(image.height);
    grid_.info.origin.position.x = config.origin[0];
    grid_.info.origin.position.y = config.origin[1];
    grid_.info.origin.position.z = 0.0;

    const double half_yaw = config.origin[2] * 0.5;
    grid_.info.origin.orientation.x = 0.0;
    grid_.info.origin.orientation.y = 0.0;
    grid_.info.origin.orientation.z = std::sin(half_yaw);
    grid_.info.origin.orientation.w = std::cos(half_yaw);

    grid_.data.assign(static_cast<size_t>(image.width) * static_cast<size_t>(image.height), -1);
    for (int y = 0; y < image.height; ++y) {
      for (int x = 0; x < image.width; ++x) {
        const size_t image_index = static_cast<size_t>(y) * static_cast<size_t>(image.width) + static_cast<size_t>(x);
        const int flipped_y = image.height - y - 1;
        const size_t map_index = static_cast<size_t>(flipped_y) * static_cast<size_t>(image.width) + static_cast<size_t>(x);
        grid_.data[map_index] = to_occupancy_value(image.pixels[image_index], image.max_value, config);
      }
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Loaded 2D map %s (%dx%d @ %.3fm) -> %s.",
        map_yaml_file_.c_str(),
        image.width,
        image.height,
        config.resolution,
        topic_name_.c_str());
  }

  void publish_once() {
    if (published_) {
      return;
    }

    const auto stamp = this->now();
    grid_.header.stamp = stamp;
    grid_.info.map_load_time = stamp;
    map_pub_->publish(grid_);
    published_ = true;

    RCLCPP_INFO(
        this->get_logger(),
        "Published latched occupancy grid on %s with transient_local durability.",
        topic_name_.c_str());

    publish_timer_->cancel();
  }

  std::string map_yaml_file_;
  std::string topic_name_;
  std::string frame_id_;
  int publish_delay_ms_ = 1000;
  bool published_ = false;

  nav_msgs::msg::OccupancyGrid grid_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<StaticOccupancyMapPublisher>());
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("static_occupancy_map_publisher"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}