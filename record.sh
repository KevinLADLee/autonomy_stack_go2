#!/bin/bash

# 数据录制脚本 - 录制机器人系统的关键 topics
# 使用 MCAP 格式进行录制，支持更好的压缩和跨平台兼容性

set -e  # 遇到错误立即退出

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 设置 ROS 环境（根据实际情况调整路径）
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/foxy/setup.bash ]; then
  source /opt/ros/foxy/setup.bash
else
  echo "警告: 未找到 ROS 环境，请手动 source ROS setup.bash"
fi

# Source workspace（如果存在）
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
  source "$SCRIPT_DIR/install/setup.bash"
fi

# 创建输出目录
OUTPUT_DIR="$SCRIPT_DIR/bags"
mkdir -p "$OUTPUT_DIR"

# 生成带时间戳的文件名
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_NAME="robot_data_${TIMESTAMP}"

echo "=========================================="
echo "开始录制数据到: $OUTPUT_DIR/$BAG_NAME"
echo "按 Ctrl+C 停止录制"
echo "=========================================="

# 使用 MCAP 格式录制，包含所有关键 topics
ros2 bag record -s mcap \
  -o "$OUTPUT_DIR/$BAG_NAME" \
  /added_obstacles \
  /api/sport/request \
  /check_obstacle \
  /cmd_vel \
  /autonomy_stack/free_paths \
  /goal_pose \
  /joint_states \
  /lowstate \
  /map_clearing \
  /navigation_boundary \
  /odin/cloud_raw \
  /odin1/camera_pose_visual \
  /odin1/cloud_render \
  /odin1/image \
  /odin1/image/compressed \
  /odin1/imu \
  /odin1/odometry_highfreq \
  /odin1/path \
  /overall_map \
  /autonomy_stack/path \
  /registered_scan \
  /robot_description \
  /rosout \
  /speed \
  /state_estimation \
  /stop \
  /terrain_map \
  /tf \
  /tf_static \
  /way_point \
  /wirelesscontroller

echo ""
echo "=========================================="
echo "录制完成！数据保存在: $OUTPUT_DIR/$BAG_NAME"
echo "=========================================="
