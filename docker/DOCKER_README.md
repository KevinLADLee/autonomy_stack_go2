# Docker部署指南 - Jetson Orin NX

本指南说明如何在Jetson Orin NX (ARM64架构, Ubuntu 20.04 + ROS2 Foxy)上使用Docker部署autonomy_stack_go2项目，并在容器内运行ROS2 Humble环境。

## 前置要求

1. **Jetson Orin NX** 已安装Ubuntu 20.04
2. **Docker** 已安装并配置
3. **Docker Compose** 已安装（或使用Docker内置的compose插件）
4. **NVIDIA Container Toolkit** 已安装（用于GPU支持）

### 安装NVIDIA Container Toolkit

对于Jetson设备，需要安装nvidia-container-toolkit以支持GPU加速：

```bash
# 方法1: 使用NVIDIA官方源（推荐）
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker

# 方法2: 对于Jetson，也可以使用JetPack自带的nvidia-container-toolkit
# 通常JetPack已经包含了必要的组件，只需确保Docker配置正确
```

验证GPU支持：
```bash
docker run --rm --runtime=nvidia nvidia/cuda:11.0-base nvidia-smi
```

## Dockerfile选择

项目提供了两个Dockerfile：

- **Dockerfile** (默认): 在构建镜像时编译整个工作空间，适合生产环境
- **Dockerfile.dev**: 不在镜像构建时编译，适合开发环境，可以更快地迭代

要使用开发版本，修改`docker/docker-compose.yml`中的`dockerfile: docker/Dockerfile`为`dockerfile: docker/Dockerfile.dev`

### 基础镜像说明

项目使用 `dustynv/ros:humble-desktop-l4t-r35.3.1` 作为基础镜像，这是专门为Jetson设备优化的ROS2 Humble镜像：
- 基于L4T (Linux for Tegra) r35.3.1
- 包含NVIDIA GPU优化和库
- 预配置了ROS2 Humble桌面环境
- 针对Jetson平台进行了性能优化

更多信息请参考: https://github.com/dusty-nv/jetson-containers

## 快速开始

### 方法1: 使用快速启动脚本（推荐）

```bash
# 1. 准备环境
chmod +x docker-setup.sh docker-run.sh
./docker-setup.sh

# 2. 构建并启动（一键部署）
./docker-run.sh rebuild

# 3. 进入容器
./docker-run.sh shell
```

### 方法2: 手动使用docker-compose

#### 1. 准备环境

运行设置脚本以准备Docker环境：

```bash
chmod +x docker-setup.sh
./docker-setup.sh
```

#### 2. 构建Docker镜像

使用docker-compose构建镜像（从项目根目录运行）：

```bash
# 使用docker-compose (旧版本)
docker-compose -f docker/docker-compose.yml build

# 或使用docker compose (新版本)
docker compose -f docker/docker-compose.yml build
```

#### 3. 启动容器

```bash
# 后台运行
docker-compose -f docker/docker-compose.yml up -d

# 或前台运行（查看日志）
docker-compose -f docker/docker-compose.yml up
```

#### 4. 进入容器

```bash
docker-compose -f docker/docker-compose.yml exec autonomy_stack bash
```

### 5. 在容器内运行系统

进入容器后，系统已经自动source了ROS2 Humble环境。你可以直接运行：

```bash
# 运行真实机器人系统
ros2 launch vehicle_simulator system_real_robot.launch.py

# 或使用提供的脚本
./system_real_robot.sh

# 运行带路径规划器的系统
./system_real_robot_with_route_planner.sh

# 运行带探索规划器的系统
./system_real_robot_with_exploration_planner.sh
```

## 配置说明

### GPU支持

容器已配置NVIDIA GPU支持，包括：
- **NVIDIA运行时**: `runtime: nvidia` - 启用GPU访问
- **环境变量**: 
  - `NVIDIA_VISIBLE_DEVICES=all` - 可见所有GPU设备
  - `NVIDIA_DRIVER_CAPABILITIES=all` - 启用所有驱动功能
  - `CUDA_VISIBLE_DEVICES=0` - 使用第一个GPU
- **库文件挂载**: 挂载Tegra和CUDA库以支持GPU加速

在容器内验证GPU：
```bash
nvidia-smi
# 或
cat /proc/driver/nvidia/version
```

### 网络配置

容器使用`host`网络模式，以便：
- ROS2 DDS通信正常工作
- 与Go2机器人通过以太网通信（IP: 192.168.123.100）

### USB设备访问

容器配置为privileged模式，并挂载了`/dev`目录，以便访问：
- L1激光雷达（通常为`/dev/ttyUSB0`）
- 游戏手柄（USB或蓝牙）
- 其他USB设备

### GUI支持（RViz）

容器配置了X11转发，支持运行RViz等GUI应用。确保：
1. 主机已安装X11
2. 运行`docker-setup.sh`创建X11认证文件
3. 设置`DISPLAY`环境变量（通常在`.bashrc`中）

### 数据持久化

以下目录被挂载到容器中，数据会持久保存：
- `./src` - 源代码
- `./install` - 编译后的安装文件
- `./build` - 编译中间文件
- `./log` - 日志文件
- `./bags` - ROS2 bag文件
- `~/Desktop` - 桌面目录（用于IMU校准数据）

## 常用命令

### 使用快速启动脚本

```bash
./docker-run.sh build      # 构建镜像
./docker-run.sh up         # 启动容器
./docker-run.sh down       # 停止容器
./docker-run.sh restart    # 重启容器
./docker-run.sh shell      # 进入容器
./docker-run.sh logs       # 查看日志
./docker-run.sh rebuild    # 重新构建并启动
./docker-run.sh status     # 查看状态
```

### 使用docker-compose直接命令

```bash
# 查看容器日志
docker-compose -f docker/docker-compose.yml logs -f autonomy_stack

# 停止容器
docker-compose -f docker/docker-compose.yml down

# 重启容器
docker-compose -f docker/docker-compose.yml restart
```

### 重新构建（代码更新后）

```bash
# 停止容器
docker-compose down

# 重新构建
docker-compose build --no-cache

# 启动
docker-compose up -d
```

### 在容器内重新编译

如果使用`Dockerfile.dev`或需要重新编译代码：

```bash
# 进入容器
docker-compose -f docker/docker-compose.yml exec autonomy_stack bash

# 在容器内编译
cd /workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 或者只编译特定包
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select local_planner
```

## 环境变量

可以通过环境变量或`docker/docker-compose.yml`配置：

- `ROS_DOMAIN_ID`: ROS2域ID（默认: 0）
- `RMW_IMPLEMENTATION`: RMW实现（默认: rmw_cyclonedds_cpp）
- `DISPLAY`: X11显示（默认: :0）

## 故障排除

### 1. X11/GUI应用无法显示

确保：
- 运行了`docker-setup.sh`
- `DISPLAY`环境变量正确设置
- X11权限正确

```bash
xhost +local:docker
```

### 2. USB设备无法访问

检查设备权限：
```bash
ls -l /dev/ttyUSB*
```

如果权限不足，可能需要将用户添加到dialout组：
```bash
sudo usermod -aG dialout $USER
```

### 3. 网络通信问题

确保：
- 容器使用`host`网络模式
- 主机网络配置正确（IP: 192.168.123.100）
- ROS_DOMAIN_ID匹配

### 4. 编译错误

如果遇到编译错误：
1. 确保所有依赖已安装
2. 清理build目录：`rm -rf build install log`
3. 重新编译

### 5. GPU无法访问

如果容器内无法访问GPU：
1. 确认已安装nvidia-container-toolkit
2. 检查Docker配置：`docker info | grep nvidia`
3. 验证运行时：`docker run --rm --runtime=nvidia nvidia/cuda:11.0-base nvidia-smi`
4. 重启Docker服务：`sudo systemctl restart docker`

### 6. 性能问题

Jetson Orin NX资源有限，如果遇到性能问题：
- 确保GPU加速已启用（检查nvidia-smi）
- 关闭不必要的服务
- 使用`nice`命令降低进程优先级
- 检查CPU和GPU温度，确保没有过热降频

## 注意事项

1. **首次构建可能需要较长时间**（30-60分钟），因为需要编译整个ROS2工作空间
2. **确保有足够的磁盘空间**（至少10GB）
3. **IMU校准数据**需要放在`~/Desktop/imu_calib_data.yaml`
4. **Unity仿真文件**（如果使用）需要放在`src/base_autonomy/vehicle_simulator/mesh/unity/`
5. **网络配置**：确保主机IP设置为192.168.123.100（如果使用外部计算机）

## 与Go2机器人通信

如果使用外部计算机通过以太网连接Go2：
1. 确保主机IP设置为192.168.123.100
2. 在容器内source unitree设置脚本（如果使用）
3. 使用`ros2 topic list`验证DDS通信

## 更新项目

当源代码更新后：

```bash
# 方法1: 在容器内重新编译
docker-compose -f docker/docker-compose.yml exec autonomy_stack bash
cd /workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 方法2: 重新构建镜像（如果Dockerfile有变化）
docker-compose -f docker/docker-compose.yml build --no-cache
docker-compose -f docker/docker-compose.yml up -d
```

## 支持

如有问题，请参考：
- 项目主README.md
- ROS2 Humble文档
- Docker文档

