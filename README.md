# ⚠️ 分支说明 / Branch Information

**当前分支 / Current Branch:** `foxy-humble-odin-neupan`

本分支基于原始的 `foxy-humble` 分支，集成了 **ODIN 激光雷达驱动** 和 **NeuPAN 深度学习导航规划器**，并进行了多项改进。

## 🔄 主要改动 / Key Changes

### ✨ 新增功能 / New Features

1. **ODIN 激光雷达支持 / ODIN LiDAR Support**
  - 新增包：`odin_ros_driver`
  - 功能更新：提供 ODIN 传感器 ROS2 驱动能力，支持点云/深度图发布、建图参数控制、地图保存与重定位流程
  - 项目链接 / Project Links: [odin_ros_driver](https://github.com/manifoldsdk/odin_ros_driver), [Odin1 Wiki](https://manifoldtechltd.github.io/wiki/Odin1/Cover.html)
  - 结果：当前分支以 ODIN 传感链路为主，不再依赖原先独立 IMU 校准包

2. **NeuPAN 深度学习导航 / NeuPAN Deep Learning Navigation**
  - 新增包：`neupan_ros2`（含 `ddr_minimal_sim`）
  - 功能更新：接入学习型导航规划器，支持多机器人参数化配置（Go2/Scout/Ranger/Limo/Simulation）
  - 功能更新：提供最小仿真环境、激光模拟和可视化链路，便于离线验证规划策略
  - 结果：新增一套可并行于传统规划器的端到端导航方案

3. **TARE 自主探索规划器 / TARE Exploration Planner**
  - 新增包：`tare_planner`、`tare_tsp_interfaces`
  - 功能更新：增加未知区域自主探索、覆盖路径生成、边界发布、可视化输出
  - 功能更新：通过 TSP 服务接口对探索访问顺序进行优化，减少无效往返
  - 结果：系统从“导航到目标点”扩展为“可主动探索未知区域”

4. **Docker 支持 / Docker Support**
  - 新增容器化支持（开发镜像 + 运行镜像 + compose）
  - 功能更新：统一依赖环境和启动流程，减少不同机器上的环境漂移问题
  - 结果：更适合在本地与 Jetson 上复现和部署

### 🔧 功能改进 / Feature Improvements

1. **局部规划器重构 / Local Planner Refactoring**
  - 相关包：`local_planner`
  - 功能更新：重构局部规划与跟踪逻辑，统一使用 `maxSpeed` 进行速度约束
  - 功能更新：新增路径文件读写能力，支持更稳定的路径复用与调试
  - 功能修复：修正点云坐标变换问题，降低局部规划偏差

2. **Go2 机器人集成 / Go2 Robot Integration**
  - 相关包：`go2_description`、`go2_sport_api`
  - 功能更新：补齐 Go2 机器人描述（URDF/Xacro/Mesh/RViz），提升模型一致性
  - 功能更新：重构运动控制接口与机器人状态客户端，新增关节状态发布
  - 结果：真机控制与状态反馈链路更完整、可观测性更高

3. **系统启动脚本更新 / System Launch Scripts Update**
  - 相关模块：`vehicle_simulator` 启动系统
  - 功能更新：将多套系统启动入口迁移到 ROS2 Python launch
  - 功能更新：拆分常规导航 / Route Planner / Exploration / NeuPAN 的独立启动流程
  - 结果：按场景切换更明确，系统集成调试更方便

4. **消息和配置更新 / Messages and Configuration Updates**
  - 相关包：`unitree_hg`、`unitree_go`、`far_planner`
  - 功能更新：扩展底层机器人消息定义（电机、IMU、低层状态等），提升状态表达粒度
  - 功能更新：补充 `unitree_go` 电机消息，增强控制与状态对齐
  - 功能更新：调整 FAR Planner 参数配置，使其更匹配当前传感器与系统链路
  - 结果：通信接口更完整，规划参数更贴合当前分支架构

### 🗑️ 移除模块 / Removed Modules

- `src/slam/point_lio_unilidar/` - 移除旧 SLAM 模块
- `src/ros_tcp_endpoint/` - 移除 Unity ROS-TCP 通道依赖
- `src/utilities/calibrate_imu/` - 独立 IMU 校准模块移除（校准流程并入 ODIN 方案）
- `src/utilities/transform_sensors/` - 独立传感器变换模块移除（能力并入新架构）

### 📝 其他改进 / Other Improvements

- 添加记录脚本 (`record.sh`)
- 优化编译脚本 (`build_odin.sh`)
- Docker 自动化部署脚本
- 改进 RVIZ 配置文件
- 增强可视化工具

---

## About

The repository contains the full autonomy stack for the [Unitree Go2 platform](https://shop.unitree.com/products/unitree-go2). The system provides a navigation capability of taking a goal point and navigating Go2 autonomously to the goal point while building a map along the way. Alternatively, the system allows users to use a joystick controller to guide the navigation while the system itself is in charge of collision avoidance. The system uses only built-in sensors on Go2 - the L1 lidar and the IMU in the lidar. The system contains a SLAM module, a route planner, and a based autonomy system, where the base autonomy system further includes fundamental navigation modules for terrain traversability analysts, collision avoidance, and waypoint following. Users can run the system on the onboard computer or an external computer connected to Go2 via an Ethernet cable. Please make sure to use the **Go2 EDU version** which has SDK support.

<p align="center">
  <img src="img/go2_sensors.jpg" alt="Go2 Sensors" width="40%"/>
</p>

## Simulation Setup

### Base Autonomy

The system is integrated with [Unity](https://unity.com) environment models. For simulation, the repository has been tested in Ubuntu 22.04 with [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation.html) and Ubuntu 22.04 with [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html). Install dependencies with the command lines below.

For Foxy:
```
sudo apt update
sudo apt install libusb-dev ros-foxy-perception-pcl ros-foxy-sensor-msgs-py ros-foxy-tf-transformations ros-foxy-joy ros-foxy-rmw-cyclonedds-cpp ros-foxy-rosidl-generator-dds-idl python3-colcon-common-extensions python-is-python3 
pip install transforms3d pyyaml
```
For Humble:
```
sudo apt update
sudo apt install libusb-dev ros-humble-perception-pcl ros-humble-sensor-msgs-py ros-humble-tf-transformations ros-humble-joy ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-generator-dds-idl python3-colcon-common-extensions python-is-python3 
pip install transforms3d pyyaml
```
Clone the open-source repository.
```
git clone https://github.com/jizhang-cmu/autonomy_stack_go2.git

```
In a terminal, go to the folder and compile.
```
cd autonomy_stack_go2
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Download a [Unity environment model for Go2](https://drive.google.com/drive/folders/11GhvA8Jz1RnRSGfiQ_MDJ4X-aNMpQPPx?usp=drive_link) and unzip to the 'src/base_autonomy/vehicle_simulator/mesh/unity' folder. The environment model files should look like below.

mesh/<br>
&nbsp;&nbsp;&nbsp;&nbsp;unity/<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;environment/<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Model_Data/ (multiple files in the folder)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Model.x86_64<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;UnityPlayer.so<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;AssetList.csv (generated at runtime)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Dimensions.csv<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Categories.csv<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;map.ply<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;object_list.txt<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;traversable_area.ply<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;map.jpg<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;render.jpg<br>

In a terminal, go to the repository folder and launch the system.
```
./system_simulation.sh
```
After seeing data showing up in RVIZ, users can use the 'Waypoint' button to set waypoints and navigate the vehicle around. Note that the waypoints are meant to be relatively close to the vehicle. Setting the waypoint too far can cause the vehicle to get stuck at a dead end. Users can also operate in *smart joystick mode* where the vehicle tries to follow joystick commands and also avoid collisions. To do this, users can use the control panel in RVIZ or a PS3/4 or Xbox controller with a USB or Bluetooth interface. When using the controller, users can also operate in *manual mode* without any collision avoidance. Detailed information about the operations in the three modes is below.

<p align="center">
  <img src="img/rviz_full.jpg" alt="RVIZ" width="80%"/>
</p>

- *Smart joystick mode (default)*: The vehicle tries to follow joystick commands and also avoid collisions. Use the control panel in RVIZ or the right joystick on the controller to set the speed and yaw rate. If the system is in another mode, doing so will switch the system to *smart joystick mode*.

- *Waypoint mode*: The vehicle tries to follow waypoints and also avoid collisions. Use the 'Waypoint' button in RVIZ to set a waypoint by first clicking the button and then clicking where the waypoint is to be set around the vehicle. If the system is in another mode, clicking the 'Resume Navigation to Goal' button in RVIZ switches the system to *waypoint mode*. Or, users can hold the 'waypoint-mode' button on the controller and use the right joystick to set the speed. If only holding the 'waypoint-mode' button, the system will use the speed sent in ROS messages.

- *Manual mode*: The vehicle tries to follow joystick commands without any collision avoidance. Pressing the 'manual-mode' button on the controller switches the system to *manual mode*. Then, use the right joystick to set the forward and lateral speed and the left joystick to set the yaw rate, in the Mode 2 convention.

<p align="center">
  <img src="img/rviz_control_panel.jpg" alt="RVIZ Control Panel" width="30%"/>
  &nbsp;&nbsp;&nbsp;&nbsp;
  <img src="img/ps3_controller.jpg" alt="PS3 Controller" width="45%"/>
</p>

Alternatively, users can run a ROS node to send a series of waypoints. In another terminal, go to the folder and source the ROS workspace, then run the ROS node with the command lines below. The ROS node sends navigation boundary and speed as well. Click the 'Resume Navigation to Goal' button in RVIZ, and the vehicle will navigate inside the boundary following the waypoints. More information about the base autonomy system is available on the [Autonomous Exploration Development Environment](https://www.cmu-exploration.com) website.
```
source install/setup.sh
ros2 launch waypoint_example waypoint_example.launch
```

### Route Planner

To launch the system with route planner, use the command line below.
```
./system_simulation_with_route_planner.sh
```
Users can send a goal point with the 'Goalpoint' button in RVIZ. The vehicle will navigate to the goal and build a visibility graph (in cyan) along the way. Areas covered by the visibility graph become free space. When navigating in free space, the planner uses the built visibility graph, and when navigating in unknown space, the planner attempts to discover a way to the goal. By pressing the 'Reset Visibility Graph' button, the planner will reinitialize the visibility graph. By unchecking the 'Planning Attemptable' checkbox, the planner will first try to find a path through the free space. The path will show in green. If such a path does not exist, the planner will consider unknown space together. The path will show in blue. By unchecking the 'Update Visibility Graph' checkbox, the planner will stop updating the visibility graph. When navigating with the route planner, the base system operates in *waypoint mode*. Users can click in the black box on the control panel to switch to *smart joystick mode*, or press the buttons on a joystick controller to switch to *smart joystick mode* or *manual mode*. To resume route planner navigation, click the 'Resume Navigation to Goal' button in RVIZ or use the 'Goalpoint' button to set a new goalpoint. More information about the route planner is available on the [FAR Planner website](https://github.com/MichaelFYang/far_planner).

<p align="center">
  <img src="img/rviz_full_with_route_planner.jpg" alt="RVIZ with Route Planner" width="80%"/>
</p>

## Real-robot Setup

### Hardware

We set up a quick and easy communication module on Go2 with a wireless HDMI transmitter and a pair of wireless keyboard and mouse. The communication module is attached to the back of Go2 and plugged into the onboard computer at the USB-C port. The signals are transmitted to a control station with a monitor. This setup allows us to work with Go2 similar to a desktop computer - the computer walks around in the environment and communicates with the control station wirelessly. Please see the images below for detailed information about the hardware setup.

<p align="center">
  <img src="img/go2_items.jpg" alt="Go2 Items" width="40%"/>
  &nbsp;&nbsp;&nbsp;&nbsp;
  <img src="img/station_items.jpg" alt="Station Items" width="40%"/>
</p>

<p align="center">
  <img src="img/all_items.jpg" alt="All Items" width="80%"/>
</p>

Below is a list of the hardware items with an overall price of about $300 (not including Go2 itself or the monitor).

- [Wireless HDMI transmitter](https://www.amazon.com/Wireless-Transmitter-Receiver-Streaming-Projector/dp/B09XQ65XWV)<br>
- [USB-C hub](https://www.amazon.com/gp/product/B0BR3M8XHK) <br>
- [PS3 controller](https://www.amazon.com/Wireless-PC-Controller-Joystick-Nintendo-Dual-Vibration/dp/B0B2R14VJB) <br>
- [WiFi adapter](https://www.amazon.com/dp/B00762YNMG) <br>
- [Wireless keyboard and mouse](https://www.amazon.com/dp/B079JLY5M5) <br>
- [USB-A to USB-C charging cable](https://www.amazon.com/dp/B08933P982) <br>
- [HDMI cable](https://www.amazon.com/dp/B08B36C6T6) <br>
- [Dual lock tape](https://www.amazon.com/Industrial-Reclosable-Boats-Cars-Drones-Indoor-Outdoor-Waterproof/dp/B09FYBZ4Z1) (For attaching the wireless HDMI transmitter and USB-C hub on Go2)

### System Setup

#### 1) On Onboard Computer

The onboard computer is pre-installed with Ubuntu 20.04 and ROS2 Foxy. Install dependencies using the command lines below.
```
sudo apt update
sudo apt install libusb-dev ros-foxy-perception-pcl ros-foxy-sensor-msgs-py ros-foxy-tf-transformations ros-foxy-joy ros-foxy-rmw-cyclonedds-cpp ros-foxy-rosidl-generator-dds-idl
pip install transforms3d pyyaml
```
Clone the open-source repository.
```
git clone https://github.com/jizhang-cmu/autonomy_stack_go2.git

```
In a terminal, go to the folder and compile.
```
cd autonomy_stack_go2
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

#### 2) On External Computer

If using an external computer to run the system, it's recommended the computer runs Ubuntu 20.04 with ROS2 Foxy. Install dependencies and [follow instructions on the Unitree website]((https://support.unitree.com/home/en/developer/ROS2_service)) to set up DDS with cyclonedds. Note that the ROS2 environment **should not be sourced** when compiling cyclonedds.
```
sudo apt update
sudo apt install libusb-dev ros-foxy-perception-pcl ros-foxy-sensor-msgs-py ros-foxy-tf-transformations ros-foxy-joy ros-foxy-rmw-cyclonedds-cpp ros-foxy-rosidl-generator-dds-idl python3-colcon-common-extensions python-is-python3 gstreamer1.0-plugins-bad gstreamer1.0-libav
pip install transforms3d pyyaml
```
Connect the external computer to the Ethernet port pointing backward on Go2. Set the computer IP to 192.168.123.100, with netmask 255.255.255.0 and gateway 192.168.123.1 (tested and recommended). Then, **source the 'unitree_ros2_setup.sh' script** as mentioned in the Unitree instructions and use `ros2 topic list` to check if the DDS setup is correct. Note that the same script needs to be sourced in the terminal that launches the IMU calibration and the system in sections below. Next, clone and compile the repository in the same way as the onboard computer. Make sure the DDS setup is complete before compiling.

### IMU Calibration

The IMU inside the L1 lidar needs to be calibrated. This needs to be done once on every Go2. Power on Go2 and let it stand up. In a terminal, go to the repository folder and use the command lines below to source the ROS workspace and launch calibration. Let Go2 step for 2 seconds, stand for 10 seconds, and then spin for 20 seconds. After completion of the calibration, the result is saved to an 'imu_calib_data.yaml' file on the desktop. The file will be loaded into the system at the system startup. Please do not change the file name or directory. If using an external computer, **source the 'unitree_ros2_setup.sh' script** first as mentioned in the Unitree instructions. 
```
source install/setup.bash
ros2 run calibrate_imu calibrate_imu
```

<p align="center">
  <img src="img/stand_up.gif" alt="Power on and Stand up" width="40%"/>
  &nbsp;&nbsp;&nbsp;&nbsp;
  <img src="img/imu_calibration.gif" alt="IMU Calibration" width="40%"/>
</p>

### System Usage

In a terminal, go to the repository folder and use the command line below to launch the system. This launches the SLAM module and the base autonomy system. If using an external computer, **source the 'unitree_ros2_setup.sh' script** first as mentioned in the Unitree instructions.
```
./system_real_robot.sh
```
To launch the system with route planner, use the command line below instead.
```
./system_real_robot_with_route_planner.sh
```
Users can follow the same operations as in the simulation setup to navigate Go2 in the environment. Please refer to the Simulation Setup section for operating the system in the *smart joystick mode*, *waypoint mode*, and *manual mode*. To launch the camera driver, in a separate terminal, go to the repository folder and use the command lines below. The camera driver takes an H.264 video stream and publishes images on the '/camera/image/raw' topic.
```
source install/setup.bash
ros2 run go2_h264_repub go2_h264_repub
```
To record the lidar and IMU data to a bagfile, use the command line below in another terminal while the system is running.
```
ros2 bag record /utlidar/cloud /utlidar/imu
```
Users can copy the bagfile to an external computer. Please copy the 'imu_calib_data.yaml' file as well and put it on the external computer desktop. [Example bagfile and 'imu_calib_data.yaml' file](https://drive.google.com/drive/folders/11GhvA8Jz1RnRSGfiQ_MDJ4X-aNMpQPPx?usp=drive_link) are provided. Launch the system with the same command lines above. In another terminal, play the bagfile. Note that the computer **should not connect to Go2** via Ethernet or the onboard sensor data may come in.
```
ros2 bag play bagfile_name.db3
```

<p align="center">
  <img src="img/collision_avoidance.gif" alt="Collision Avoidance" width="80%"/>
</p>

## Notes

- The onboard computer is installed with Ubuntu 20.04. The password is ‘123’. After logging into the onboard computer, open a terminal, and you are asked to choose between ROS1 Noetic and ROS2 Foxy. Press ‘1’ to choose ROS2 Foxy to use the system.

- It happens that after powering on, the clock on the onboard computer is reset to 1970, causing issues with the system. Connecting the onboard computer to WiFi automatically updates the clock.

- The default IP of the Ethernet port pointing backward is 192.168.123.18, with netmask 255.255.255.0 and gateway 192.168.123.1. Changing the IP may interrupt the communication to the platform. Please keep it.

- When running the system on an external computer with Ubuntu 22.04 with ROS2 Humble, we experience large data delay (> 1s).

- When running the system on the onboard computer or an external computer with Ubuntu 20.04 and ROS2 Foxy, We occasionally observe data delay at the system startup. The delay lasts for 10-20s. A quick way to check the delay is using the joystick controller to change the Go2 heading while also checking the vehicle coordinate frame in RVIZ. The delay can be obvious (> 1s). Please wait for the delay to fade away before navigating Go2.

- The L1 lidar is a low-cost lidar with relatively high noise. With this lidar, the terrain reversibility analysis cannot distinguish low obstacles. Please make sure all obstacles in the environments are high enough above the ground (> 0.3m).

- Occasionally, we observe SLAM drift which can cause issues with the terrain map and collision avoidance. Symptoms are Go2 getting stuck or moving in strange directions and not following waypoints. Users can press the 'clear-terrain-map button' on the joystick controller to reset the terrain map.

- The timestamps in the images are not synchronized with the lidar or IMU data, while the timestamps in the lidar and IMU data are synchronized to each other. Also, recording the images to a bagfile can cause significant frame loss.

- In simulation, the bridge between Unity and the system is not completely stable. At the system launch, if you see an error regarding 'ros_tcp_endpoint', simply try for a second time.

## Main Contributors

[Guofei Chen](https://gfchen01.cc), [Botao He](https://github.com/Bottle101), [Guanya Shi](https://www.gshi.me), and [Ji Zhang](https://frc.ri.cmu.edu/~zhangji)

## Credits

[point_lio_unilidar](https://github.com/unitreerobotics/point_lio_unilidar), [Unitree ROS2 drive](https://github.com/unitreerobotics/unitree_ros2), and [ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) packages are from open-source releases.

## Relevant Links

The SLAM module is based on [Point-LIO](https://github.com/hku-mars/Point-LIO).

The base autonomy system is based on [Autonomous Exploration Development Environment](https://www.cmu-exploration.com).

The route planner is based on [FAR Planner](https://github.com/MichaelFYang/far_planner).

The ODIN driver integration refers to [odin_ros_driver](https://github.com/manifoldsdk/odin_ros_driver) and [Odin1 Wiki](https://manifoldtehltd.github.io/wiki/Odin1/Cover.html).
