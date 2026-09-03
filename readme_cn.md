<h3 align="center">Inertial-Only Spatiotemporal Calibration of Multiple IMUs Using Motion-Adaptive Non-Uniform B-Splines</h3>

<p align="center">
    <a href="readme.md">English</a> | 中文
</p>

<p align="center">
    <a href="https://github.com/CuiLongji1"><strong>维护者 » CuiLongji1</strong></a>
</p>

<p align="center"><i>面向异步多 IMU 系统的纯惯性时空标定方法。</i></p>

<p align="center">
    <img src="https://img.shields.io/badge/ROS-Noetic-22314E" alt="ROS Noetic">
    <img src="https://img.shields.io/badge/C++-17-00599C" alt="C++17">
    <img src="https://img.shields.io/badge/Spline-Non--Uniform_B--Spline-2EA44F" alt="Non-uniform B-spline">
    <img src="https://img.shields.io/badge/Calibration-Multi--IMU-D73A49" alt="Multi-IMU calibration">
</p>

---

本仓库提供一种面向刚性连接异步多 IMU 系统的纯惯性时空标定方法。该方法仅使用陀螺仪和加速度计的原始测量，即可估计参考 IMU 与多个辅助 IMU 之间的相对旋转、相对平移和时间偏移。

该方法采用运动自适应非均匀 B 样条描述参考 IMU 的连续时间运动。系统根据参考 IMU 的局部运动变化调整节点密度：在高动态区间布置更密集的样条段，在平稳区间减少控制点数量。完整流程包括运动激励评估、分阶段初始化、连续时间批量优化，以及初始化结果与最终结果的一致性评估。

<div align="center">
    <img src="img/framework.png" alt="多 IMU 时空标定流程图" width="70%">
</div>

## 主要特点

这是一种面向刚性连接异步多 IMU 系统的纯惯性时空标定方法，主要特点如下：

+ ***纯惯性：*** 仅使用陀螺仪和加速度计的原始测量；不需要相机、LiDAR 和标定靶。
+ ***多 IMU 时空标定：*** 估计参考 IMU 与每个辅助 IMU 之间的相对旋转、相对平移和时间偏移。
+ ***运动自适应非均匀 B 样条：*** 根据陀螺仪和加速度计测量的局部变化，确定每个时间窗口内的样条节点数。
+ ***分阶段初始化与联合优化：*** 分阶段初始化轨迹、重力、外参和时间偏移，再通过连续时间批量优化联合更新这些参数。

更多方法细节请参阅配套论文：

+ **“Inertial-Only Spatiotemporal Calibration of Multiple IMUs Using Motion-Adaptive Non-Uniform B-Splines.”** 论文正式公开后，将在此补充公开链接和完整的 BibTeX 引用信息。

> **代码发布：** 源代码将在论文接收后公开。

---

<p align="left">
    <a href="#build"><strong>教程：构建标定程序 »</strong></a>
</p>

+ 安装 ROS Noetic 和所需第三方库；
+ 克隆本仓库及其子模块；
+ 编译项目内的 CTraj 依赖和 ROS 工作空间。

<p align="left">
    <a href="#run"><strong>教程：运行多 IMU 标定 »</strong></a>
</p>

+ 准备 ROS bag 和各 IMU 的内参文件；
+ 配置参考 IMU、消息话题、时间偏移范围和自适应节点参数；
+ 启动标定程序并检查输出文件。

<a id="build"></a>

## 1. 构建标定程序

### 1.1 环境依赖

+ Ubuntu 20.04 和 ROS Noetic；
+ CMake 3.16 或更高版本，以及支持 C++17 的编译器；
+ Ceres Solver 2.2 或更高版本，并包含 `Manifold` 接口；
+ Sophus、Pangolin、magic_enum、cereal、yaml-cpp 和 spdlog；
+ ROS 软件包 `rosbag`、`roscpp`、`rospy`、`sensor_msgs`、`std_msgs` 和 `sbg_driver`。

通过 Ubuntu 软件包管理器安装 cereal、yaml-cpp 和 spdlog：

```bash
sudo apt-get update
sudo apt-get install libcereal-dev libyaml-cpp-dev libspdlog-dev
```

请安装与 ROS Noetic 兼容的 Ceres、Sophus、Pangolin 和 magic_enum。编译 Sophus 时，可启用 `SOPHUS_USE_BASIC_LOGGING`，以避免外部 `fmt` 与 spdlog 发生潜在冲突。

### 1.2 克隆与编译

创建 catkin 工作空间，并将本软件包克隆为 `mi_calib`：

```bash
source /opt/ros/noetic/setup.bash

mkdir -p ~/multi_imu_calib_ws/src
cd ~/multi_imu_calib_ws/src

git clone --recursive https://github.com/CuiLongji1/MI-NURBS-Calib.git mi_calib
git clone https://github.com/SBG-Systems/sbg_ros_driver.git
```

编译项目内的第三方库：

```bash
cd ~/multi_imu_calib_ws/src/mi_calib
chmod +x build_thirdparty.sh
./build_thirdparty.sh
```

该脚本会在项目目录内编译 CTraj 和 tiny-viewer，不会将它们安装到 `/usr/local`。

编译 ROS 工作空间：

```bash
cd ~/multi_imu_calib_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

<a id="run"></a>

## 2. 运行标定

### 2.1 准备数据

运行标定至少需要：

+ 两个刚性连接的 IMU；
+ 包含带时间戳原始 IMU 消息的 ROS bag；
+ 每个 IMU 对应的预标定内参文件；
+ 具有充分多轴旋转和平移激励的运动数据。

当前支持以下 ROS 消息类型：

+ `SENSOR_IMU`：`sensor_msgs/Imu`；
+ `SBG_IMU`：由 `sbg_driver` 提供的消息类型。

选定的参考 IMU 用于定义机体坐标系和时间基准。参考 IMU 的位姿及时间偏移固定不变，其他辅助 IMU 的参数均相对于参考 IMU 进行估计。

### 2.2 配置标定程序

复制 [config/example-non-uniform.yaml](config/example-non-uniform.yaml)，并修改以下字段：

+ `IMUTopics`：每个 IMU 的话题、消息类型和内参文件；
+ `ReferIMU`：参考 IMU 的话题；
+ `BagPath`：输入 ROS bag 的路径；
+ `OutputPath`：标定结果输出目录；
+ `TimeOffsetPadding`：允许的时间偏移范围及残差因子支撑范围；
+ `KnotTimeDist`：自适应非均匀样条参数。

自适应节点参数如下：

| 字段 | 说明 |
|---|---|
| `Adaptive` | 是否启用基于运动状态的样条段分配。 |
| `WindowTime` | 局部运动统计窗口的时长。 |
| `MinSegments` | 单个窗口内的最少样条段数。 |
| `MaxSegments` | 单个窗口内的最多样条段数。 |
| `GyroStdScale` | 陀螺仪变化量的饱和尺度。 |
| `AcceStdScale` | 加速度计变化量的饱和尺度。 |
| `GyroWeight` | 归一化陀螺仪变化量的权重。 |
| `AcceWeight` | 归一化加速度计变化量的权重。 |

当前代码使用四舍五入的线性映射，将归一化运动强度转换为 `MinSegments` 与 `MaxSegments` 之间的整数。由于旋转样条和加速度样条共用同一组节点，因此 `SO3Spline` 与 `LinAcceSpline` 必须设置为相同数值。

### 2.3 启动程序

```bash
source ~/multi_imu_calib_ws/devel/setup.bash

roslaunch mi_calib mi-calib-prog.launch \
    config_path:=/absolute/path/to/config.yaml
```

标定参数将以 `calibration_non_uniform.<format>` 的名称保存到 `OutputPath`。启用 `OutputBSplines` 和 `OutputKinematics` 后，程序还会输出样条及采样后的运动学数据。

## 3. 注意事项

+ 数据采集期间，所有 IMU 必须保持刚性连接。
+ 应充分激励三个轴向的旋转，并包含变化的角速度。单纯增加样条密度无法恢复不可观参数。
+ 检查每个 IMU 的时间戳单位、时间戳定义、话题顺序和原始数据约定。
+ `TimeOffsetPadding` 应覆盖预期的时间偏移绝对值；设置过大会增加计算量。
+ 建议预先完成各 IMU 的内参标定。
+ 示例配置仅用于说明程序用法，并非论文全部实验的逐项复现配置。

## 4. 引用与致谢

如果本项目对您的研究有所帮助，请在论文公开引用信息发布后引用配套论文：

> **“Inertial-Only Spatiotemporal Calibration of Multiple IMUs Using Motion-Adaptive Non-Uniform B-Splines.”**

本工作基于以下连续时间多 IMU 标定框架进行开发：

+ **S. Chen**, X. Li, S. Li, Y. Zhou, and S. Wang, “MI-Calib: An Open-Source Spatiotemporal Calibrator for Multiple IMUs Based on Continuous-Time Batch Optimization,” *IEEE Robotics and Automation Letters*, 2024. [[论文](https://ieeexplore.ieee.org/document/10629054)] [[上游代码](https://github.com/Unsigned-Long/MI-Calib)]

本实现还使用并修改了 [CTraj](https://github.com/Unsigned-Long/CTraj) 和 [tiny-viewer](https://github.com/Unsigned-Long/tiny-viewer) 中的部分组件。请保留所有上游版权、引用和许可证声明。

## 5. 许可证

仓库整体采用 MIT 许可证。部分上游源文件保留其原有 BSD 风格条款，各子模块遵循各自的许可证。详情请参阅 [LICENSE](LICENSE) 和 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

---

<p align="center"><i>
    Inertial-Only Spatiotemporal Calibration of Multiple IMUs Using Motion-Adaptive Non-Uniform B-Splines<br>
    维护者：<a href="https://github.com/CuiLongji1">CuiLongji1</a><br>
    基于运动自适应非均匀 B 样条的多 IMU 时空标定
</i></p>
