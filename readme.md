<h3 align="center">Inertial-Only Spatiotemporal Calibration of Multiple IMUs Using Motion-Adaptive Non-Uniform B-Splines</h3>

<p align="center">
    English | <a href="readme_cn.md">中文</a>
</p>

<p align="center">
    <a href="https://github.com/CuiLongji1"><strong>Maintainer » CuiLongji1</strong></a>
</p>

<p align="center"><i>Inertial-only spatiotemporal calibration for asynchronous multiple IMUs.</i></p>

<p align="center">
    <img src="https://img.shields.io/badge/ROS-Noetic-22314E" alt="ROS Noetic">
    <img src="https://img.shields.io/badge/C++-17-00599C" alt="C++17">
    <img src="https://img.shields.io/badge/Spline-Non--Uniform_B--Spline-2EA44F" alt="Non-uniform B-spline">
    <img src="https://img.shields.io/badge/Calibration-Multi--IMU-D73A49" alt="Multi-IMU calibration">
</p>

---

This repository provides an inertial-only spatiotemporal calibration framework for rigidly connected asynchronous IMUs. It estimates the relative rotations, translations, and time offsets between a reference IMU and multiple auxiliary IMUs using only raw gyroscope and accelerometer measurements.

The framework represents the reference motion with motion-adaptive non-uniform B-splines. Local knot density is adjusted according to the motion variation of the reference IMU: dynamic intervals use denser spline segments, while smooth intervals use fewer control points. The complete pipeline integrates motion-excitation assessment, staged initialization, continuous-time batch optimization, and initialization-to-final consistency evaluation.

<div align="center">
    <img src="img/framework.png" alt="Calibration framework" width="70%">
</div>

## Main Features

This is an inertial-only spatiotemporal calibration method for rigidly connected asynchronous IMUs. Its main features are:

+ ***Inertial-only:*** uses only raw gyroscope and accelerometer measurements; cameras, LiDAR, and calibration targets are not required.
+ ***Multi-IMU spatiotemporal calibration:*** estimates the relative rotation, translation, and time offset between the reference IMU and each auxiliary IMU.
+ ***Motion-adaptive non-uniform B-splines:*** sets the number of spline knots in each time window from the local variation of gyroscope and accelerometer measurements.
+ ***Staged initialization and joint optimization:*** initializes the trajectory, gravity, extrinsics, and time offsets in stages, then jointly refines them through continuous-time batch optimization.

For more details, please refer to the accompanying manuscript:

+ **“Inertial-Only Spatiotemporal Calibration of Multiple IMUs Using Motion-Adaptive Non-Uniform B-Splines.”** The public paper link and complete BibTeX entry will be added after the publication metadata is released.

> **Code release:** The source code will be released after the paper is accepted.

---

<p align="left">
    <a href="#build"><strong>Tutorial: Build the Calibration Framework »</strong></a>
</p>

+ install ROS Noetic and the required third-party libraries;
+ clone this repository and its submodules;
+ compile the project-local CTraj dependencies and ROS workspace.

<p align="left">
    <a href="#run"><strong>Tutorial: Run Multi-IMU Calibration »</strong></a>
</p>

+ prepare the ROS bag and IMU intrinsic files;
+ configure the reference IMU, topics, time-offset range, and adaptive-knot parameters;
+ launch calibration and inspect the output files.

<a id="build"></a>

## 1. Build the Calibration Framework

### 1.1 Requirements

+ Ubuntu 20.04 and ROS Noetic;
+ CMake 3.16 or newer and a C++17 compiler;
+ Ceres Solver 2.2 or newer with the `Manifold` interface;
+ Sophus, Pangolin, magic_enum, cereal, yaml-cpp, and spdlog;
+ ROS packages `rosbag`, `roscpp`, `rospy`, `sensor_msgs`, `std_msgs`, and `sbg_driver`.

Install cereal, yaml-cpp, and spdlog from the Ubuntu package manager:

```bash
sudo apt-get update
sudo apt-get install libcereal-dev libyaml-cpp-dev libspdlog-dev
```

Install Ceres, Sophus, Pangolin, and magic_enum using versions compatible with ROS Noetic. When compiling Sophus, enabling `SOPHUS_USE_BASIC_LOGGING` can avoid a possible external `fmt` conflict with spdlog.

### 1.2 Clone and Compile

Create a catkin workspace and clone the package as `mi_calib`:

```bash
source /opt/ros/noetic/setup.bash

mkdir -p ~/multi_imu_calib_ws/src
cd ~/multi_imu_calib_ws/src

git clone --recursive https://github.com/CuiLongji1/MI-NURBS-Calib.git mi_calib
git clone https://github.com/SBG-Systems/sbg_ros_driver.git
```

Build the project-local third-party libraries:

```bash
cd ~/multi_imu_calib_ws/src/mi_calib
chmod +x build_thirdparty.sh
./build_thirdparty.sh
```

The script builds CTraj and tiny-viewer inside the project tree and does not install them into `/usr/local`.

Build the ROS workspace:

```bash
cd ~/multi_imu_calib_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

<a id="run"></a>

## 2. Run Calibration

### 2.1 Prepare the Data

Calibration requires:

+ at least two rigidly connected IMUs;
+ a ROS bag containing timestamped raw IMU messages;
+ one pre-calibrated intrinsic file for each IMU;
+ sufficiently excited multi-axis rotational and translational motion.

The supported ROS message types are:

+ `SENSOR_IMU`: `sensor_msgs/Imu`;
+ `SBG_IMU`: messages provided by `sbg_driver`.

The selected reference IMU defines the body frame and temporal gauge. Its pose and time offset are fixed, and all auxiliary-IMU parameters are estimated relative to it.

### 2.2 Configure the Calibrator

Copy [config/example-non-uniform.yaml](config/example-non-uniform.yaml) and update the following fields:

+ `IMUTopics`: topic, message type, and intrinsic file for every IMU;
+ `ReferIMU`: reference-IMU topic;
+ `BagPath`: input ROS bag;
+ `OutputPath`: calibration output directory;
+ `TimeOffsetPadding`: permitted time-offset range and factor support;
+ `KnotTimeDist`: adaptive non-uniform spline settings.

The adaptive-knot parameters are:

| Field | Description |
|---|---|
| `Adaptive` | Enables motion-dependent segment allocation. |
| `WindowTime` | Duration of each local motion-statistics window. |
| `MinSegments` | Minimum number of spline segments in one window. |
| `MaxSegments` | Maximum number of spline segments in one window. |
| `GyroStdScale` | Saturation scale for gyroscope variation. |
| `AcceStdScale` | Saturation scale for accelerometer variation. |
| `GyroWeight` | Weight of normalized gyroscope variation. |
| `AcceWeight` | Weight of normalized accelerometer variation. |

The current code maps the normalized motion score to an integer between `MinSegments` and `MaxSegments` using rounded linear interpolation. `SO3Spline` and `LinAcceSpline` must be equal because the rotation and acceleration trajectories share one knot vector.

### 2.3 Launch

```bash
source ~/multi_imu_calib_ws/devel/setup.bash

roslaunch mi_calib mi-calib-prog.launch \
    config_path:=/absolute/path/to/config.yaml
```

The calibration parameters are saved as `calibration_non_uniform.<format>` in `OutputPath`. Spline and sampled-kinematics files are also generated when `OutputBSplines` and `OutputKinematics` are enabled.

## 3. Notes

+ Keep all IMUs rigidly fixed during data collection.
+ Excite rotation around all three axes and include changing angular rates. Increasing spline density cannot recover unobservable parameters.
+ Verify the timestamp unit, timestamp definition, topic ordering, and raw-data convention of every IMU.
+ Set `TimeOffsetPadding` large enough to contain the expected time offsets. An unnecessarily large range increases computation.
+ Pre-calibrated IMU intrinsic parameters are recommended.
+ The example configuration is a usage template and is not a bit-for-bit reproduction configuration for every manuscript experiment.

## 4. Citation and Acknowledgements

If this project contributes to your research, please cite the accompanying manuscript after its public citation information is released:

> **“Inertial-Only Spatiotemporal Calibration of Multiple IMUs Using Motion-Adaptive Non-Uniform B-Splines.”**

This work is developed from the continuous-time multi-IMU calibration framework introduced by:

+ **S. Chen**, X. Li, S. Li, Y. Zhou, and S. Wang, “MI-Calib: An Open-Source Spatiotemporal Calibrator for Multiple IMUs Based on Continuous-Time Batch Optimization,” *IEEE Robotics and Automation Letters*, 2024. [[paper](https://ieeexplore.ieee.org/document/10629054)] [[upstream code](https://github.com/Unsigned-Long/MI-Calib)]

The implementation also uses modified components from [CTraj](https://github.com/Unsigned-Long/CTraj) and [tiny-viewer](https://github.com/Unsigned-Long/tiny-viewer). Please preserve all upstream copyright, citation, and license notices.

## 5. License

The repository-level license is MIT. Individual upstream files retain their embedded BSD-style terms, and submodules retain their own licenses. See [LICENSE](LICENSE) and [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

---

<p align="center"><i>
    Inertial-Only Spatiotemporal Calibration of Multiple IMUs Using Motion-Adaptive Non-Uniform B-Splines<br>
    Maintainer: <a href="https://github.com/CuiLongji1">CuiLongji1</a><br>
    Motion-Adaptive Non-Uniform B-Spline Multi-IMU Spatiotemporal Calibration
</i></p>
