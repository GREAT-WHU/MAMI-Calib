<h3 align="center">Inertial-Only Spatiotemporal Calibration of Multiple IMUs Using Motion-Adaptive Non-Uniform B-Splines</h3>

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

This repository presents an inertial-only spatiotemporal calibration method for rigidly connected asynchronous IMUs. It estimates the relative rotation, translation, and time offset between a reference IMU and each auxiliary IMU using raw gyroscope and accelerometer measurements.

The reference motion is represented by motion-adaptive non-uniform B-splines. The knot density is adjusted according to local motion variation, with more knots placed in highly dynamic intervals and fewer knots in smooth intervals.

## Main Features

+ ***Inertial-only:*** uses only raw gyroscope and accelerometer measurements; cameras, LiDAR, and calibration targets are not required.
+ ***Multi-IMU spatiotemporal calibration:*** estimates the relative rotation, translation, and time offset between the reference IMU and each auxiliary IMU.
+ ***Motion-adaptive non-uniform B-splines:*** sets the number of spline knots in each time window from the local variation of gyroscope and accelerometer measurements.
+ ***Staged initialization and joint optimization:*** initializes the trajectory, gravity, extrinsics, and time offsets in stages, then jointly refines them through continuous-time batch optimization.

## Publication

For more details, please refer to the accompanying manuscript:

> **“Inertial-Only Spatiotemporal Calibration of Multiple IMUs Using Motion-Adaptive Non-Uniform B-Splines.”**

The public paper link and BibTeX entry will be added when the publication metadata becomes available.

> **Code release:** The source code will be released after the paper is accepted.
