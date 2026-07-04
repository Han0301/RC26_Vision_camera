# RC26 Vision Camera — Camera Workspace

> 相机视觉感知工作区，基于 ROS Noetic，用于 Realsense 相机图像采集与 PnP 位姿估计。

## 项目概述

本项目是 RC26 机器人竞赛的**相机视觉感知**工作区，核心功能包括 Realsense 相机驱动、图像采集、PnP 位姿解算。

### 核心功能包

| 包名 | 功能 |
|------|------|
| `camera` | 相机驱动、图像采集、位姿估计、OpenVINO 推理、YOLO 检测 |

## 使用方法

### 环境要求

- Ubuntu 20.04 + ROS Noetic
- Intel Realsense 相机 + `realsense2_msgs`
- OpenVINO Runtime（部分版本需要）
- apriltag 库（2.51 版本需要）

### 编译

```bash
cd /path/to/camera_ws
catkin_make
source devel/setup.bash
```

### 运行

```bash
# 启动相机采集节点
rosrun camera test_node

# PnP 位姿估计节点（camera_ws3/ws4）
rosrun camera test_pnp_node

# 多线程处理节点（camera_ws2.3/ws2.4）
rosrun camera test_thread_node
```

## 版本迭代日志

### camera_ws3 — PnP 初始化版
- 基础 Realsense 相机驱动与图像采集
- PnP 位姿解算初始实现
- 含 `test_node` 和 `test_pnp_node` 两个节点

### camera_ws4 — OpenVINO 集成版
- **新增** `openvino.cpp/h`: OpenVINO 推理优化模块
- **新增** `yolo/` YOLO 目标检测集成
- 优化 PnP 位姿解算精度
- 更新 PnP 调试与参数配置

### camera_ws — YOLO 初始版
- **引入** YOLO 目标检测（`yolo_base.cpp` + 多种模型适配）
- **移除** OpenVINO 推理模块
- **移除** PnP 位姿解算模块（`PnP/` 目录）
- **移除** `test_pnp_node` 节点
- **精简** CMakeLists.txt，去除 OpenVINO 和 Realsense 依赖
- 单一 `test_node` 节点，专注 YOLO 检测

### camera_ws2 — YOLO 改进版
- 优化 YOLO 集成方式
- 改进相机处理流程

### camera_ws2.22 — 基础稳定版
- 精简稳定的相机驱动版本
- 移除不必要的依赖

### camera_ws2.3 — 多线程处理版
- 引入线程池（`threadpool.cpp`）
- 新增 `test_thread_node` 多线程处理节点
- 提升图像处理并发能力

### camera_ws2.4 — 方块位姿识别版
- 识别 kfs（35cm 方块）位姿
- 优化多线程处理管线
- 保持 `test_node` + `test_thread_node` 双节点

### camera_ws2.51 — Apriltag 识别版
- 引入 apriltag 库（`find_package(apriltag REQUIRED)`）
- 在 R1 机器人上固定 apriltag 标识，通过识别 apriltag 定位 R1
- 移除多线程节点和 OpenVINO 依赖
- 简化管线，专注 apriltag 定位