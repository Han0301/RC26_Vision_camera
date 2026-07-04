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

### camera_ws2 — YOLO + OpenVINO 融合版
- **恢复** `openvino.cpp/h`: OpenVINO 推理重新集成
- **新增** `plane_fit.h`: 点云平面拟合模块
- 融合 YOLO 检测与 OpenVINO 推理优化
- 优化 Plane_FitLocator 点云处理管线

### camera_ws2.22 — KFS 定位基础版
- **新增** `kfs_locator/`: KFS 方块定位模块
- **新增** `config/`: 配置文件目录
- **移除** `openvino.cpp/h`: 移除 OpenVINO 推理
- **移除** `Plane_FitLocator/`: 移除平面拟合定位
- **移除** `image/`: 移除图像资源
- 精简依赖，专注 KFS 定位功能

### camera_ws2.3 — 多线程处理版
- **新增** `test_thread.cpp`: 多线程处理节点 (`test_thread_node`)
- **新增** `threadpool.cpp`: 线程池实现
- **新增** `learn/`: 学习测试目录
- **新增** CMakeLists.txt 中 test_thread_node 编译目标
- **重构** `kfs_locator`: 移除 params_loader，优化 set_plane/set_result
- **移除** `config/` 配置文件目录
- 引入多线程并行处理架构

### camera_ws2.4 — KFS 方块位姿识别版
- **新增** `set_filter.h`: 点云滤波模块，过滤无效点
- **新增** `set_result.cpp`: 识别结果处理与输出
- **新增** YOLO 设计文档 (`yolo_polymorphic_design.md`)
- **新增** `learn/` 移至 camera/src 目录
- **优化** `kfs_locator` 各模块: 识别 35cm 方块 (kfs) 完整位姿
- **优化** 多线程处理管线 (test_thread.cpp / threadpool.h)

### camera_ws2.51 — Apriltag 识别版
- 引入 apriltag 库（`find_package(apriltag REQUIRED)`）
- 在 R1 机器人上固定 apriltag 标识，通过识别 apriltag 定位 R1
- 移除多线程节点和 OpenVINO 依赖
- 简化管线，专注 apriltag 定位