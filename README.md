# RC26 Vision Camera — Camera Workspace

> 相机视觉感知工作区，基于 ROS Noetic + RealSense，用于点云平面拟合、PnP 位姿解算、KFS 方块定位与 Apriltag 识别。

## 项目概述

本项目是 RC26 机器人竞赛的**相机视觉感知**工作区，经过多个版本迭代，涵盖 RealSense 相机驱动、点云平面拟合 (Plane_FitLocator)、PnP 位姿解算 (KFS)、多线程流水线、KFS 方块识别与 Apriltag 标签检测。

### 核心功能包

| 包名 | 功能 |
|------|------|
| `camera` | 相机驱动、点云平面拟合、PnP 位姿解算、OpenVINO 推理、YOLO 模型、KFS 定位、Apriltag 检测 |

## 使用方法

### 环境要求

- Ubuntu 20.04 + ROS Noetic
- Intel RealSense 相机 + `librealsense2` / `realsense2_camera`
- OpenVINO Runtime（camera_ws4/ws2/ws2.3 需要）
- apriltag 库（camera_ws2.51 需要）

### 编译

```bash
cd /path/to/camera_ws
catkin_make
source devel/setup.bash
```

### 运行节点

| 节点 | 适用版本 | 功能 |
|------|---------|------|
| `rosrun camera test_node` | 所有版本 | 主节点，功能因版本而异 |
| `rosrun camera test_pnp_node` | ws3, ws4 | PnP 位姿解算 |
| `rosrun camera test_thread_node` | ws2.3, ws2.4 | 多线程处理 / 线程池学习 |
| `rosrun camera kfs_locator_test_node` | ws2.6 | KFS 方块定位（实时 / bag 回放 / 批量偏差） |

## 版本迭代日志

### camera_ws3 — PnP 初始化版
- **RealSense** 相机驱动，深度图与彩色图对齐
- **`test_node`**: 点云平面拟合 (`Plane_FitLocator`) — ROI 滤波 → 平面提取 → 方形拟合
- **`test_pnp_node`**: PnP 位姿解算 (`PnP/`) — 基于 KFS 方块的 6D 位姿估计
- 含完整 `kfsPnpRosNode` 封装

### camera_ws4 — 数据集录制 + OpenVINO 集成版
- **`test_node` 完全重写**:
  - 从 `rosbag` 读取图像/深度数据
  - 使用 `PnP/pnp_main.h` 进行 PnP 处理
  - **新增数据集录制**: 保存图像 + 标签到磁盘
- **新增** `openvino.cpp/h`: OpenVINO 推理后端（供 `PnP/pnp_func.h` 和 `yolo/yolo_base.h` 使用）
- **新增** `yolo/` 目录: 多种 YOLO 模型推理头文件
- **优化** PnP 位姿解算与调试输出

### camera_ws — Plane_FitLocator 回归版
- **`test_node`** 回到 `Plane_FitLocator` 点云平面拟合（同 ws3 的 test1）
- **移除** `rosbag` 读取、数据集录制
- **移除** `PnP/` 模块、OpenVINO 链接
- YOLO 模型文件 (`yolo_*.h`) 保留但**未在管线中启用**

### camera_ws2 — 平面拟合 + rosbag 回放版
- **`test_node`** 多模式测试:
  - `test1_frombag()`: rosbag 回放 → 平面拟合
  - `test1_fromframe()`: 相机实时帧 → 平面拟合
  - `test2()`: 定点误差对比
- **新增** `plane_fit.h`: 独立的点云平面拟合封装
- **恢复** OpenVINO 推理（YOLO 仍编译未启用）

### camera_ws2.22 — KFS 方块定位基础版
- **`test_node`** → 使用 `kfs_locator/set_result.h` 进行 KFS 方块定位
- **新增** `kfs_locator/`: KFS 方块定位子模块（set_detect / set_plane / set_result）
- **新增** `config/params.yaml`: 参数配置文件
- 支持 `frombag` / `fromframe` / `test2` 三种模式
- **移除** `Plane_FitLocator/`, `openvino.cpp/h`, `image/`

### camera_ws2.3 — 多线程流水线版
- **`test_node`**: 新增 `test3_batch_bias()` 批量偏差分析；bag 帧间隔控制
- **`test_thread_node`**: 线程池学习节点 — 基础线程池、并发测试、背压机制、运行标志、CPU 绑定
- **新增** `threadpool.cpp/h`: 通用线程池
- **新增** `learn/`: 多线程/线程池学习笔记
- **恢复** OpenVINO 链接（供线程节点可选使用）

### camera_ws2.4 — KFS 方块位姿识别版
- **`test_node`**: `test1_frombag()` (KFS PnP) + `test2_frombag()` (批量偏差)
- **`test_thread_node`**: 正式多线程处理函数 `test()` + `test_thread()`
- **新增** `set_filter.h`: 点云滤波（无效点过滤）
- **新增** `set_result.cpp`: KFS 识别结果输出
- **完善** 35cm 方块 (kfs) 的完整 6D 位姿识别管线

### camera_ws2.51 — Apriltag R1 识别版
- **`test_node`** 完整 Apriltag 视觉管线:
  - `apriltag_detect/`: 检测 R1 机器人上固定的 Apriltag 标签
  - 实时 OpenCV GUI（角点 / 中心 / ID 绘制）
  - 键盘控制录制 (`r`/`s`/`q`)
  - YOLOv5 格式 JSON 标注输出
  - 自动目录管理 (`images/` + `labels/`)
- **新增** `apriltag_detector.h`, `README_APRILTAG.md`
- **新增** CMakeLists.txt `find_package(apriltag REQUIRED)`
- **移除** `kfs_locator/`, `test_thread.cpp`, `threadpool.cpp/h`, `learn/`
- 全管线重构，专注 Apriltag 识别 R1 机器人

### camera_ws2.6 — KFS 方块定位回归版
- **`kfs_locator_test_node`**: 从 camera_ws2.4 回归 KFS 方块定位模块
  - `kfs_locator/`: 完整子模块（set_detect / set_plane / set_result / set_filter / debug_pcl）
  - `test1_fromframe()`: 相机实时帧 → 平面拟合 → 位姿输出
  - `test1_frombag()`: rosbag 回放 → KFS 定位 → 偏差统计
  - `test2_frombag()`: 多 bag 批量偏差分析 → 汇总统计文件
- **新增** `launch/kfs_locator_test.launch`: 对应的 roslaunch 启动文件
- Apriltag 测试节点 `test_node` 保持不变，两者共存

## 稳定版本

| 版本 | 标签 | Release |
|------|------|---------|
| camera_ws3 | `camera/v3.0` | [v3.0](https://github.com/Han0301/RC26_Vision_camera/releases/tag/camera/v3.0) |
| camera_ws4 | `camera/v4.0` | [v4.0](https://github.com/Han0301/RC26_Vision_camera/releases/tag/camera/v4.0) |
| camera_ws | `camera/v1.0` | [v1.0](https://github.com/Han0301/RC26_Vision_camera/releases/tag/camera/v1.0) |
| camera_ws2 | `camera/v2.0` | [v2.0](https://github.com/Han0301/RC26_Vision_camera/releases/tag/camera/v2.0) |
| camera_ws2.22 | `camera/v2.22` | [v2.22](https://github.com/Han0301/RC26_Vision_camera/releases/tag/camera/v2.22) |
| camera_ws2.3 | `camera/v2.3` | [v2.3](https://github.com/Han0301/RC26_Vision_camera/releases/tag/camera/v2.3) |
| camera_ws2.4 | `camera/v2.4` | [v2.4](https://github.com/Han0301/RC26_Vision_camera/releases/tag/camera/v2.4) |
| camera_ws2.51 | `camera/v2.51` | [v2.51](https://github.com/Han0301/RC26_Vision_camera/releases/tag/camera/v2.51) |
| camera_ws2.6 | `camera/v2.6` | [v2.6](https://github.com/Han0301/RC26_Vision_camera/releases/tag/camera/v2.6) |
