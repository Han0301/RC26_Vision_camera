# AprilTag 检测模块说明

> **场景**: Intel RealSense D435 摄像头, 1~3m 范围, 20cm 尺寸图案  
> **字典**: `tagStandard41h12` (41个标签, 汉明距离12)  
> **底层库**: [AprilRobotics/apriltag](https://github.com/AprilRobotics/apriltag) 原生 C 库 (ros-noetic-apriltag 3.2.0)  
> **语言/标准**: C++17, header-only 封装

---

## 目录

- [1. 文件结构](#1-文件结构)
- [2. 整体架构与数据流](#2-整体架构与数据流)
- [3. 核心类详解: AprilTagDetector](#3-核心类详解-apriltagdetector)
- [4. 检测结果: AprilTagResult](#4-检测结果-apriltagresult)
- [5. 管道详解: detect() 内部发生了什么](#5-管道详解-detect-内部发生了什么)
- [6. 关键参数调优指南](#6-关键参数调优指南)
- [7. 主程序执行流程: test.cpp](#7-主程序执行流程-testcpp)
- [8. 编译与依赖](#8-编译与依赖)
- [9. 运行与调试](#9-运行与调试)
- [10. 扩展: 切换标签家族 / 添加位姿解算](#10-扩展)

---

## 1. 文件结构

```
src/camera/
├── apriltag_detector.h   # [新增] AprilTag 检测器, header-only, 核心代码
├── test.cpp              # [改写] ROS 节点主程序, 相机→检测→发布
├── camera.h              # [已有] D435 相机封装 (Ten::Ten_camera)
├── CMakeLists.txt        # [修改] 添加 apriltag::apriltag 依赖
└── ... (其他已有文件不变)
```

**设计原则**:
- `apriltag_detector.h` 是纯 header-only 的 C++ 封装, 零 `.cpp` 文件, 直接 `#include` 即可使用
- 所有类在 `namespace Ten` 下, 与项目已有的 `Ten_camera` 风格一致
- 对外只暴露两个东西: `AprilTagResult` (数据) + `AprilTagDetector` (行为)

---

## 2. 整体架构与数据流

```
┌──────────────┐     cv::Mat(BGR)     ┌──────────────────────┐
│  D435 相机    │ ──────────────────▶ │  AprilTagDetector    │
│ Ten_camera   │                     │                      │
└──────────────┘                     │  detect(image)       │
                                     │    │                 │
                                     │    ├─ BGR→Gray       │
                                     │    ├─ cv::Mat→       │
                                     │    │  image_u8_t     │
                                     │    ├─ apriltag_      │
                                     │    │  detector_detect│
                                     │    └─ zarray遍历     │
                                     │      转为vector       │
                                     └──────────┬───────────┘
                                                │
                                   std::vector<AprilTagResult>
                                                │
                    ┌───────────────────────────┤
                    │                           │
                    ▼                           ▼
           drawResults()                终端 ROS_INFO 输出
           (标注到原图)                 (ID, 中心, 面积, 置信度)
                    │
                    ▼
          /camera/apriltag_result
          (sensor_msgs/Image, BGR8)
```

---

## 3. 核心类详解: AprilTagDetector

### 3.1 类声明

```cpp
namespace Ten {

class AprilTagDetector {
public:
    AprilTagDetector();                                         // 构造: 创建家族+检测器+调参
    ~AprilTagDetector();                                        // 析构: RAII 释放 apriltag 资源
    AprilTagDetector(const AprilTagDetector&) = delete;         // 禁止拷贝
    AprilTagDetector& operator=(const AprilTagDetector&) = delete;

    std::vector<AprilTagResult> detect(const cv::Mat& image);   // 核心: 检测
    void drawResults(cv::Mat& image, ...);                       // 绘制
    apriltag_detector_t* detector();                             // 获取底层指针(高级调参)

private:
    apriltag_family_t*   tf_;       // 标签家族 (tagStandard41h12)
    apriltag_detector_t* td_;       // 检测器实例
    image_u8_t*          im_gray_;  // 预分配灰度图 (避免每帧 malloc)
};
}
```

### 3.2 构造函数详解

构造时做了三件事, 顺序不能乱:

```
1. 创建标签家族 (tagStandard41h12_create)
   → 这是一个包含41个标签的"密码本"
   → 每个标签是一个二进制矩阵, 任意两个标签间汉明距离 ≥ 12
   → 汉明距离越大, 误识别率越低

2. 创建检测器 (apriltag_detector_create)
   → 检测器是算法核心, 包含四边形检测、解码、多线程等

3. 注册家族 (apriltag_detector_add_family)
   → 把"密码本"交给检测器
   → 支持同时注册多个家族
```

**参数调优 (在构造函数中完成)**:

| 参数 | 默认值 | 含义 | 调整建议 |
|------|--------|------|---------|
| `nthreads` | 2 | 并行线程数 | = CPU 核心数 |
| `quad_decimate` | 1.0 | 图像降采样倍数 | 1.0=不降(最准), 2.0=加速但远处可能丢 |
| `quad_sigma` | 0.0 | 高斯模糊 σ | D435 画质好用 0; 噪点多用 0.8 |
| `refine_edges` | 1 | 边缘精炼 | 建议保持 1 |
| `decode_sharpening` | 0.25 | 解码时锐化量 | 0.25 通用; 模糊场景加大到 0.5 |
| `qtp.min_cluster_pixels` | 5 | 最小四边形像素数 | 过滤噪点 |
| `qtp.critical_rad` | 10° | 四边形角度的最大容忍 | 调大=容忍更大畸变, 但误检增加 |
| `qtp.min_white_black_diff` | 5 | 黑白像素最小亮度差 | 光照差时可降低 |

### 3.3 析构函数: RAII

```cpp
~AprilTagDetector() {
    if (im_gray_)   image_u8_destroy(im_gray_);   // 释放灰度图
    if (td_)        apriltag_detector_destroy(td_); // 释放检测器
    if (tf_)        tagStandard41h12_destroy(tf_);  // 释放标签家族
}
```

按照"先创建后销毁"的逆序释放, 不会泄漏。禁止拷贝确保不会出现 double-free。

---

## 4. 检测结果: AprilTagResult

```cpp
struct AprilTagResult {
    int    id;               // 标签 ID (0~40, 对应 tagStandard41h12)
    double p[4][2];          // 4个角点像素坐标, 逆时针排列
    double center[2];        // 中心像素坐标
    float  decision_margin;  // 决策裕度: 解码置信度 (越高越好, >30 基本可靠)
    int    hamming;          // 纠错位数: 被纠正的错误 bit 数 (≤2 可控)
};
```

**便捷方法** (inline, 零开销):

| 方法 | 返回值 | 说明 |
|------|--------|------|
| `corners()` | `vector<cv::Point2f>` | 转为 OpenCV 格式的4个角点 |
| `centerPt()` | `cv::Point2f` | 中心点 |
| `area()` | `float` | 标签在图像中的面积 (px²) |
| `sideLengthPx()` | `float` | 估算标签边长 (px), 对角线平均 |

**`decision_margin` 的含义**:
- 解码时, 每个数据 bit 都有一个"距离决策边界的距离"
- `decision_margin` 是所有 bit 该距离的平均值
- **经验阈值**: `> 30` 可靠, `15~30` 可疑, `< 15` 大概率误检
- 可以在 `detect()` 之后过滤: `if (r.decision_margin < 25) continue;`

**`hamming` 的含义**:
- 检测到的码字与字典中最近码字的汉明距离
- `0` = 完美匹配, `1` = 纠正了1个错误 bit, `2` = 纠正了2个
- apriltag 库默认最多纠正 2 个 bit (`apriltag_detector_add_family` 的 `bits_corrected` 参数)

---

## 5. 管道详解: detect() 内部发生了什么

```cpp
std::vector<AprilTagResult> detect(const cv::Mat& image)
```

### 步骤 1: BGR → Gray

```
cv::Mat (BGR, 3通道)  →  cv::cvtColor  →  cv::Mat (Gray, 1通道)
```

apriltag 库只需要灰度信息, 提前转换避免后续重复操作。

### 步骤 2: 自适应分辨率

```cpp
if (im_gray_->width != gray.cols || im_gray_->height != gray.rows) {
    image_u8_destroy(im_gray_);
    im_gray_ = image_u8_create(gray.cols, gray.rows);
}
```

预分配的 `image_u8_t` 与当前帧分辨率不匹配时自动重建。首次构造时用 1920×1080, 如果相机切到 640×480 也能自适应。

### 步骤 3: cv::Mat → image_u8_t (内存拷贝)

```cpp
std::memcpy(im_gray_->buf, gray.data, gray.cols * gray.rows);
```

为什么需要拷贝？
- `apriltag_detector_detect()` 的输入是 `image_u8_t*`
- `image_u8_t` 的 `buf` 字段是 `uint8_t*`, 与 `cv::Mat` 的 `.data` 一致
- 两个结构体内存布局略有不同 (`image_u8_t` 有 `stride` 字段), 所以不能零拷贝包装

### 步骤 4: 调用 apriltag 检测引擎

```cpp
zarray_t* detections = apriltag_detector_detect(td_, im_gray_);
```

**这行代码内部发生了什么 (apriltag 算法)**:

```
┌─────────────────────────────────────────────┐
│  a. 自适应阈值分割                           │
│     → 将灰度图分成多个局部区块               │
│     → 每个区块用局部阈值二值化               │
│     → 合并所有区块的二值结果                 │
├─────────────────────────────────────────────┤
│  b. 连通域分析                               │
│     → 提取二值图的所有连通域                  │
│     → 聚类为候选四边形 (quad)                │
├─────────────────────────────────────────────┤
│  c. 四边形拟合                                │
│     → 对每个候选区域做直线拟合                │
│     → 找到4条边, 形成四边形                  │
│     → 过滤非四边形 (边数≠4, 角度太直/太平)  │
├─────────────────────────────────────────────┤
│  d. 解码                                      │
│     → 对每个四边形做透视变换到正方形          │
│     → 按 tag family 定义的 bit 位置采样       │
│     → 与字典中所有码字比对 (汉明距离)         │
│     → 取最近匹配, 计算 decision_margin        │
├─────────────────────────────────────────────┤
│  e. 边缘精炼 (如果 refine_edges=1)            │
│     → 在原始图像中沿四边形边缘搜索梯度        │
│     → 微调角点位置到亚像素精度                │
└─────────────────────────────────────────────┘
```

### 步骤 5: zarray → std::vector<AprilTagResult>

```cpp
for (int i = 0; i < zarray_size(detections); ++i) {
    apriltag_detection_t* det = nullptr;
    zarray_get(detections, i, &det);
    // 拷贝字段到 AprilTagResult...
}
apriltag_detections_destroy(detections);  // 释放 C 库返回的数组
```

`zarray` 是 apriltag 库的泛型动态数组 (类似 Java ArrayList)。遍历提取后必须调用 `apriltag_detections_destroy` 释放。

### 步骤 6: 返回

返回 `std::vector<AprilTagResult>`, 空的表示未检测到任何标签。

---

## 6. 关键参数调优指南

### 6.1 检测不到远处的标签

```cpp
detector.detector()->quad_decimate = 1.0f;    // 确保不降采样
detector.detector()->decode_sharpening = 0.5;  // 加大锐化
detector.detector()->qtp.min_cluster_pixels = 3; // 允许更小的四边形
```

### 6.2 误检太多

```cpp
// 在 detect() 返回后过滤
for (auto& r : results) {
    if (r.decision_margin < 30.0f) continue;  // 提高置信度门槛
    if (r.hamming > 1) continue;              // 不允许纠正超过1个bit
    // ... 保留这个结果
}
```

### 6.3 追求速度

```cpp
detector.detector()->quad_decimate = 2.0f;  // 降采样到 1/2 分辨率
detector.detector()->nthreads = 4;          // 使用全部 CPU 核心
```

> ⚠️ `quad_decimate=2.0` 在 3m 距离 20cm 标签场景下可能导致检测率下降 20-30%。

### 6.4 图像噪声大 (低光照)

```cpp
detector.detector()->quad_sigma = 0.8f;              // 加高斯模糊
detector.detector()->qtp.min_white_black_diff = 3;   // 降低黑白差异要求
detector.detector()->qtp.deglitch = 1;               // 去除椒盐噪声
```

---

## 7. 主程序执行流程: test.cpp

```
main()
 ├── ros::init("test_node")
 ├── image_transport 发布器 → /camera/apriltag_result
 ├── Ten::Ten_camera::GetInstance(1920, 1080, 30)  // D435 单例
 ├── Ten::AprilTagDetector detector;               // 检测器 (触发构造)
 │
 └── while(ros::ok()) @ 30Hz:
      ├── cam.camera_read()           → cv::Mat frame (BGR)
      ├── detector.detect(frame)      → vector<AprilTagResult>
      ├── detector.drawResults(frame) → 标注到图像
      ├── ROS_INFO 终端输出
      ├── cv_bridge::CvImage → sensor_msgs::Image
      └── pub_result.publish(msg)
```

**ROS 话题**:

| 话题 | 类型 | 说明 |
|------|------|------|
| `/camera/apriltag_result` | `sensor_msgs/Image` | 带标注的 BGR8 图像, 可直接 `rqt_image_view` |

**终端输出示例**:
```
[apriltag] ID:3 | center:(1024,540) | area:18432 px² | margin:45.2
[apriltag] ID:7 | center:(1560,320) | area:9820 px² | margin:38.1
```

---

## 8. 编译与依赖

### 依赖

```
ros-noetic-apriltag (3.2.0)   → 原生 apriltag C 库
ros-noetic-apriltag-ros       → ROS 封装 (可选, 未直接使用)
OpenCV 4.2+                   → 图像处理
librealsense2                 → D435 驱动
```

### CMake 配置 (关键部分)

```cmake
find_package(apriltag REQUIRED)          # 查找 apriltag

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src      # 让 test.cpp 能找到 apriltag_detector.h
)

target_link_libraries(test_node
    apriltag::apriltag                   # 链接 apriltag 库 (导入目标)
)
```

> 使用 `apriltag::apriltag` 导入目标, 而非手动写 `-lapriltag`, 这样 CMake 会自动处理 include 路径和传递依赖 (pthread, m)。

### 编译命令

```bash
cd ~/RC2026/camera_ws2.5
catkin build camera
source devel/setup.bash
```

---

## 9. 运行与调试

### 基本运行

```bash
rosrun camera test_node
```

### 可视化

```bash
# 终端1: 运行节点
rosrun camera test_node

# 终端2: 查看标注图像
rqt_image_view /camera/apriltag_result

# 终端3: 查看话题信息
rostopic info /camera/apriltag_result
rostopic hz  /camera/apriltag_result
```

### 调试技巧

**1. 开启 apriltag 调试图像 (会输出到当前目录)**:
```cpp
detector.detector()->debug = 1;
```
会在运行目录生成 `debug_*.pnm` 图像, 展示阈值分割、四边形检测的中间结果。

**2. 打印检测统计**:
```cpp
auto* td = detector.detector();
ROS_INFO("edges:%u segments:%u quads:%u", td->nedges, td->nsegments, td->nquads);
```
可以观察每帧有多少候选四边形进入了检测管道。

**3. 降低分辨率提速测试**:
在 `test.cpp` 中把相机初始化改为 `GetInstance(640, 480, 30)`, 然后:
```cpp
detector.detector()->quad_decimate = 1.0f; // 低分辨率下不用降采样
```
D435 的 640×480 模式在 3m 处 20cm 标签约 30px, 仍可检测但精度下降。

---

## 10. 扩展

### 10.1 切换到其他标签家族

只需修改 `apriltag_detector.h` 的 3 处:

```cpp
// 1. 改 include
#include <apriltag/tag36h11.h>        // 原来是 tagStandard41h12.h

// 2. 改构造函数
tf_ = tag36h11_create();              // 原来是 tagStandard41h12_create()

// 3. 改析构函数
tag36h11_destroy(tf_);               // 原来是 tagStandard41h12_destroy()
```

已安装的可用家族:
```
tag16h5      tag25h9      tag36h10      tag36h11
tagCircle21h7   tagCircle49h12
tagCustom48h12  tagStandard41h12   tagStandard52h13
```

### 10.2 添加位姿解算 (PnP)

`apriltag_detection_t` 已经提供了单应矩阵 `det->H` (3×3), 可以直接做位姿估计:

```cpp
// 在 detect() 中, 从 det->H 提取位姿
cv::Mat H(3, 3, CV_64F);
for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++)
        H.at<double>(r,c) = matd_get(det->H, r, c);

// 用相机内参 + solvePnP 解算
cv::solvePnP(objectPoints, tag.corners(), K, distCoeffs, rvec, tvec);
```

### 10.3 发布单独的检测结果话题

可以在 `test.cpp` 中添加一个自定义消息或使用 `vision_msgs` 发布每个检测结果的 ID 和位姿, 供下游节点消费。

---

## 附录: 为什么选择原生 apriltag 库而非 OpenCV ArUco

| | OpenCV ArUco (4.2) | 原生 apriltag |
|---|---|---|
| AprilTag 支持 | ❌ (需 4.6+) | ✅ 原生 |
| 远距离检测率 | 一般 | **高** (优化过的解码算法) |
| 角点精度 | 亚像素 (CORNER_REFINE_SUBPIX) | 亚像素 (边缘梯度精炼) |
| 误检率 | 较低 | **更低** (汉明距离约束更强) |
| decimate 加速 | ❌ | ✅ 内置 |
| 多线程 | ❌ | ✅ 内置 |
| 调试可视化 | ❌ | ✅ debug 模式 |
