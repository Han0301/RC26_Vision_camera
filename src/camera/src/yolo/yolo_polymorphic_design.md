# YOLO 多态推理模块设计文档

## 目录

- [1. 概述](#1-概述)
- [2. 类继承体系](#2-类继承体系)
- [3. 核心设计模式](#3-核心设计模式)
- [4. 各层详细设计](#4-各层详细设计)
- [5. 设计优点](#5-设计优点)
- [6. 扩展新模型的步骤](#6-扩展新模型的步骤)
- [7. 改进建议](#7-改进建议)

---

## 1. 概述

本模块面向 RoboMaster 机器人比赛中的多型号装甲板检测任务，需要在同一个代码框架下支持：

- **YOLOv5**（检测）
- **YOLOv11**（检测）
- **YOLOv26-OBB**（旋转框检测）
- **YOLOv11-cls**（分类）
- **YOLO-Han / Han2**（多图批次分类）

不同 YOLO 版本在**输出张量的形状、布局、解析方式**上差异巨大，但**模型加载、预处理、推理执行**的流程高度一致。本设计通过**两层继承 + 模板方法模式**，在复用公共逻辑的同时隔离差异。

---

## 2. 类继承体系

```
                    ┌─────────────────┐
                    │      yolo       │  ← 抽象基类（纯虚接口）
                    │  (yolo_base.h)  │
                    └────────┬────────┘
                             │ public
                    ┌────────▼────────┐
                    │    yolo_vx      │  ← 中间层（实现公共逻辑）
                    │  (yolo_vx.h)    │     封装 OpenVINO 加载/预处理/推理
                    └──┬──┬──┬──┬────┘
        ┌─────────────┘  │  │  └──────────────┐
       │           │          │                 │
┌──────▼─────┐ ┌───▼────┐ ┌──▼──────┐ ┌───────▼──────┐
│  yolo_v5   │ │yolo_v11│ │yolo_26obb│ │yolo_v11_cls  │
│ (检测框)   │ │(检测框)│ │(旋转框)  │ │  (分类)      │
└────────────┘ └────────┘ └──────────┘ └──────────────┘

┌───────────┐  ┌───────────┐
│ yolo_han  │  │ yolo_han2 │  ← 独立类（输入接口不同，
│ (3分类)   │  │ (2分类)   │      无法共享 yolo 接口）
└───────────┘  └───────────┘
```

---

## 3. 核心设计模式

### 3.1 模板方法模式（Template Method）

这是整个架构的灵魂。基类 `yolo_vx` 中的 `worker()` 定义了推理的**骨架流程**，子类只负责填充变化的部分：

```cpp
// yolo_vx::worker() — 骨架（在基类中固定）
virtual std::vector<Detection> worker(cv::Mat img) override
{
    cv::Mat resized;
    cv::resize(img, resized, cv::Size(input_shape_[3], input_shape_[2]));

    ov::Tensor input_tensor = preprocess(resized);  // ① 预处理（基类实现）

    infer_request_.set_input_tensor(input_tensor);
    infer_request_.start_async();
    infer_request_.wait();                           // ② 推理（基类实现）

    auto output = infer_request_.get_output_tensor();
    auto detections = postprocess(output, img.cols, img.rows); // ③ 后处理（子类实现）
    return detections;
}
```

**三步走流程**：

| 步骤 | 方法 | 实现位置 | 原因 |
|------|------|----------|------|
| ① 预处理 | `preprocess()` | `yolo_vx` | RGB 转换 + 归一化 + HWC→CHW，所有 YOLO 检测模型通用 |
| ② 推理 | 内联在 `worker()` | `yolo_vx` | OpenVINO 异步推理，逻辑完全相同 |
| ③ 后处理 | `postprocess()` | 各子类 | 输出张量布局不同，是唯一的差异点 |

### 3.2 策略模式（Strategy）

每个子类就是一个**后处理策略**。客户端通过多态指针调用统一的 `worker()` 接口，运行时根据实际类型动态分发到不同的解析逻辑：

```cpp
// 客户端代码 — 完全不用关心是哪个 YOLO 版本
std::unique_ptr<Ten::yolo::yolo> detector;

if (config.model_type == "v5")
    detector = std::make_unique<Ten::yolo::yolo_v5>(path, "CPU", 0.75, 0.75, 0.5);
else if (config.model_type == "v11")
    detector = std::make_unique<Ten::yolo::yolo_v11>(path, "CPU", 0.75, 0.5);

// 统一调用
auto detections = detector->worker(frame);
```

### 3.3 两层继承的职责分离

| 层级 | 类 | 职责 |
|------|-----|------|
| L0 接口层 | `yolo` | 定义纯虚接口 `worker()` / `preprocess()` / `postprocess()` |
| L1 复用层 | `yolo_vx` | 持有 OpenVINO 资源（Core、Model、Request），实现 `worker()` 和 `preprocess()`，将 `postprocess()` 留白 |
| L2 差异层 | `yolo_v5` / `yolo_v11` / `yolo_26obb` / `yolo_v11_cls` | 只实现 `postprocess()`（和可选的 `Get_result()`） |

---

## 4. 各层详细设计

### 4.1 抽象基类 `yolo`（`yolo_base.h`）

```cpp
class yolo
{
public:
    virtual std::vector<Detection> worker(cv::Mat img) = 0;     // 推理入口
protected:
    virtual ov::Tensor preprocess(cv::Mat& image) = 0;          // 预处理
    virtual std::vector<Detection> postprocess(ov::Tensor& output,
                                               int orig_w, int orig_h) = 0; // 后处理
};
```

- 三个纯虚函数构成了**最小接口契约**：任何检测模型都必须能"预处理→推理→后处理"
- `Detection` 结构体统一了输出格式：中心坐标、宽高、置信度、类别、角度（旋转框复用）
- `model` 结构体 + `compareConfidence` 作为辅助工具放在同一头文件

### 4.2 中间层 `yolo_vx`（`yolo_vx.h`）

这一层是复用的核心，封装了所有与 OpenVINO 打交道的逻辑：

```cpp
class yolo_vx : public yolo
{
protected:
    ov::Core core_;                    // OpenVINO 核心引擎（全局唯一即可）
    ov::CompiledModel compiled_model_; // 硬件编译后的模型
    ov::InferRequest infer_request_;   // 推理请求
    ov::Shape input_shape_;            // 输入形状 [1, 3, 640, 640]
    ov::Shape output_shape_;           // 输出形状 [1, N, M]
    int flag_ = 1;                     // 模型校验标志
};
```

**关键设计决策**：

1. **资源作为成员变量而非局部变量**：避免每次推理都重新加载模型，一次构造、反复推理，极大提升实时性
2. **构造函数完成全部初始化**：`read_model` → `compile_model` → `create_infer_request`，失败则设置 `flag_ = 0`，后续 `worker()` 直接短路返回
3. **`Get_result()` 虚函数**：为 YOLOv5 等需要复杂 NMS 的模型提供钩子，子类可选择覆盖

### 4.3 检测子类（L2 层）

#### YOLOv5 vs YOLOv11 的核心差异

两者的输出张量形状看似类似（都是 `[1, 特征数, 检测框数]`），但**内存布局完全不同**：

```
YOLOv5:  [1, 25200, 8]   →  data[i * 8 + k]
                             按行优先：框0的8个特征连续存放
                             访问方式：data[框索引 * 特征数 + 通道]

YOLOv11: [1, 8, 8400]    →  data[k * 8400 + i]
                             按通道优先：所有框的cx连续存放，然后所有框的cy...（注意这里通道维度在8400前）
                             
                             PS: 输出形状实际为 [1, 144, 8400]（144=4坐标+C类），
                             前4通道为 cx/cy/w/h，行为类似通道优先布局
```

如果不用多态隔离，调用方需要写两套完全不同的解析代码。通过多态，差异被封装在各自的 `postprocess()` / `Get_result()` 中。

#### YOLOv26-OBB（旋转框）

继承 `yolo_vx`，输出增加角度维度 `[1, 300, 7]`（7 = cx + cy + w + h + conf + cls + angle），NMS 使用旋转框专用的 `nmsFilterOBB()`。

#### YOLOv11-cls（分类）

继承 `yolo_vx`，输出为 `[1, class_num]` 的一维概率向量。`postprocess()` 中取 top-k 索引，映射回原类别 ID。

### 4.4 独立类 `yolo_han` / `yolo_han2`

这两个类**没有继承** `yolo`，原因是它们的输入接口与检测模型不兼容：

| 对比项 | yolo 体系 | yolo_han 体系 |
|--------|----------|---------------|
| 输入类型 | `cv::Mat`（单张图） | `std::vector<cv::Mat>`（12 张图批次） |
| 输出类型 | `std::vector<Detection>` | `std::vector<han>` / `std::vector<han2>` |
| 预处理 | RGB 归一化 | 特殊的均值/标准差归一化 + 批次组装 |

强行继承 `yolo` 会导致接口语义扭曲，因此选择独立实现。这是**接口隔离原则（ISP）**的体现——不应强迫类依赖它不需要的接口。

---

## 5. 设计优点

### 5.1 开闭原则（OCP）

新增一个 YOLO 变体，只需：
1. 新建一个 `.h` 文件
2. 继承 `yolo_vx`
3. 重写 `postprocess()`（约 30~60 行）

**不需要修改任何已有代码**。`yolo_vx`、`yolo`、其他子类、客户端调用逻辑全部保持不变。

### 5.2 单一职责原则（SRP）

```
yolo          → 定义"什么是检测模型"
yolo_vx       → 封装 OpenVINO 交互
yolo_v5/v11   → 各自只负责输出解析
```

每层只有一个变化的理由。

### 5.3 代码复用

如果 6 种模型都各自实现 OpenVINO 加载/推理，代码量约为：

$$ 6 \times (\text{模型加载} + \text{预处理} + \text{推理执行} + \text{后处理}) \approx 6 \times 120 \text{行} = 720 \text{行} $$

实际设计：

$$ \underbrace{1 \times (\text{公共逻辑})}_{yolo\_vx: \sim 80行} + \sum_{i=1}^{4} \underbrace{(\text{各自后处理})}_{\sim 40行} + \underbrace{2 \times (\text{han独立逻辑})}_{\sim 100行} \approx 440 \text{行} $$

节省约 **40%** 代码量，且消除重复带来的维护风险。

### 5.4 运行时多态的优势

```cpp
// 配置驱动：根据配置文件/参数动态选择模型
std::map<std::string, std::function<std::unique_ptr<yolo>()>> factory = {
    {"v5",    []{ return std::make_unique<yolo_v5>(...); }},
    {"v11",   []{ return std::make_unique<yolo_v11>(...); }},
    {"v11cls", []{ return std::make_unique<yolo_v11_cls>(...); }},
};

auto detector = factory[config.model_name]();
// 之后的所有代码完全不用关心具体类型
```

### 5.5 单元测试友好

可以为每个子类独立编写测试，也可以 mock 基类来测试客户端逻辑：

```cpp
// 测试 yolo_v5 的后处理逻辑
yolo_v5 detector("model", "CPU", 0.5, 0.5, 0.5);
auto fake_output = create_mock_tensor(/* 模拟 YOLOv5 输出 */);
auto result = detector.postprocess(fake_output, 640, 480);
assert(result.size() > 0);
```

---

## 6. 扩展新模型的步骤

假设需要支持 YOLOv8：

**步骤 1**：创建 `yolo_v8.h`

```cpp
#include "yolo_vx.h"

class yolo_v8 : public yolo_vx
{
public:
    yolo_v8(const std::string& model_path, const std::string& xpu,
            float conf_thres = 0.75, float iou = 0.5)
        : yolo_vx(model_path, xpu), conf_thres_(conf_thres), iou_(iou)
    {
        // 校验输出形状（YOLOv8 输出可能是 [1, 84, 8400]）
        if (output_shape_.size() != 3 || output_shape_[1] != 84)
            flag_ = 0;
    }

protected:
    std::vector<Detection> postprocess(ov::Tensor& output,
                                       int orig_w, int orig_h) override
    {
        // 实现 YOLOv8 特有的输出解析逻辑
        // ...
    }

    float conf_thres_;
    float iou_;
};
```

**步骤 2**：在工厂函数中注册

```cpp
factory["v8"] = []{ return std::make_unique<yolo_v8>(...); };
```

**完成**。`yolo_vx` 中的模型加载、预处理、推理执行全部自动复用。

---

## 7. 改进建议

当前设计已经非常合理，以下是一些可选的增强方向：

### 7.1 将 `yolo_han` 也纳入继承体系

可以抽象一个更上层的 `model_base`：

```cpp
template <typename InputType, typename OutputType>
class model_base
{
public:
    virtual OutputType worker(InputType input) = 0;
};
```

然后 `yolo` 是 `model_base<cv::Mat, std::vector<Detection>>`，`yolo_han` 是 `model_base<std::vector<cv::Mat>, std::vector<han>>`。不过考虑到输入输出类型差异太大，当前独立实现也是合理的。

### 7.2 用纯虚接口替代 `flag_` 校验

当前通过 `flag_` 整型标志在运行时判断模型是否有效。更 C++ 的做法是构造失败时抛异常，或使用 `std::optional` 工厂：

```cpp
static std::optional<std::unique_ptr<yolo>> create(const std::string& path,
                                                     const std::string& xpu)
{
    auto ptr = std::make_unique<yolo_v5>(path, xpu);
    if (ptr->flag_ == 0) return std::nullopt;
    return ptr;
}
```

### 7.3 将 `Get_result()` 提升到 `yolo_vx` 的接口文档中

当前 `Get_result()` 是 `yolo_vx` 的 `protected virtual` 方法，但 `yolo_v11` 和 `yolo_26obb` 选择直接覆盖 `postprocess()` 而不走 `Get_result()`。建议在注释中明确约定：子类要么覆盖 `postprocess()`，要么覆盖 `Get_result()`，二者选一。

### 7.4 增加 `noexcept` 和移动语义

实时系统中，避免异常的传播和多余的拷贝：

```cpp
virtual std::vector<Detection> worker(cv::Mat img) noexcept override;
```

---

## 总结

本 YOLO 多态模块的核心设计思想是：

> **用两层继承将"不变"与"可变"分离——OpenVINO 的加载/推理是稳定的公共层，各 YOLO 版本的输出解析是独立的差异层。通过模板方法模式，骨架在基类中固化，细节在子类中填充。**

这一设计的最大价值在于：当 YOLO 从 v5 演进到 v11 再到未来的新版本时，你只需要写 30~60 行后处理代码，其余所有基础设施自动复用。
