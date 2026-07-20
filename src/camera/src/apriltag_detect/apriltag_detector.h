#ifndef __APRILTAG_DETECTOR_H_
#define __APRILTAG_DETECTOR_H_
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/common/image_u8.h>
#include <apriltag/common/zarray.h>
#include <vector>
#include <string>
#include "./../yolo/yolo_v5.h"

namespace Ten::apriltag_detect
{
// 单个检测结果 (用于生成 YOLOv5 格式 JSON)
struct TagDetection
{
    int    id;              // 标签ID
    float  bbox[4];         // 外接矩形: [x1, y1, x2, y2] 像素坐标
    float  corners[4][2];   // 4个角点像素坐标
    float  center[2];       // 中心点
    float  confidence;      // 置信度 (基于 decision_margin)
};

class AprilTagDetector
{
public:
    // 构造和析构
    explicit AprilTagDetector(int nthreads = 1);
    ~AprilTagDetector();

    // 禁止拷贝构造和赋值
    AprilTagDetector(const AprilTagDetector&) = delete;
    AprilTagDetector& operator=(const AprilTagDetector&) = delete;

    // 核心检测器 (返回 true 表示检测到至少一个标签)
    bool detect(cv::Mat& image);

    // 取上次检测到的标签结果
    const std::vector<TagDetection>& detections() const { return detections_; }

    // 取状态
    const std::string& state() const { return state_; }
    apriltag_detector_t* detector() { return td_; }

private:
    std::string           state_;
    apriltag_family_t*    tf_      = nullptr;
    apriltag_detector_t*  td_      = nullptr;
    image_u8_t*           im_gray_ = nullptr;
    std::vector<TagDetection> detections_;  // 上次检测结果
};

// 构造实现
inline AprilTagDetector::AprilTagDetector(int nthreads)
    : state_("init")
{
    tf_ = tagStandard41h12_create();
    if (!tf_)
    {
        state_ = "init_fail: tagStandard41h12_create";
        return;
    }

    td_ = apriltag_detector_create();
    if (!td_)
    {
        tagStandard41h12_destroy(tf_);
        tf_ = nullptr;
        state_ = "init_fail: apriltag_detector_create";
        return;
    }

    apriltag_detector_add_family(td_, tf_);

    td_->nthreads          = nthreads;
    td_->quad_decimate     = 1.0f;
    td_->quad_sigma        = 0.8f;
    td_->refine_edges      = 1;
    td_->decode_sharpening = 0.5f;

    td_->qtp.min_cluster_pixels   = 5;
    td_->qtp.max_nmaxima          = 10;
    td_->qtp.critical_rad         = (float)(20.0 * M_PI / 180.0);
    td_->qtp.max_line_fit_mse     = 10.0f;
    td_->qtp.min_white_black_diff = 5;
    td_->qtp.deglitch             = 0;

    im_gray_ = nullptr;  // 首次 detect() 时惰性分配

    state_ = "AprilTagDetector ok";
}

// 析构实现
inline AprilTagDetector::~AprilTagDetector()
{
    if (im_gray_) { image_u8_destroy(im_gray_); }
    if (td_)      { apriltag_detector_destroy(td_); }
    if (tf_)      { tagStandard41h12_destroy(tf_); }
}

// 检测实现
inline bool AprilTagDetector::detect(cv::Mat& image)
{
    // 1. 状态与输入检查
    detections_.clear();
    if (state_ != "AprilTagDetector ok") { return false; }

    if (image.empty())
    {
        state_ = "empty_frame";
        return false;
    }

    // 2. BGR → Gray
    cv::Mat gray;
    if (image.channels() == 3)
    {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }
    else
    {
        gray = image;
    }

    // 3. 自适应分辨率 (首次分配或分辨率变化)
    if (!im_gray_ || im_gray_->width != gray.cols || im_gray_->height != gray.rows)
    {
        if (im_gray_) { image_u8_destroy(im_gray_); }
        im_gray_ = image_u8_create(gray.cols, gray.rows);
        if (!im_gray_)
        {
            state_ = "gray_realloc_fail";
            return false;
        }
    }

    // 4. 拷贝灰度数据 (逐行, 适配 image_u8 stride 对齐)
    for (int r = 0; r < gray.rows; ++r)
    {
        std::memcpy(im_gray_->buf + r * im_gray_->stride, gray.ptr<uint8_t>(r), gray.cols);
    }

    // 5. 调用 apriltag 检测
    zarray_t* dets = apriltag_detector_detect(td_, im_gray_);
    if (!dets)
    {
        state_ = "detect_returned_null";
        detections_.clear();
        return false;
    }

    // 6. 提取检测结果 (角点、外接矩形等)
    int n = zarray_size(dets);
    detections_.clear();
    detections_.reserve(n);

    for (int i = 0; i < n; ++i)
    {
        apriltag_detection_t* det = nullptr;
        zarray_get(dets, i, &det);
        if (!det) continue;

        TagDetection td;
        td.id = det->id;
        td.center[0] = (float)det->c[0];
        td.center[1] = (float)det->c[1];

        // 角点
        float xmin = 1e9f, ymin = 1e9f, xmax = -1e9f, ymax = -1e9f;
        for (int j = 0; j < 4; ++j)
        {
            td.corners[j][0] = (float)det->p[j][0];
            td.corners[j][1] = (float)det->p[j][1];
            if (det->p[j][0] < xmin) xmin = (float)det->p[j][0];
            if (det->p[j][1] < ymin) ymin = (float)det->p[j][1];
            if (det->p[j][0] > xmax) xmax = (float)det->p[j][0];
            if (det->p[j][1] > ymax) ymax = (float)det->p[j][1];
        }
        td.bbox[0] = xmin;
        td.bbox[1] = ymin;
        td.bbox[2] = xmax;
        td.bbox[3] = ymax;

        // 置信度: 将 decision_margin 映射到 [0,1]
        td.confidence = 1.0f / (1.0f + std::exp(-det->decision_margin * 0.1f));

        detections_.push_back(td);
    }

    apriltag_detections_destroy(dets);
    state_ = "AprilTagDetector ok";

    if (n > 0)
    {
        return true;
    }
    return false;
}
}  // namespace Ten::apriltag_detect
#endif  // __APRILTAG_DETECTOR_H_