#ifndef __APRILTAG_DETECTOR_H_
#define __APRILTAG_DETECTOR_H_

#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/common/image_u8.h>
#include <apriltag/common/zarray.h>
#include <vector>
#include <string>
#include <cmath>
#include <cstring>

namespace Ten
{

// 单次检测结果
struct AprilTagResult
{
    int    id;                    // 标签ID
    double p[4][2];              // 4个角点像素坐标 (CCW)
    double center[2];            // 中心点像素坐标
    float  decision_margin;      // 解码质量 (越高越好)
    int    hamming;              // 纠错位数

    // 兼容旧接口: 转为 cv::Point2f
    std::vector<cv::Point2f> corners() const
    {
        return {
            cv::Point2f((float)p[0][0], (float)p[0][1]),
            cv::Point2f((float)p[1][0], (float)p[1][1]),
            cv::Point2f((float)p[2][0], (float)p[2][1]),
            cv::Point2f((float)p[3][0], (float)p[3][1])
        };
    }

    cv::Point2f centerPt() const
    {
        return cv::Point2f((float)center[0], (float)center[1]);
    }

    float area() const
    {
        auto c = corners();
        return std::abs((float)cv::contourArea(c));
    }

    float sideLengthPx() const
    {
        auto c = corners();
        float d1 = cv::norm(c[0] - c[2]);
        float d2 = cv::norm(c[1] - c[3]);
        return (d1 + d2) / 2.f;
    }
};

// 检测器
class AprilTagDetector
{
public:

    explicit AprilTagDetector(int nthreads = 1)
    {
        // 创建 tagStandard41h12 家族
        tf_ = tagStandard41h12_create();
        if (!tf_)
        {
            throw std::runtime_error("Failed to create tagStandard41h12 family");
        }

        // 创建检测器
        td_ = apriltag_detector_create();
        if (!td_)
        {
            tagStandard41h12_destroy(tf_);
            throw std::runtime_error("Failed to create apriltag detector");
        }

        // 添加家族到检测器
        apriltag_detector_add_family(td_, tf_);

        // ---------- 调优参数 (D435 1~3m / 20cm) ----------
        td_->nthreads         = nthreads;  // 线程数: 默认1安全, 单线程场景可用2~4加速
        td_->quad_decimate    = 1.0f;   // 不降采样, 保证远距离精度
        td_->quad_sigma       = 0.0f;   // 不高斯模糊 (D435 图像质量好)
        td_->refine_edges     = 1;      // 边缘精炼
        td_->decode_sharpening = 0.25;  // 解码锐化

        // 四边形检测阈值
        td_->qtp.min_cluster_pixels  = 5;
        td_->qtp.max_nmaxima         = 10;
        td_->qtp.critical_rad        = (float)(10.0 * M_PI / 180.0);
        td_->qtp.max_line_fit_mse    = 10.0f;
        td_->qtp.min_white_black_diff = 5;
        td_->qtp.deglitch            = 0;

        // 预分配灰度图 (避免每帧分配)
        im_gray_ = image_u8_create(1920, 1080);
    }

    ~AprilTagDetector()
    {
        if (im_gray_)   image_u8_destroy(im_gray_);
        if (td_)        apriltag_detector_destroy(td_);
        if (tf_)        tagStandard41h12_destroy(tf_);
    }

    // 禁止拷贝
    AprilTagDetector(const AprilTagDetector&) = delete;
    AprilTagDetector& operator=(const AprilTagDetector&) = delete;

    // 检测标签
    std::vector<AprilTagResult> detect(const cv::Mat& image)
    {
        std::vector<AprilTagResult> results;

        if (image.empty()) return results;

        // 转为灰度 cv::Mat
        cv::Mat gray;
        if (image.channels() == 3)
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        else
            gray = image;

        // 如果分辨率变了，重建 image_u8
        if (im_gray_->width  != gray.cols ||
            im_gray_->height != gray.rows)
        {
            image_u8_destroy(im_gray_);
            im_gray_ = image_u8_create(gray.cols, gray.rows);
        }

        // 拷贝灰度数据到 apriltag 图像结构
        // cv::Mat 是连续存储的 row-major uint8
        if (gray.isContinuous())
        {
            std::memcpy(im_gray_->buf, gray.data,
                        gray.cols * gray.rows);
        }
        else
        {
            for (int r = 0; r < gray.rows; ++r)
            {
                std::memcpy(im_gray_->buf + r * im_gray_->stride,
                            gray.ptr<uint8_t>(r),
                            gray.cols);
            }
        }

        // 调用 apriltag 检测
        zarray_t* detections = apriltag_detector_detect(td_, im_gray_);
        if (!detections) return results;

        int n = zarray_size(detections);
        results.reserve(n);

        for (int i = 0; i < n; ++i)
        {
            apriltag_detection_t* det = nullptr;
            zarray_get(detections, i, &det);
            if (!det) continue;

            AprilTagResult r;
            r.id              = det->id;
            r.decision_margin = det->decision_margin;
            r.hamming         = det->hamming;
            r.center[0]       = det->c[0];
            r.center[1]       = det->c[1];
            for (int j = 0; j < 4; ++j)
            {
                r.p[j][0] = det->p[j][0];
                r.p[j][1] = det->p[j][1];
            }
            results.push_back(r);
        }

        apriltag_detections_destroy(detections);
        return results;
    }

    void drawResults(
        cv::Mat&                           image,
        const std::vector<AprilTagResult>& results,
        cv::Scalar                         color     = cv::Scalar(0, 255, 0),
        int                                thickness = 2) const
    {
        for (const auto& tag : results)
        {
            // 绘制四边形轮廓
            std::vector<cv::Point> poly(4);
            for (int j = 0; j < 4; ++j)
                poly[j] = cv::Point((int)tag.p[j][0], (int)tag.p[j][1]);
            cv::polylines(image, {poly}, true, color, thickness);

            // 绘制 ID
            std::string label = "ID:" + std::to_string(tag.id);
            cv::Point text_org((int)tag.center[0] - 20,
                               (int)tag.center[1] - 10);
            cv::putText(image, label, text_org,
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);

            // 中心点
            cv::circle(image,
                       cv::Point((int)tag.center[0], (int)tag.center[1]),
                       3, cv::Scalar(0, 0, 255), -1);

            // 决策裕度 (可选)
            cv::putText(image,
                        cv::format("M:%.1f", tag.decision_margin),
                        text_org + cv::Point(0, 18),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45,
                        cv::Scalar(255, 255, 0), 1);
        }
    }

private:
    apriltag_family_t*   tf_      = nullptr;
    apriltag_detector_t* td_      = nullptr;
    image_u8_t*          im_gray_ = nullptr;
    
};

}  // namespace Ten

#endif  // __APRILTAG_DETECTOR_H_
