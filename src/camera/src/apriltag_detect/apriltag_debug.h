#ifndef __APRILTAG_DEBUG_H_
#define __APRILTAG_DEBUG_H_

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <iomanip>
#include <sstream>
#include "apriltag_detector.h"

namespace Ten::apriltag_detect
{

class AprilTagDebug
{
public:
    explicit AprilTagDebug(AprilTagDetector& detector)
        : td_(detector.detector())
    {}

    void drawOverlay(cv::Mat& image, int tag_count, double fps,
                     bool show_stats = false)
    {
        int y = 40;
        cv::putText(image,
                    "FPS: " + std::to_string((int)fps),
                    cv::Point(20, y),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0,
                    cv::Scalar(0, 255, 255), 2);

        y += 35;
        cv::putText(image,
                    "Tags: " + std::to_string(tag_count),
                    cv::Point(20, y),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0,
                    cv::Scalar(0, 255, 255), 2);

        if (show_stats && td_)
        {
            y += 30;
            cv::putText(image,
                cv::format("edges:%u seg:%u quads:%u",
                           td_->nedges, td_->nsegments, td_->nquads),
                cv::Point(20, y),
                cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(200, 200, 200), 1);

            y += 22;
            cv::putText(image,
                cv::format("decimate:%.1f sharpen:%.2f nthreads:%d",
                           td_->quad_decimate,
                           td_->decode_sharpening,
                           td_->nthreads),
                cv::Point(20, y),
                cv::FONT_HERSHEY_SIMPLEX, 0.45,
                cv::Scalar(180, 180, 180), 1);
        }
    }

    // 开启后会在运行目录输出 debug_*.pnm 中间图像 (非常慢, 仅调试用)
    void enableNativeDebug(bool on)
    {
        if (td_)
        {
            td_->debug = on ? 1 : 0;
            ROS_INFO("[apriltag] Native debug mode: %s",
                     on ? "ON (slow!)" : "OFF");
        }
    }

    void printStats() const
    {
        if (!td_) return;

        ROS_INFO_THROTTLE(2,
            "[apriltag] Stats: edges=%u segments=%u quads=%u | "
            "decimate=%.1f sharpen=%.2f nthreads=%d",
            td_->nedges, td_->nsegments, td_->nquads,
            td_->quad_decimate, td_->decode_sharpening, td_->nthreads);
    }

private:
    apriltag_detector_t* td_;
};

}  // namespace Ten::apriltag_detect

#endif  // __APRILTAG_DEBUG_H_
