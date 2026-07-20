#ifndef __APRILTAG_DEBUG_H_
#define __APRILTAG_DEBUG_H_

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <iomanip>
#include <sstream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "apriltag_detector.h"

namespace Ten::apriltag_detect
{

class AprilTagDebug
{
public:
    explicit AprilTagDebug(AprilTagDetector& detector,
                           ros::NodeHandle& nh,
                           const std::string& topic = "/camera/apriltag_debug")
        : td_(detector.detector())
        , it_(nh)
        , pub_(it_.advertise(topic, 1))
    {
        ROS_INFO("[AprilTagDebug] Publishing debug image on %s", topic.c_str());
    }

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

    /**
     * @brief 绘制检测结果并发布调试图像
     * @param image       原始 BGR 图像（会被绘制）
     * @param detections  检测结果列表
     * @param fps         当前帧率（可选，显示在左上角）
     */
    void publishDebugImage(cv::Mat& image,
                           const std::vector<TagDetection>& detections,
                           double fps = 0.0)
    {
        // 绘制检测框
        drawDetections(image, detections);

        // 绘制覆盖信息
        drawDebugOverlay(image, (int)detections.size(), fps);

        // 发布为 ROS 图像话题
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(
            std_msgs::Header(), "bgr8", image).toImageMsg();
        msg->header.stamp = ros::Time::now();
        pub_.publish(msg);
    }

private:
    // 在图像上绘制检测框
    static void drawDetections(cv::Mat& img,
                               const std::vector<TagDetection>& detections)
    {
        for (const auto& t : detections)
        {
            // 外接矩形
            cv::rectangle(img,
                          cv::Point((int)t.bbox[0], (int)t.bbox[1]),
                          cv::Point((int)t.bbox[2], (int)t.bbox[3]),
                          cv::Scalar(0, 255, 0), 2);

            // 4个角点
            for (int j = 0; j < 4; ++j)
                cv::circle(img,
                           cv::Point((int)t.corners[j][0], (int)t.corners[j][1]),
                           3, cv::Scalar(0, 0, 255), -1);

            // 中心点
            cv::circle(img,
                       cv::Point((int)t.center[0], (int)t.center[1]),
                       4, cv::Scalar(255, 0, 0), -1);

            // 标签 ID
            cv::putText(img,
                        "ID:" + std::to_string(t.id),
                        cv::Point((int)t.bbox[0], (int)t.bbox[1] - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(0, 255, 0), 2);
        }
    }

    // 绘制调试覆盖信息（FPS、标签数等）
    void drawDebugOverlay(cv::Mat& image, int tag_count, double fps) const
    {
        int y = 30;
        if (fps > 0)
        {
            cv::putText(image,
                        cv::format("FPS: %.1f", fps),
                        cv::Point(10, y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8,
                        cv::Scalar(0, 255, 255), 2);
            y += 30;
        }

        cv::putText(image,
                    "Tags: " + std::to_string(tag_count),
                    cv::Point(10, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(0, 255, 255), 2);
    }

    apriltag_detector_t* td_;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub_;
};

}  // namespace Ten::apriltag_detect

#endif  // __APRILTAG_DEBUG_H_
