#ifndef __CAMERA_H_
#define __CAMERA_H_
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <mutex>
#include <unistd.h>

namespace Ten
{

struct camera_frame
{
    cv::Mat bgr_image;
    cv::Mat depth_image;
    std::shared_ptr<rs2::depth_frame> raw_depth_frame;
};
    


class Ten_camera
{
public:
    //禁用拷贝构造
    Ten_camera(const Ten_camera& serial) = delete;
    //禁用赋值
    Ten_camera& operator=(const Ten_camera& serial) = delete;
    /**
        @brief 设置分辨率和帧率
        @param w: 1920 640
        @param h: 1080 480
        @param fps: 帧率
        @return Ten_camera& 返回Ten_camera实例
    */
    static Ten_camera& GetInstance(size_t w = 1920, size_t h = 1080, size_t fps = 30);

    /** 
        @brief 读取图片 
        @return cv::Mat 
    */
    cv::Mat camera_read();

    /** 
        @brief 读取bgr和depth图片
        @return 结构体 camera_frame
    */
    camera_frame camera_read_depth();

    /**
        @brief 设置分辨率和帧率
        @param w: 1920 640
        @param h: 1080 480
        @param fps: 帧率
    */
    void reset_camera(size_t w, size_t h, size_t fps);

    // /**
    //     @brief 高效读取图片
    //     @param int: 无实际意义，用于函数重载
    //     @return cv::Mat* 外面要delete 对象
    // */

    // cv::Mat* camera_read(int);
    /**
        @brief 设置分辨率和帧率（启用深度和bgr双加载）
        @param w: 1920 640
        @param h: 1080 480
        @param fps: 帧率
    */
    void reset_camera_depth(size_t w, size_t h, size_t fps);

    rs2_intrinsics get_color_intrinsics();

    ~Ten_camera()
    {
        pipe.stop();
    }
private:
    // 内部构造函数：直接使用已配置好的 config 启动管道
    Ten_camera(rs2::config&& cfg, size_t w, size_t h, rs2_format fmt)
        : config(std::move(cfg))
        , color_fmt_(fmt)
    {
        pipe.start(config);
        _w = w;
        _h = h;
        std::cout << "camera started: " << w << "x" << h << std::endl;
    }

    // 如果相机输出 RGB8，原地转换为 BGR8
    void ensureBGR(cv::Mat& img) const
    {
        if (color_fmt_ == RS2_FORMAT_RGB8)
            cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
    }

    static std::unique_ptr<Ten_camera> create(size_t w, size_t h, size_t fps) {
        // 尝试候选配置列表，直到成功
        struct Candidate { size_t w, h; int fps; rs2_format fmt; };
        std::vector<Candidate> candidates = {
            {w, h, (int)fps, RS2_FORMAT_BGR8},            // ① 完全匹配
            {w, h, (int)fps, RS2_FORMAT_RGB8},            // ② 同分辨率帧率，格式降级
            {w, h, 30,        RS2_FORMAT_BGR8},            // ③ 降帧到 30
            {w, h, 30,        RS2_FORMAT_RGB8},
            {1280, 720, 30,   RS2_FORMAT_BGR8},            // ④ 降分辨率
            {640, 480, 30,    RS2_FORMAT_BGR8},            // ⑤ 最终兜底
        };

        for (const auto& c : candidates)
        {
            try
            {
                rs2::config cfg;
                cfg.enable_stream(RS2_STREAM_COLOR, c.w, c.h, c.fmt, c.fps);
                auto cam = std::unique_ptr<Ten_camera>(
                    new Ten_camera(std::move(cfg), c.w, c.h, c.fmt));
                std::cout << "[camera] ✅ " << c.w << "x" << c.h
                          << "@" << c.fps << " "
                          << (c.fmt == RS2_FORMAT_BGR8 ? "BGR8" : "RGB8") << std::endl;
                return cam;
            }
            catch (const rs2::error& e)
            {
                std::cerr << "[camera] ❌ " << c.w << "x" << c.h
                          << "@" << c.fps << " — " << e.what() << std::endl;
            }
        }

        throw std::runtime_error("No supported camera configuration found!");
    }

    rs2::pipeline pipe;
    rs2::config config;
    rs2_format    color_fmt_ = RS2_FORMAT_BGR8;
    rs2_intrinsics color_intr_;
    std::mutex read_mtx_;
    size_t _w = 0;
    size_t _h = 0;
    static std::once_flag camera_flag_;
    rs2::align align_to_color_{RS2_STREAM_COLOR};

};


}
#endif


