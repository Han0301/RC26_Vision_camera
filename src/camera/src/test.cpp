#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <limits>
#include <string>
#include <vector>
#include <utility>
#include <cmath>
#include <cstdlib>
#include <sys/stat.h>
#include <dirent.h>
#include <algorithm>
#include <thread>
#include <mutex>
#include <atomic>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "camera.h"
#include "apriltag_detect/apriltag_detector.h"
#include "apriltag_detect/apriltag_debug.h"

// ---------- 非阻塞键盘输入 ----------
static struct termios g_original_tio;

static void setupNonBlockingInput()
{
    tcgetattr(STDIN_FILENO, &g_original_tio);
    struct termios tio = g_original_tio;
    tio.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &tio);
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}

static void restoreInput()
{
    tcsetattr(STDIN_FILENO, TCSANOW, &g_original_tio);
}

// 非阻塞读取一个字符, 无输入返回 -1
static int nb_getchar()
{
    char c;
    int r = read(STDIN_FILENO, &c, 1);
    return (r == 1) ? (int)c : -1;
}

// ---------- 目录创建 ----------
static void ensureDir(const std::string& path)
{
    struct stat st;
    if (stat(path.c_str(), &st) != 0)
        mkdir(path.c_str(), 0755);
}

// ---------- 录制控制状态 ----------
struct RecordState
{
    bool     active    = false;
    int      frame_cnt = 0;
    int      save_cnt  = 0;
    std::string images_dir;
    std::string labels_dir;

    RecordState(const std::string& base_dir = "/home/h/output")
        : images_dir(base_dir + "/images")
        , labels_dir(base_dir + "/labels")
    {
        ensureDir(base_dir);
        ensureDir(images_dir);
        ensureDir(labels_dir);
    }
};

// ---------- 显示线程共享数据 ----------
struct DisplayFrame
{
    cv::Mat image;
    bool    detected = false;
    std::vector<Ten::apriltag_detect::TagDetection> tags;
    bool    recording = false;
    int     frame_cnt = 0;
    int     save_cnt  = 0;
};

static std::mutex    g_disp_mutex;
static DisplayFrame  g_disp_frame;
static std::atomic<bool> g_disp_running{true};
static std::atomic<bool> g_quit_from_gui{false};

// 在显示帧上绘制检测框
static void drawDetections(cv::Mat& img,
                           const std::vector<Ten::apriltag_detect::TagDetection>& tags)
{
    for (const auto& t : tags)
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

// 显示线程
static void displayThread()
{
    cv::namedWindow("AprilTag Detection", cv::WINDOW_NORMAL);
    cv::resizeWindow("AprilTag Detection", 640, 480);

    while (g_disp_running.load())
    {
        cv::Mat disp;
        {
            std::lock_guard<std::mutex> lock(g_disp_mutex);
            if (!g_disp_frame.image.empty())
                g_disp_frame.image.copyTo(disp);
        }

        if (!disp.empty())
        {
            // 绘制检测框
            if (!g_disp_frame.tags.empty())
                drawDetections(disp, g_disp_frame.tags);

            // 状态文字
            std::string rec_str = g_disp_frame.recording ? "REC ON" : "REC OFF";
            cv::Scalar rec_color = g_disp_frame.recording
                                   ? cv::Scalar(0, 0, 255)
                                   : cv::Scalar(128, 128, 128);
            cv::putText(disp, rec_str,
                        cv::Point(10, 25),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, rec_color, 2);

            if (g_disp_frame.recording)
            {
                cv::putText(disp,
                            "frame:" + std::to_string(g_disp_frame.frame_cnt) +
                            " saved:" + std::to_string(g_disp_frame.save_cnt),
                            cv::Point(10, 55),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5,
                            cv::Scalar(255, 255, 0), 1);
            }

            cv::imshow("AprilTag Detection", disp);
        }

        // waitKey 返回按键, 'q' 或 ESC 退出
        int k = cv::waitKey(16);  // ~60fps 刷新率
        if (k == 'q' || k == 'Q' || k == 27)
        {
            g_quit_from_gui.store(true);
            g_disp_running.store(false);
            break;
        }
    }

    cv::destroyAllWindows();
}

// ---------- YOLOv5 格式 JSON 生成 ----------
static void saveYoloJson(const std::string& json_path,
                         const std::string& image_name,
                         int img_w, int img_h,
                         const std::vector<Ten::apriltag_detect::TagDetection>& detections)
{
    std::ofstream ofs(json_path);
    if (!ofs.is_open())
    {
        std::cerr << "[JSON] Failed to open: " << json_path << std::endl;
        return;
    }

    ofs << "[\n";
    for (size_t i = 0; i < detections.size(); ++i)
    {
        const auto& d = detections[i];
        ofs << "  {\n";
        ofs << "    \"name\": \"apriltag_" << d.id << "\",\n";
        ofs << "    \"class\": 0,\n";
        ofs << "    \"confidence\": " << std::fixed << std::setprecision(4) << d.confidence << ",\n";
        ofs << "    \"bbox\": {\n";
        ofs << "      \"x1\": " << d.bbox[0] << ",\n";
        ofs << "      \"y1\": " << d.bbox[1] << ",\n";
        ofs << "      \"x2\": " << d.bbox[2] << ",\n";
        ofs << "      \"y2\": " << d.bbox[3] << "\n";
        ofs << "    },\n";
        ofs << "    \"image_size\": {\"width\": " << img_w << ", \"height\": " << img_h << "},\n";
        ofs << "    \"image_name\": \"" << image_name << "\",\n";
        ofs << "    \"corners\": [\n";
        for (int j = 0; j < 4; ++j)
        {
            ofs << "      [" << d.corners[j][0] << ", " << d.corners[j][1] << "]";
            ofs << (j < 3 ? ",\n" : "\n");
        }
        ofs << "    ]\n";
        ofs << "  }" << (i < detections.size() - 1 ? "," : "") << "\n";
    }
    ofs << "]\n";
    ofs.close();
}

// ---------- 主循环 ----------
void test()
{
    Ten::Ten_camera& camera = Ten::Ten_camera::GetInstance();
    camera.reset_camera(640, 480, 60);
    Ten::apriltag_detect::AprilTagDetector Apriltag_d;

    setupNonBlockingInput();

    // 启动显示线程
    std::thread disp_thread(displayThread);

    RecordState rec("/home/h/output");
    std::cout << "======================================" << std::endl;
    std::cout << "  Keyboard / GUI Control:" << std::endl;
    std::cout << "    press 'i' to START recording" << std::endl;
    std::cout << "    press 'o' to STOP  recording" << std::endl;
    std::cout << "    press 'q' or close window to QUIT" << std::endl;
    std::cout << "  Every 3 frames -> save 1 frame" << std::endl;
    std::cout << "  Output dir: /home/h/output/" << std::endl;
    std::cout << "======================================" << std::endl;

    while (ros::ok() && !g_quit_from_gui.load())
    {
        // 1. 处理键盘输入 (非阻塞)
        int key = nb_getchar();
        while (key != -1)
        {
            if (key == 'i' || key == 'I')
            {
                rec.active = true;
                rec.frame_cnt = 0;
                std::cout << "\n>>> [RECORD START] frame_cnt reset, save_cnt="
                          << rec.save_cnt << std::endl;
            }
            else if (key == 'o' || key == 'O')
            {
                rec.active = false;
                std::cout << "\n>>> [RECORD STOP] total saved="
                          << rec.save_cnt << std::endl;
            }
            else if (key == 'q' || key == 'Q')
            {
                std::cout << "\n>>> [QUIT from keyboard]" << std::endl;
                goto cleanup;
            }
            key = nb_getchar();
        }

        // 2. 读取相机帧
        cv::Mat frame = camera.camera_read();
        if (frame.empty())
        {
            std::cout << "frame.empty()" << std::endl;
            ros::Duration(0.01).sleep();
            continue;
        }

        // 3. 实时检测 (始终运行, 不受录制状态影响)
        bool detected = Apriltag_d.detect(frame);

        // 终端打印检测状态 (仅状态变化时打印, 避免刷屏)
        {
            static bool last_detected = false;
            if (detected != last_detected)
            {
                if (detected)
                    std::cout << "[DETECT] found " << Apriltag_d.detections().size()
                              << " tag(s)" << std::endl;
                else
                    std::cout << "[DETECT] no tag" << std::endl;
                last_detected = detected;
            }
        }

        // 4. 仅在录制状态下保存
        if (rec.active)
        {
            rec.frame_cnt++;

            if (rec.frame_cnt % 3 == 0)
            {
                std::ostringstream oss;
                oss << std::setw(6) << std::setfill('0') << rec.save_cnt;
                std::string idx_str = oss.str();

                std::string img_name = idx_str + ".jpg";
                std::string img_path = rec.images_dir + "/" + img_name;

                cv::imwrite(img_path, frame);
                std::cout << "[SAVE] " << img_name << "  detected="
                          << (detected ? "YES" : "NO");

                if (detected)
                {
                    const auto& detections = Apriltag_d.detections();
                    std::string json_path = rec.labels_dir + "/" + idx_str + ".json";
                    saveYoloJson(json_path, img_name,
                                 frame.cols, frame.rows, detections);
                    std::cout << "  tags=" << detections.size()
                              << "  json=" << idx_str << ".json";
                }

                std::cout << std::endl;
                rec.save_cnt++;
            }

            if (rec.frame_cnt % 30 == 0)
            {
                std::cout << "[REC] frame=" << rec.frame_cnt
                          << "  saved=" << rec.save_cnt << std::endl;
            }
        }

        // 5. 更新显示帧 (线程安全)
        {
            std::lock_guard<std::mutex> lock(g_disp_mutex);
            frame.copyTo(g_disp_frame.image);
            g_disp_frame.detected  = detected;
            g_disp_frame.tags      = Apriltag_d.detections();
            g_disp_frame.recording = rec.active;
            g_disp_frame.frame_cnt = rec.frame_cnt;
            g_disp_frame.save_cnt  = rec.save_cnt;
        }

        // 6. 让 ROS 处理回调
        ros::spinOnce();
    }

cleanup:
    // 通知显示线程退出
    g_disp_running.store(false);
    restoreInput();

    if (disp_thread.joinable())
        disp_thread.join();

    std::cout << "Exited. Total saved frames: " << rec.save_cnt << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    test();
    return 0;
}