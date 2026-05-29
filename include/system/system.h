#pragma once
/**
 * @file System.h
 * @brief VIO 系统主类，协调前端特征追踪、后端优化、可视化
 *
 * 数据流:
 *   IMU数据 ---> imu_buf ----+
 *                             +--> getMeasurements() --> ProcessBackEnd()
 *   图像数据 --> 前端追踪 --> feature_buf --+
 *
 *   可视化线程(Draw): 读取 vPath_to_draw 绘制3D轨迹
 */

#include <queue>
#include <mutex>
#include <thread>
#include <fstream>
#include <condition_variable>

#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>

#include "system/estimator.h"
#include "system/parameters.h"
#include "frontend/feature_tracker.h"

// ============================================================
//  传感器数据消息结构
// ============================================================

/// IMU 测量数据 (单次采样)
struct ImuMessage
{
    double timestamp;                    // 时间戳 (秒)
    Eigen::Vector3d accelerometer;       // 加速度计 (m/s^2)
    Eigen::Vector3d gyroscope;           // 陀螺仪 (rad/s)
};
using ImuMessageConstPtr = std::shared_ptr<ImuMessage const>;

/// 图像特征数据 (经前端提取后的特征点集合)
struct FeatureMessage
{
    double timestamp;                               // 时间戳 (秒)
    std::vector<Eigen::Vector3d> points;            // 归一化坐标 (x, y, 1)
    std::vector<int>   feature_ids;                 // 特征 ID
    std::vector<float> pixel_u;                     // 像素坐标 u
    std::vector<float> pixel_v;                     // 像素坐标 v
    std::vector<float> velocity_x;                  // x 方向像素速度
    std::vector<float> velocity_y;                  // y 方向像素速度
};
using FeatureMessageConstPtr = std::shared_ptr<FeatureMessage const>;

// ============================================================
//  VIO 系统主类
// ============================================================

class System
{
public:
    // --- 构造/析构 ---
    explicit System(const std::string& config_path);
    ~System();

    // --- 数据输入接口 (由数据发布线程调用) ---
    void PubImuData(double timestamp_sec,
                    const Eigen::Vector3d& gyroscope,
                    const Eigen::Vector3d& accelerometer);

    void PubImageData(double timestamp_sec, cv::Mat& image);

    // --- 线程入口函数 ---
    void ProcessBackEnd();   // 后端优化线程
    void Draw();             // 3D 可视化线程

    // --- 公开成员 (可视化需要访问) ---
    FeatureTracker trackerData[NUM_OF_CAM];

private:
    // ----- 前端状态 -----
    bool   is_first_feature_ = false;   // 是否已跳过第一帧 (无光流速度)
    bool   is_first_image_   = true;    // 第一帧标志
    double first_image_time_ = 0;       // 第一帧时间 (用于频率控制)
    double last_image_time_  = 0;       // 上一帧时间
    int    pub_count_        = 1;       // 已发布帧计数
    bool   init_pub_         = false;   // 是否已完成初始化发布

    // ----- 后端估计器 -----
    Estimator estimator_;

    // ----- 线程同步 -----
    std::condition_variable data_ready_cond_;   // 数据就绪条件变量
    std::mutex buf_mutex_;                      // 保护 imu_buf / feature_buf
    std::mutex estimator_mutex_;                // 保护 estimator 状态

    // ----- 数据缓冲队列 -----
    std::queue<ImuMessageConstPtr>     imu_buf_;
    std::queue<FeatureMessageConstPtr> feature_buf_;

    // ----- 后端辅助状态 -----
    double current_time_ = -1;
    double last_imu_time_ = 0;
    int    wait_count_ = 0;
    bool   backend_running_ = true;

    // ----- 输出 -----
    std::ofstream pose_output_file_;
    std::vector<Eigen::Vector3d> trajectory_;   // 用于 3D 可视化

    // ----- 可视化辅助 -----
    pangolin::OpenGlRenderState gl_render_state_;
    pangolin::View              gl_display_;
    double viz_fps_ = 0.0;
    double prev_frame_time_ = 0.0;

    // ----- 私有方法 -----

    /// 从缓冲区中取出时间对齐的 (IMU序列, 特征) 测量对
    using Measurement = std::pair<std::vector<ImuMessageConstPtr>, FeatureMessageConstPtr>;
    std::vector<Measurement> getMeasurements();

    /// 2D 特征可视化 (在 PubImageData 中调用)
    void visualizeFeatures(const cv::Mat& image);

    /// 绘制相机锥体
    void drawCameraFrustum(const Eigen::Vector3d& position,
                           const Eigen::Matrix3d& rotation,
                           bool is_current_frame);
};
