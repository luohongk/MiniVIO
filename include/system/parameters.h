#pragma once
/**
 * @file parameters.h
 * @brief 全局参数定义与加载
 *
 * 参数来源: euroc_config.yaml
 * 命名约定:
 *   - 编译期常量: UPPER_CASE (SLAM 社区惯例, 如 WINDOW_SIZE)
 *   - 运行期参数: UPPER_CASE extern (保持向后兼容)
 *   - 枚举: PascalCase
 *   - 函数: PascalCase
 */

#include <vector>
#include <string>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include "utility/utility.h"

// ============================================================
//  编译期常量
// ============================================================

constexpr int NUM_OF_CAM = 1;        // 相机数量
constexpr int WINDOW_SIZE = 10;      // 滑动窗口大小
constexpr int NUM_OF_F = 1000;       // 最大特征数

// ============================================================
//  前端参数 (Feature Tracker)
// ============================================================

extern int    FOCAL_LENGTH;          // 焦距 (像素, 用于归一化)
extern std::string IMAGE_TOPIC;      // 图像 topic
extern std::string IMU_TOPIC;        // IMU topic
extern std::string FISHEYE_MASK;     // 鱼眼 mask 路径
extern std::vector<std::string> CAM_NAMES;  // 相机配置文件列表
extern int    MAX_CNT;               // 最大特征点数
extern int    MIN_DIST;              // 特征点最小间距 (像素)
extern int    FREQ;                  // 发布频率 (Hz)
extern double F_THRESHOLD;           // RANSAC 阈值 (像素)
extern int    SHOW_TRACK;            // 是否显示追踪结果
extern bool   STEREO_TRACK;          // 是否双目追踪
extern int    EQUALIZE;              // 直方图均衡化
extern int    FISHEYE;               // 是否鱼眼相机
extern bool   PUB_THIS_FRAME;        // 当前帧是否发布 (由频率控制设置)

// ============================================================
//  后端参数 (Estimator)
// ============================================================

extern double INIT_DEPTH;            // 特征初始深度
extern double MIN_PARALLAX;          // 关键帧判选视差阈值

extern double ACC_N;                 // 加速度计噪声标准差
extern double ACC_W;                 // 加速度计零偏随机游走
extern double GYR_N;                 // 陀螺仪噪声标准差
extern double GYR_W;                 // 陀螺仪零偏随机游走

extern std::vector<Eigen::Matrix3d> RIC;  // IMU-Camera 旋转外参
extern std::vector<Eigen::Vector3d> TIC;  // IMU-Camera 平移外参
extern Eigen::Vector3d G;                 // 重力向量

extern double BIAS_ACC_THRESHOLD;    // 加速度计零偏阈值
extern double BIAS_GYR_THRESHOLD;    // 陀螺仪零偏阈值
extern double SOLVER_TIME;           // 优化最大耗时 (秒)
extern int    NUM_ITERATIONS;        // 优化最大迭代次数
extern int    ESTIMATE_EXTRINSIC;    // 外参估计模式 (0:固定, 1:优化, 2:标定)
extern int    ESTIMATE_TD;           // 是否在线估计时间延迟
extern int    ROLLING_SHUTTER;       // 是否卷帘快门
extern double TD;                    // 时间延迟初值
extern double TR;                    // 卷帘快门读出时间
extern double ROW, COL;              // 图像尺寸

extern std::string EX_CALIB_RESULT_PATH;
extern std::string VINS_RESULT_PATH;

// ============================================================
//  参数加载函数
// ============================================================

/// 从 YAML 配置文件加载所有参数
void ReadParameters(const std::string& config_file);

/// 向后兼容旧接口
inline void readParameters(std::string config_file) { ReadParameters(config_file); }

// ============================================================
//  枚举定义
// ============================================================

/// 位姿参数化维度
enum SizeParameterization
{
    SIZE_POSE = 7,       // [px, py, pz, qx, qy, qz, qw]
    SIZE_SPEEDBIAS = 9,  // [vx, vy, vz, bax, bay, baz, bgx, bgy, bgz]
    SIZE_FEATURE = 1     // [inverse_depth]
};

/// 状态向量排列顺序
enum StateOrder
{
    O_P = 0,     // position
    O_R = 3,     // rotation
    O_V = 6,     // velocity
    O_BA = 9,    // acc bias
    O_BG = 12    // gyro bias
};

/// 噪声向量排列顺序
enum NoiseOrder
{
    O_AN = 0,    // acc noise
    O_GN = 3,    // gyro noise
    O_AW = 6,    // acc walk
    O_GW = 9     // gyro walk
};
