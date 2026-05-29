#pragma once
/**
 * @file initial_alignment.h
 * @brief 视觉-惯性对齐 (Visual-Inertial Alignment)
 *
 * 在 SFM 成功后, 将视觉尺度与 IMU 预积分对齐:
 *   - 估计陀螺仪零偏
 *   - 恢复重力方向和尺度
 *   - 估计每帧速度
 */

#include <map>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "factor/integration_base.h"
#include "utility/utility.h"
#include "frontend/feature_manager.h"

/// 单帧图像数据 (用于初始化阶段的帧管理)
class ImageFrame
{
public:
    ImageFrame() {}
    ImageFrame(const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>& _points, double _t)
        : points(_points), t(_t), is_key_frame(false) {}

    std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> points;  // 特征观测
    double t = 0;                          // 时间戳
    Eigen::Matrix3d R;                     // 旋转 (SFM 结果)
    Eigen::Vector3d T;                     // 平移 (SFM 结果)
    IntegrationBase* pre_integration = nullptr;  // 该帧对应的 IMU 预积分
    bool is_key_frame = false;             // 是否为关键帧
};

/// 视觉-惯性对齐: 估计陀螺零偏、重力方向、尺度、速度
/// @return 对齐是否成功
bool VisualIMUAlignment(std::map<double, ImageFrame>& all_image_frame,
                        Eigen::Vector3d* Bgs,
                        Eigen::Vector3d& g,
                        Eigen::VectorXd& x);
