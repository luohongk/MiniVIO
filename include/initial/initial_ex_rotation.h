#pragma once
/**
 * @file initial_ex_rotation.h
 * @brief IMU-Camera 外参旋转在线标定
 *
 * 当外参完全未知时, 利用 IMU 旋转增量和视觉旋转增量
 * 的一致性约束来估计 IMU 到 Camera 的旋转矩阵
 */

#include <vector>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include "system/parameters.h"

/// IMU-Camera 外参旋转标定器
class InitialEXRotation
{
public:
    InitialEXRotation();

    /// 利用视觉和 IMU 旋转增量标定外参旋转
    /// @param corres 两帧之间的特征对应
    /// @param delta_q_imu IMU 预积分得到的旋转增量
    /// @param calib_ric_result 输出标定结果
    /// @return 标定是否收敛
    bool CalibrationExRotation(
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corres,
        Eigen::Quaterniond delta_q_imu,
        Eigen::Matrix3d& calib_ric_result);

private:
    /// 从特征对应求解相对旋转 (本质矩阵分解)
    Eigen::Matrix3d SolveRelativeR(
        const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& corres);

    double TestTriangulation(const std::vector<cv::Point2f>& l,
                             const std::vector<cv::Point2f>& r,
                             cv::Mat_<double> R, cv::Mat_<double> t);

    void DecomposeE(cv::Mat E,
                    cv::Mat_<double>& R1, cv::Mat_<double>& R2,
                    cv::Mat_<double>& t1, cv::Mat_<double>& t2);

    int frame_count_;
    std::vector<Eigen::Matrix3d> Rc_;      // 视觉相对旋转序列
    std::vector<Eigen::Matrix3d> Rimu_;    // IMU 相对旋转序列
    std::vector<Eigen::Matrix3d> Rc_g_;    // 全局相机旋转
    Eigen::Matrix3d ric_;                  // 标定结果
};
