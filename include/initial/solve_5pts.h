#pragma once
/**
 * @file solve_5pts.h
 * @brief 五点法求解本质矩阵 + 恢复相对位姿 (R, T)
 */

#include <vector>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

/// 运动估计器: 用对极几何从特征对应恢复相对旋转和平移
class MotionEstimator
{
public:
    /// 从特征对应点对求解相对 R, T (五点法 + RANSAC)
    bool SolveRelativeRT(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& corres,
                         Eigen::Matrix3d& R, Eigen::Vector3d& T);

    // 向后兼容
    inline bool solveRelativeRT(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& corres,
                                Eigen::Matrix3d& R, Eigen::Vector3d& T)
    { return SolveRelativeRT(corres, R, T); }

private:
    /// 验证三角化结果是否合理 (正深度比例)
    double TestTriangulation(const std::vector<cv::Point2f>& l,
                             const std::vector<cv::Point2f>& r,
                             cv::Mat_<double> R, cv::Mat_<double> t);

    /// 分解本质矩阵为 R1, R2, t1, t2
    void DecomposeE(cv::Mat E,
                    cv::Mat_<double>& R1, cv::Mat_<double>& R2,
                    cv::Mat_<double>& t1, cv::Mat_<double>& t2);
};
