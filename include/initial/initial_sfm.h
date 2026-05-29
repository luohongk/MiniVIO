#pragma once
/**
 * @file initial_sfm.h
 * @brief Structure from Motion 初始化
 *
 * 利用滑动窗口中的特征对应, 通过 SFM 恢复所有帧的位姿和特征 3D 坐标:
 *   1. 选取参考帧对, 用五点法恢复相对位姿
 *   2. 三角化特征点
 *   3. PnP 恢复其余帧位姿
 *   4. Ceres 全局 BA 优化
 */

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <map>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

/// SFM 中的特征观测记录
struct SFMFeature
{
    bool state = false;                  // 是否已三角化
    int id = -1;                         // 特征 ID
    std::vector<std::pair<int, Eigen::Vector2d>> observation;  // (帧索引, 归一化坐标)
    double position[3] = {0};            // 三角化后的 3D 坐标
    double depth = -1;                   // 深度
};

/// Ceres 重投影误差 (用于 SFM 全局 BA)
struct ReprojectionError3D
{
    ReprojectionError3D(double observed_u, double observed_v)
        : observed_u(observed_u), observed_v(observed_v) {}

    template <typename T>
    bool operator()(const T* const camera_R, const T* const camera_T,
                    const T* point, T* residuals) const
    {
        T p[3];
        ceres::QuaternionRotatePoint(camera_R, point, p);
        p[0] += camera_T[0];
        p[1] += camera_T[1];
        p[2] += camera_T[2];
        T xp = p[0] / p[2];
        T yp = p[1] / p[2];
        residuals[0] = xp - T(observed_u);
        residuals[1] = yp - T(observed_v);
        return true;
    }

    static ceres::CostFunction* Create(double observed_x, double observed_y)
    {
        return new ceres::AutoDiffCostFunction<ReprojectionError3D, 2, 4, 3, 3>(
            new ReprojectionError3D(observed_x, observed_y));
    }

    double observed_u;
    double observed_v;
};

/// 全局 SFM 求解器
class GlobalSFM
{
public:
    GlobalSFM();

    /// 构建 SFM: 恢复所有帧位姿和特征 3D 坐标
    /// @param frame_num 帧数
    /// @param q 输出: 每帧的旋转四元数
    /// @param T 输出: 每帧的平移
    /// @param l 参考帧索引
    /// @param relative_R 参考帧对的相对旋转
    /// @param relative_T 参考帧对的相对平移
    /// @param sfm_f 特征观测数据
    /// @param sfm_tracked_points 输出: 特征 ID → 3D 坐标
    bool Construct(int frame_num, Eigen::Quaterniond* q, Eigen::Vector3d* T, int l,
                   const Eigen::Matrix3d relative_R, const Eigen::Vector3d relative_T,
                   std::vector<SFMFeature>& sfm_f,
                   std::map<int, Eigen::Vector3d>& sfm_tracked_points);

    // 向后兼容
    inline bool construct(int frame_num, Eigen::Quaterniond* q, Eigen::Vector3d* T, int l,
                          const Eigen::Matrix3d relative_R, const Eigen::Vector3d relative_T,
                          std::vector<SFMFeature>& sfm_f,
                          std::map<int, Eigen::Vector3d>& sfm_tracked_points)
    { return Construct(frame_num, q, T, l, relative_R, relative_T, sfm_f, sfm_tracked_points); }

private:
    /// PnP 求解单帧位姿
    bool SolveFrameByPnP(Eigen::Matrix3d& R_initial, Eigen::Vector3d& P_initial,
                         int i, std::vector<SFMFeature>& sfm_f);

    /// 三角化单个点
    void TriangulatePoint(Eigen::Matrix<double, 3, 4>& Pose0,
                          Eigen::Matrix<double, 3, 4>& Pose1,
                          Eigen::Vector2d& point0, Eigen::Vector2d& point1,
                          Eigen::Vector3d& point_3d);

    /// 三角化两帧之间的所有共视特征
    void TriangulateTwoFrames(int frame0, Eigen::Matrix<double, 3, 4>& Pose0,
                              int frame1, Eigen::Matrix<double, 3, 4>& Pose1,
                              std::vector<SFMFeature>& sfm_f);

    int feature_num_;
};
