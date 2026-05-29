#pragma once
/**
 * @file feature_manager.h
 * @brief 特征管理器 — 管理所有被追踪特征的生命周期
 *
 * 核心数据结构:
 *   FeaturePerFrame: 单个特征在某一帧中的观测
 *   FeaturePerId:    某个特征从首次观测到消失的完整记录
 *   FeatureManager:  管理所有 FeaturePerId, 负责:
 *     - 新特征添加 / 关键帧判选
 *     - 三角化深度估计
 *     - 滑动窗口边缘化时的特征维护
 */

#include <list>
#include <vector>
#include <map>
#include <algorithm>
#include <numeric>

#include <eigen3/Eigen/Dense>
#include "system/parameters.h"

// ============================================================
//  单帧观测数据
// ============================================================

/// 一个特征点在某一帧中的观测信息
class FeaturePerFrame
{
public:
    FeaturePerFrame(const Eigen::Matrix<double, 7, 1>& raw_point, double td)
    {
        // raw_point = [x_norm, y_norm, z_norm, u_pixel, v_pixel, vx, vy]
        point.x() = raw_point(0);
        point.y() = raw_point(1);
        point.z() = raw_point(2);
        uv.x() = raw_point(3);
        uv.y() = raw_point(4);
        velocity.x() = raw_point(5);
        velocity.y() = raw_point(6);
        cur_td = td;
    }

    double cur_td;                  // 时间延迟补偿
    Eigen::Vector3d point;          // 归一化坐标 (x, y, 1)
    Eigen::Vector2d uv;             // 像素坐标 (u, v)
    Eigen::Vector2d velocity;       // 像素速度

    double z = 0;                   // 深度 (投影后)
    bool is_used = false;           // 是否被使用
    double parallax = 0;            // 视差
    Eigen::MatrixXd A;              // 信息矩阵
    Eigen::VectorXd b;              // 信息向量
    double dep_gradient = 0;        // 深度梯度
};

// ============================================================
//  特征完整生命周期
// ============================================================

/// 一个特征从首次被观测到消失的完整记录
class FeaturePerId
{
public:
    const int feature_id;                           // 全局唯一特征 ID
    int start_frame;                                // 首次观测所在的帧索引
    std::vector<FeaturePerFrame> feature_per_frame; // 每帧的观测数据

    int used_num = 0;                // 被使用次数
    bool is_outlier = false;         // 是否为外点
    bool is_margin = false;          // 是否已被边缘化
    double estimated_depth = -1.0;   // 估计深度 (-1 表示未初始化)
    int solve_flag = 0;              // 0:未求解, 1:求解成功, 2:求解失败

    Eigen::Vector3d gt_p;            // ground truth (调试用)

    FeaturePerId(int id, int start)
        : feature_id(id), start_frame(start) {}

    /// 该特征最后一次被观测到的帧索引
    int EndFrame();

    // 向后兼容
    inline int endFrame() { return EndFrame(); }
};

// ============================================================
//  特征管理器
// ============================================================

class FeatureManager
{
public:
    explicit FeatureManager(Eigen::Matrix3d Rs[]);

    /// 设置 IMU-Camera 旋转外参
    void SetRic(Eigen::Matrix3d ric[]);

    /// 清空所有特征状态
    void ClearState();

    /// 获取有效特征数量 (观测>=2帧且已初始化)
    int GetFeatureCount();

    /// 添加新观测, 同时判断是否为关键帧 (基于视差)
    bool AddFeatureCheckParallax(int frame_count,
        const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>& image,
        double td);

    /// 获取两帧之间的特征对应点对
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
        GetCorresponding(int frame_l, int frame_r);

    /// 设置所有特征的逆深度
    void SetDepth(const Eigen::VectorXd& x);

    /// 获取所有特征的逆深度向量
    Eigen::VectorXd GetDepthVector();

    /// 三角化计算特征深度
    void Triangulate(Eigen::Vector3d Ps[], Eigen::Vector3d tic[], Eigen::Matrix3d ric[]);

    /// 移除求解失败的特征
    void RemoveFailures();

    /// 清除深度信息
    void ClearDepth(const Eigen::VectorXd& x);

    /// 边缘化最旧帧时, 更新特征深度到新的参考帧
    void RemoveBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P,
                              Eigen::Matrix3d new_R, Eigen::Vector3d new_P);

    /// 边缘化最旧帧
    void RemoveBack();

    /// 边缘化次新帧
    void RemoveFront(int frame_count);

    /// 移除外点
    void RemoveOutlier();

    /// 调试输出
    void DebugShow();

    // --- 向后兼容旧接口 ---
    inline void setRic(Eigen::Matrix3d r[]) { SetRic(r); }
    inline void clearState() { ClearState(); }
    inline int getFeatureCount() { return GetFeatureCount(); }
    inline bool addFeatureCheckParallax(int fc,
        const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>& img, double td)
        { return AddFeatureCheckParallax(fc, img, td); }
    inline std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
        getCorresponding(int l, int r) { return GetCorresponding(l, r); }
    inline void setDepth(const Eigen::VectorXd& x) { SetDepth(x); }
    inline Eigen::VectorXd getDepthVector() { return GetDepthVector(); }
    inline void triangulate(Eigen::Vector3d Ps[], Eigen::Vector3d tic[], Eigen::Matrix3d ric[])
        { Triangulate(Ps, tic, ric); }
    inline void removeFailures() { RemoveFailures(); }
    inline void clearDepth(const Eigen::VectorXd& x) { ClearDepth(x); }
    inline void removeBackShiftDepth(Eigen::Matrix3d mR, Eigen::Vector3d mP,
                                     Eigen::Matrix3d nR, Eigen::Vector3d nP)
        { RemoveBackShiftDepth(mR, mP, nR, nP); }
    inline void removeBack() { RemoveBack(); }
    inline void removeFront(int fc) { RemoveFront(fc); }
    inline void removeOutlier() { RemoveOutlier(); }
    inline void debugShow() { DebugShow(); }

    // --- 公开数据 ---
    std::list<FeaturePerId> feature;    // 所有特征的生命周期列表
    int last_track_num = 0;             // 上一帧追踪成功的特征数

private:
    /// 计算补偿后的视差 (用于关键帧判选)
    double CompensatedParallax2(const FeaturePerId& it_per_id, int frame_count);

    const Eigen::Matrix3d* Rs_;                 // 指向滑动窗口中所有帧的旋转
    Eigen::Matrix3d ric_[NUM_OF_CAM];           // IMU-Camera 外参旋转
};
