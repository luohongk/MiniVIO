#pragma once
/**
 * @file feature_tracker.h
 * @brief 前端特征追踪器
 *
 * 功能:
 *   - KLT 光流追踪 (帧间特征匹配)
 *   - 基础矩阵 RANSAC 外点剔除
 *   - Harris 角点检测 (补充新特征)
 *   - 去畸变 + 归一化坐标计算
 *   - 特征点像素速度估计
 */

#include <cstdio>
#include <iostream>
#include <queue>
#include <vector>
#include <map>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "system/parameters.h"
#include "utility/tic_toc.h"

// ============================================================
//  工具函数
// ============================================================

/// 判断像素点是否在图像边界内
bool inBorder(const cv::Point2f& pt);

/// 根据 status 向量剔除无效元素
void reduceVector(std::vector<cv::Point2f>& v, std::vector<uchar> status);
void reduceVector(std::vector<int>& v, std::vector<uchar> status);

// ============================================================
//  特征追踪器类
// ============================================================

class FeatureTracker
{
public:
    FeatureTracker();

    // --- 主要接口 ---

    /// 处理一帧图像: 光流追踪 + 外点剔除 + 新特征检测 + 去畸变
    void ReadImage(const cv::Mat& image, double timestamp);

    /// 为新检测到的特征分配全局唯一 ID
    bool UpdateID(unsigned int i);

    /// 加载相机内参 (用于去畸变)
    void ReadIntrinsicParameter(const std::string& calib_file);

    // --- 向后兼容旧接口 ---
    inline void readImage(const cv::Mat& img, double t) { ReadImage(img, t); }
    inline bool updateID(unsigned int i) { return UpdateID(i); }
    inline void readIntrinsicParameter(const std::string& f) { ReadIntrinsicParameter(f); }

    // --- 追踪结果 (供外部读取) ---
    std::vector<cv::Point2f> cur_pts;       // 当前帧像素坐标
    std::vector<cv::Point2f> cur_un_pts;    // 当前帧去畸变归一化坐标
    std::vector<cv::Point2f> pts_velocity;  // 特征点像素速度
    std::vector<int> ids;                   // 特征点全局 ID
    std::vector<int> track_cnt;             // 每个特征的连续追踪帧数

private:
    // --- 内部方法 ---

    /// 设置特征检测 mask (已有特征周围不再检测)
    void SetMask();

    /// 将新检测的特征加入追踪列表
    void AddNewFeatures();

    /// 用基础矩阵 RANSAC 剔除外点
    void RejectWithFundamentalMatrix();

    /// 对当前帧特征进行去畸变, 计算归一化坐标和像素速度
    void UndistortPoints();

    /// 显示去畸变图像 (调试用)
    void ShowUndistortion(const std::string& name);

    // --- 内部状态 ---
    cv::Mat mask_;                           // 特征检测 mask
    cv::Mat fisheye_mask_;                   // 鱼眼相机 mask
    cv::Mat prev_img_, cur_img_, forw_img_;  // 前一帧/当前帧/最新帧图像
    std::vector<cv::Point2f> new_pts_;       // 新检测的角点
    std::vector<cv::Point2f> prev_pts_;      // 前一帧像素坐标
    std::vector<cv::Point2f> forw_pts_;      // 最新帧像素坐标 (光流追踪结果)
    std::vector<cv::Point2f> prev_un_pts_;   // 前一帧去畸变坐标

    std::map<int, cv::Point2f> cur_un_pts_map_;   // 当前帧 id->归一化坐标
    std::map<int, cv::Point2f> prev_un_pts_map_;  // 前一帧 id->归一化坐标

    camodocal::CameraPtr camera_;            // 相机模型 (去畸变)
    double cur_time_  = 0;                   // 当前帧时间戳
    double prev_time_ = 0;                   // 前一帧时间戳

    static int next_id_;                     // 全局特征 ID 计数器
};
