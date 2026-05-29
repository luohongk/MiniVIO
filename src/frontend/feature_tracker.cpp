/**
 * @file feature_tracker.cpp
 * @brief 前端 KLT 光流特征追踪实现
 */

#include "frontend/feature_tracker.h"

using namespace std;
using namespace camodocal;

// 全局特征 ID 计数器
int FeatureTracker::next_id_ = 0;

// ============================================================
//  工具函数
// ============================================================

bool inBorder(const cv::Point2f& pt)
{
    const int kBorderSize = 1;
    int x = cvRound(pt.x);
    int y = cvRound(pt.y);
    return kBorderSize <= x && x < COL - kBorderSize
        && kBorderSize <= y && y < ROW - kBorderSize;
}

void reduceVector(vector<cv::Point2f>& v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int>& v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

// ============================================================
//  构造函数
// ============================================================

FeatureTracker::FeatureTracker()
{
}

// ============================================================
//  主处理流程: ReadImage
// ============================================================

void FeatureTracker::ReadImage(const cv::Mat& image, double timestamp)
{
    cv::Mat img;
    cur_time_ = timestamp;

    // Step 1: 图像预处理 (直方图均衡化)
    if (EQUALIZE)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(image, img);
    }
    else
    {
        img = image;
    }

    // Step 2: 初始化 (第一帧)
    if (forw_img_.empty())
    {
        prev_img_ = cur_img_ = forw_img_ = img;
    }
    else
    {
        forw_img_ = img;
    }

    forw_pts_.clear();

    // Step 3: KLT 光流追踪
    if (cur_pts.size() > 0)
    {
        vector<uchar> status;
        vector<float> err;
        cv::calcOpticalFlowPyrLK(cur_img_, forw_img_, cur_pts, forw_pts_,
                                 status, err, cv::Size(21, 21), 3);

        // 剔除跑出边界的点
        for (int i = 0; i < int(forw_pts_.size()); i++)
            if (status[i] && !inBorder(forw_pts_[i]))
                status[i] = 0;

        reduceVector(prev_pts_, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts_, status);
        reduceVector(ids, status);
        reduceVector(cur_un_pts, status);
        reduceVector(track_cnt, status);
    }

    // Step 4: 递增追踪计数
    for (auto& n : track_cnt)
        n++;

    // Step 5: 外点剔除 + 新特征检测 (仅在发布帧)
    if (PUB_THIS_FRAME)
    {
        RejectWithFundamentalMatrix();

        SetMask();

        // 检测新特征填补空缺
        int max_new_features = MAX_CNT - static_cast<int>(forw_pts_.size());
        if (max_new_features > 0)
        {
            cv::goodFeaturesToTrack(forw_img_, new_pts_,
                                   MAX_CNT - forw_pts_.size(),
                                   0.01, MIN_DIST, mask_);
        }
        else
        {
            new_pts_.clear();
        }

        AddNewFeatures();
    }

    // Step 6: 更新帧缓存
    prev_img_ = cur_img_;
    prev_pts_ = cur_pts;
    prev_un_pts_ = cur_un_pts;
    cur_img_ = forw_img_;
    cur_pts = forw_pts_;

    // Step 7: 去畸变 + 计算速度
    UndistortPoints();
    prev_time_ = cur_time_;
}

// ============================================================
//  设置检测 Mask (已有特征附近不再检测)
// ============================================================

void FeatureTracker::SetMask()
{
    if (FISHEYE)
        mask_ = fisheye_mask_.clone();
    else
        mask_ = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));

    // 按追踪时长排序, 优先保留追踪久的特征
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;
    for (unsigned int i = 0; i < forw_pts_.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts_[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(),
         [](const auto& a, const auto& b) { return a.first > b.first; });

    forw_pts_.clear();
    ids.clear();
    track_cnt.clear();

    for (auto& it : cnt_pts_id)
    {
        if (mask_.at<uchar>(it.second.first) == 255)
        {
            forw_pts_.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask_, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

// ============================================================
//  添加新检测的特征
// ============================================================

void FeatureTracker::AddNewFeatures()
{
    for (auto& p : new_pts_)
    {
        forw_pts_.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
}

// ============================================================
//  基础矩阵 RANSAC 外点剔除
// ============================================================

void FeatureTracker::RejectWithFundamentalMatrix()
{
    if (forw_pts_.size() < 8) return;

    // 将像素坐标转为归一化坐标再投影回像素 (用于 F 矩阵计算)
    vector<cv::Point2f> un_cur_pts(cur_pts.size());
    vector<cv::Point2f> un_forw_pts(forw_pts_.size());

    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector3d p3d;
        camera_->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), p3d);
        un_cur_pts[i] = cv::Point2f(
            FOCAL_LENGTH * p3d.x() / p3d.z() + COL / 2.0,
            FOCAL_LENGTH * p3d.y() / p3d.z() + ROW / 2.0);

        camera_->liftProjective(Eigen::Vector2d(forw_pts_[i].x, forw_pts_[i].y), p3d);
        un_forw_pts[i] = cv::Point2f(
            FOCAL_LENGTH * p3d.x() / p3d.z() + COL / 2.0,
            FOCAL_LENGTH * p3d.y() / p3d.z() + ROW / 2.0);
    }

    vector<uchar> status;
    cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);

    reduceVector(prev_pts_, status);
    reduceVector(cur_pts, status);
    reduceVector(forw_pts_, status);
    reduceVector(cur_un_pts, status);
    reduceVector(ids, status);
    reduceVector(track_cnt, status);
}

// ============================================================
//  分配全局 ID
// ============================================================

bool FeatureTracker::UpdateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = next_id_++;
        return true;
    }
    return false;
}

// ============================================================
//  加载相机内参
// ============================================================

void FeatureTracker::ReadIntrinsicParameter(const string& calib_file)
{
    cout << "[FeatureTracker] Loading camera: " << calib_file << endl;
    camera_ = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

// ============================================================
//  去畸变 + 计算像素速度
// ============================================================

void FeatureTracker::UndistortPoints()
{
    cur_un_pts.clear();
    cur_un_pts_map_.clear();

    // 对每个特征点: 像素坐标 → 去畸变归一化坐标
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d pixel(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d point_3d;
        camera_->liftProjective(pixel, point_3d);

        cv::Point2f normalized_pt(point_3d.x() / point_3d.z(),
                                  point_3d.y() / point_3d.z());
        cur_un_pts.push_back(normalized_pt);
        cur_un_pts_map_.insert(make_pair(ids[i], normalized_pt));
    }

    // 计算像素速度: (当前归一化坐标 - 上一帧归一化坐标) / dt
    pts_velocity.clear();
    if (!prev_un_pts_map_.empty())
    {
        double dt = cur_time_ - prev_time_;
        for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        {
            auto it = prev_un_pts_map_.find(ids[i]);
            if (ids[i] != -1 && it != prev_un_pts_map_.end())
            {
                double vx = (cur_un_pts[i].x - it->second.x) / dt;
                double vy = (cur_un_pts[i].y - it->second.y) / dt;
                pts_velocity.push_back(cv::Point2f(vx, vy));
            }
            else
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
            pts_velocity.push_back(cv::Point2f(0, 0));
    }

    prev_un_pts_map_ = cur_un_pts_map_;
}

// ============================================================
//  调试: 显示去畸变图像
// ============================================================

void FeatureTracker::ShowUndistortion(const string& name)
{
    cv::Mat undistorted_img(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distorted_pts, undistorted_pts;

    for (int i = 0; i < COL; i++)
    {
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d pixel(i, j);
            Eigen::Vector3d point_3d;
            camera_->liftProjective(pixel, point_3d);
            distorted_pts.push_back(pixel);
            undistorted_pts.push_back(Eigen::Vector2d(point_3d.x() / point_3d.z(),
                                                      point_3d.y() / point_3d.z()));
        }
    }

    for (int i = 0; i < int(undistorted_pts.size()); i++)
    {
        float u = undistorted_pts[i].x() * FOCAL_LENGTH + COL / 2;
        float v = undistorted_pts[i].y() * FOCAL_LENGTH + ROW / 2;
        if (v + 300 >= 0 && v + 300 < ROW + 600 && u + 300 >= 0 && u + 300 < COL + 600)
        {
            undistorted_img.at<uchar>(v + 300, u + 300) =
                cur_img_.at<uchar>(distorted_pts[i].y(), distorted_pts[i].x());
        }
    }
    cv::imshow(name, undistorted_img);
    cv::waitKey(0);
}
