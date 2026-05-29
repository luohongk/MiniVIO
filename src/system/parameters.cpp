/**
 * @file parameters.cpp
 * @brief 从 YAML 配置文件加载所有 VIO 参数
 */

#include "system/parameters.h"
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

// ============================================================
//  全局参数定义
// ============================================================

// 前端参数
int    FOCAL_LENGTH;
string IMAGE_TOPIC;
string IMU_TOPIC;
string FISHEYE_MASK;
vector<string> CAM_NAMES;
int    MAX_CNT;
int    MIN_DIST;
int    FREQ;
double F_THRESHOLD;
int    SHOW_TRACK;
bool   STEREO_TRACK;
int    EQUALIZE;
int    FISHEYE;
bool   PUB_THIS_FRAME;

// 后端参数
double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;
vector<Eigen::Matrix3d> RIC;
vector<Eigen::Vector3d> TIC;
Eigen::Vector3d G{0.0, 0.0, 9.8};
double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int    NUM_ITERATIONS;
int    ESTIMATE_EXTRINSIC;
int    ESTIMATE_TD;
int    ROLLING_SHUTTER;
double ROW, COL;
double TD, TR;
string EX_CALIB_RESULT_PATH;
string VINS_RESULT_PATH;

// ============================================================
//  参数加载
// ============================================================

void ReadParameters(const string& config_file)
{
    cv::FileStorage fs(config_file, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        cerr << "[Parameters] ERROR: Cannot open " << config_file << endl;
        return;
    }

    // --- IMU/Image topics ---
    fs["imu_topic"] >> IMU_TOPIC;
    fs["image_topic"] >> IMAGE_TOPIC;

    // --- 优化器参数 ---
    FOCAL_LENGTH = 460;
    SOLVER_TIME = fs["max_solver_time"];
    NUM_ITERATIONS = fs["max_num_iterations"];
    MIN_PARALLAX = fs["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    string output_path;
    fs["output_path"] >> output_path;
    VINS_RESULT_PATH = output_path + "/vins_result_no_loop.txt";

    // --- IMU 噪声参数 ---
    ACC_N = fs["acc_n"];
    ACC_W = fs["acc_w"];
    GYR_N = fs["gyr_n"];
    GYR_W = fs["gyr_w"];
    G.z() = fs["g_norm"];

    // --- 图像尺寸 ---
    ROW = fs["image_height"];
    COL = fs["image_width"];

    // --- 外参 ---
    ESTIMATE_EXTRINSIC = fs["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        // 无先验，从零标定
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = output_path + "/extrinsic_parameter.csv";
    }
    else
    {
        if (ESTIMATE_EXTRINSIC == 1)
            EX_CALIB_RESULT_PATH = output_path + "/extrinsic_parameter.csv";
        if (ESTIMATE_EXTRINSIC == 0)
            cout << " fix extrinsic param " << endl;

        cv::Mat cv_R, cv_T;
        fs["extrinsicRotation"] >> cv_R;
        fs["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        RIC.push_back(eigen_R);
        TIC.push_back(eigen_T);
    }

    // --- 初始化参数 ---
    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    // --- 时间同步 ---
    TD = fs["td"];
    ESTIMATE_TD = fs["estimate_td"];
    ROLLING_SHUTTER = fs["rolling_shutter"];
    TR = ROLLING_SHUTTER ? (double)fs["rolling_shutter_tr"] : 0.0;

    // --- 前端参数 ---
    MAX_CNT = fs["max_cnt"];
    MIN_DIST = fs["min_dist"];
    FREQ = fs["freq"];
    F_THRESHOLD = fs["F_threshold"];
    SHOW_TRACK = fs["show_track"];
    EQUALIZE = fs["equalize"];
    FISHEYE = fs["fisheye"];
    CAM_NAMES.push_back(config_file);
    STEREO_TRACK = false;
    PUB_THIS_FRAME = false;
    if (FREQ == 0) FREQ = 10;

    fs.release();

    // --- 打印参数摘要 ---
    cout << "[Parameters] Loaded from: " << config_file << "\n"
         << "  INIT_DEPTH: " << INIT_DEPTH << "\n"
         << "  MIN_PARALLAX: " << MIN_PARALLAX << "\n"
         << "  ACC_N: " << ACC_N << "  ACC_W: " << ACC_W << "\n"
         << "  GYR_N: " << GYR_N << "  GYR_W: " << GYR_W << "\n"
         << "  RIC:\n" << RIC[0] << "\n"
         << "  TIC: " << TIC[0].transpose() << "\n"
         << "  G: " << G.transpose() << "\n"
         << "  SOLVER_TIME: " << SOLVER_TIME << "  NUM_ITERATIONS: " << NUM_ITERATIONS << "\n"
         << "  ESTIMATE_EXTRINSIC: " << ESTIMATE_EXTRINSIC << "\n"
         << "  ESTIMATE_TD: " << ESTIMATE_TD << "  TD: " << TD << "  TR: " << TR << "\n"
         << "  ROW: " << ROW << "  COL: " << COL << "\n"
         << "  FOCAL_LENGTH: " << FOCAL_LENGTH << "\n"
         << "  MAX_CNT: " << MAX_CNT << "  MIN_DIST: " << MIN_DIST << "\n"
         << "  FREQ: " << FREQ << "  SHOW_TRACK: " << SHOW_TRACK << "\n"
         << "  EQUALIZE: " << EQUALIZE << "  FISHEYE: " << FISHEYE << "\n"
         << endl;
}

// Keep backward compatibility alias (declaration is inline in header)
// void readParameters(std::string config_file) is already defined inline in parameters.h
