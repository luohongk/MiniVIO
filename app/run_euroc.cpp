/**
 * @file run_euroc.cpp
 * @brief EuRoC 数据集离线运行入口
 *
 * 本程序模拟传感器数据发布，启动 4 个线程协同工作：
 *   1. IMU 数据发布线程   —— 逐行读取 IMU 数据并喂给系统
 *   2. 图像数据发布线程   —— 逐帧读取图像并喂给系统
 *   3. 后端优化线程       —— VIO 状态估计（滑动窗口优化）
 *   4. 可视化线程         —— Pangolin 3D 轨迹显示
 *
 * 用法: ./run_euroc <数据集路径> <配置文件目录>
 * 示例: ./run_euroc /home/data/EuRoC/V2_02_medium/mav0/ ../config/
 */

#include <unistd.h>
#include <iostream>
#include <thread>
#include <iomanip>
#include <sstream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include "system/system.h"

using namespace std;
using namespace cv;
using namespace Eigen;

// ============================================================
//  全局配置
// ============================================================

/// 数据发布的时间缩放因子 (越大运行越慢，方便调试)
const int kReplaySpeedFactor = 2;

/// 路径配置 (从命令行参数和 YAML 配置文件中读取)
struct DatasetPaths {
    string data_dir;           // 数据集根目录, 如 .../mav0/
    string config_dir;         // 配置文件目录, 如 ../config/
    string imu_filename;       // IMU 数据文件名 (相对于 config_dir)
    string image_list_filename;// 图像列表文件名 (相对于 config_dir)
    string image_subdir;       // 图像存放子目录 (相对于 data_dir)
} g_paths;

/// VIO 系统实例
std::shared_ptr<System> g_system;

// ============================================================
//  配置读取
// ============================================================

/**
 * @brief 从 YAML 配置文件中读取数据集相关路径
 * @param config_file 完整的 YAML 文件路径
 *
 * YAML 中对应的字段:
 *   imu_data_file:   "V101_imu0.txt"
 *   image_data_file: "V101_cam0.txt"
 *   image_subdir:    "cam0/data/"
 */
void loadDatasetPaths(const string& config_file)
{
    cv::FileStorage fs(config_file, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        cerr << "[ERROR] Cannot open config: " << config_file << endl;
        exit(-1);
    }

    fs["imu_data_file"]   >> g_paths.imu_filename;
    fs["image_data_file"] >> g_paths.image_list_filename;
    fs["image_subdir"]    >> g_paths.image_subdir;
    fs.release();

    // 使用默认值 (如果 YAML 中未配置)
    if (g_paths.imu_filename.empty())        g_paths.imu_filename = "imu0.txt";
    if (g_paths.image_list_filename.empty()) g_paths.image_list_filename = "cam0.txt";
    if (g_paths.image_subdir.empty())        g_paths.image_subdir = "cam0/data/";

    // 确保子目录以 '/' 结尾
    if (g_paths.image_subdir.back() != '/')
        g_paths.image_subdir += '/';

    cout << "[Config] imu_data_file:   " << g_paths.imu_filename << endl;
    cout << "[Config] image_data_file: " << g_paths.image_list_filename << endl;
    cout << "[Config] image_subdir:    " << g_paths.image_subdir << endl;
}

// ============================================================
//  数据发布线程
// ============================================================

/**
 * @brief IMU 数据发布线程
 *
 * 逐行读取 IMU 文件, 每行格式:
 *   timestamp(ns) gyr_x gyr_y gyr_z acc_x acc_y acc_z
 *
 * 以 5ms × kReplaySpeedFactor 的间隔模拟 IMU 实时输入
 */
void publishImuData()
{
    const string imu_file = g_paths.config_dir + g_paths.imu_filename;
    cout << "[IMU] Reading: " << imu_file << endl;

    ifstream ifs(imu_file);
    if (!ifs.is_open())
    {
        cerr << "[IMU] ERROR: Cannot open " << imu_file << endl;
        return;
    }

    string line;
    while (getline(ifs, line) && !line.empty())
    {
        istringstream iss(line);
        double timestamp_ns;
        Vector3d gyroscope, accelerometer;

        iss >> timestamp_ns
            >> gyroscope.x() >> gyroscope.y() >> gyroscope.z()
            >> accelerometer.x() >> accelerometer.y() >> accelerometer.z();

        // 时间戳从纳秒转为秒后送入系统
        double timestamp_sec = timestamp_ns / 1e9;
        g_system->PubImuData(timestamp_sec, gyroscope, accelerometer);

        // 模拟 200Hz IMU 频率
        usleep(5000 * kReplaySpeedFactor);
    }

    ifs.close();
    cout << "[IMU] Finished." << endl;
}

/**
 * @brief 图像数据发布线程
 *
 * 逐行读取图像列表文件, 每行格式:
 *   timestamp(ns) image_filename
 *
 * 以 50ms × kReplaySpeedFactor 的间隔模拟 20Hz 相机输入
 */
void publishImageData()
{
    const string image_list_file = g_paths.config_dir + g_paths.image_list_filename;
    cout << "[Image] Reading: " << image_list_file << endl;

    ifstream ifs(image_list_file);
    if (!ifs.is_open())
    {
        cerr << "[Image] ERROR: Cannot open " << image_list_file << endl;
        return;
    }

    string line;
    while (getline(ifs, line) && !line.empty())
    {
        istringstream iss(line);
        double timestamp_ns;
        string image_filename;

        iss >> timestamp_ns >> image_filename;

        // 拼接完整图像路径: data_dir + image_subdir + filename
        string image_path = g_paths.data_dir + g_paths.image_subdir + image_filename;

        // 以灰度模式读取图像
        Mat image = imread(image_path, IMREAD_GRAYSCALE);
        if (image.empty())
        {
            cerr << "[Image] ERROR: Cannot read " << image_path << endl;
            return;
        }

        // 时间戳从纳秒转为秒后送入系统
        double timestamp_sec = timestamp_ns / 1e9;
        g_system->PubImageData(timestamp_sec, image);

        // 模拟 20Hz 相机频率
        usleep(50000 * kReplaySpeedFactor);
    }

    ifs.close();
    cout << "[Image] Finished." << endl;
}

// ============================================================
//  主函数
// ============================================================

int main(int argc, char** argv)
{
    // --- 1. 解析命令行参数 ---
    if (argc != 3)
    {
        cerr << "Usage: ./run_euroc <dataset_path> <config_path>" << endl;
        cerr << "Example: ./run_euroc /home/data/EuRoC/V2_02_medium/mav0/ ../config/" << endl;
        return -1;
    }

    g_paths.data_dir   = argv[1];
    g_paths.config_dir = argv[2];

    // 确保路径以 '/' 结尾
    if (g_paths.data_dir.back() != '/')   g_paths.data_dir += '/';
    if (g_paths.config_dir.back() != '/') g_paths.config_dir += '/';

    // --- 2. 读取配置文件 ---
    string config_yaml = g_paths.config_dir + "euroc_config.yaml";
    loadDatasetPaths(config_yaml);

    // --- 3. 初始化 VIO 系统 ---
    g_system = make_shared<System>(g_paths.config_dir);

    // --- 4. 启动各线程 ---
    // 后端优化线程 (滑动窗口 + 边缘化)
    thread thread_backend(&System::ProcessBackEnd, g_system);

    // 传感器数据发布线程
    thread thread_imu(publishImuData);
    thread thread_image(publishImageData);

    // 可视化线程
    thread thread_visualization(&System::Draw, g_system);

    // --- 5. 等待数据发布完毕 ---
    thread_imu.join();
    thread_image.join();
    thread_visualization.join();

    cout << "[Main] Done. Goodbye!" << endl;
    return 0;
}
