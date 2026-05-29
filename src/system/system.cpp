/**
 * @file System.cpp
 * @brief VIO 系统实现
 *
 * 职责划分:
 *   - 构造/析构: 读取参数, 初始化各模块
 *   - PubImuData: IMU 数据入队
 *   - PubImageData: 前端特征追踪 + 特征入队
 *   - ProcessBackEnd: 后端 IMU 预积分 + 视觉优化
 *   - Draw: 3D 可视化
 */

#include "system/system.h"
#include <pangolin/pangolin.h>

using namespace std;
using namespace cv;
using namespace Eigen;

// ============================================================
//  构造 / 析构
// ============================================================

System::System(const string& config_path)
    : backend_running_(true)
{
    string config_file = config_path + "euroc_config.yaml";
    cout << "[System] Loading config: " << config_file << endl;

    // 读取全局参数 (相机内参、IMU噪声、窗口大小等)
    ReadParameters(config_file);

    // 初始化前端特征追踪器 (读取相机内参用于去畸变)
    trackerData[0].readIntrinsicParameter(config_file);

    // 初始化后端估计器
    estimator_.setParameter();

    // 打开位姿输出文件
    pose_output_file_.open("./pose_output.txt", fstream::out);
    if (!pose_output_file_.is_open())
        cerr << "[System] WARNING: Cannot open pose_output.txt" << endl;

    cout << "[System] Initialization complete." << endl;
}

System::~System()
{
    backend_running_ = false;
    pangolin::QuitAll();

    // 清空缓冲区
    buf_mutex_.lock();
    while (!feature_buf_.empty()) feature_buf_.pop();
    while (!imu_buf_.empty())     imu_buf_.pop();
    buf_mutex_.unlock();

    // 清理估计器状态
    estimator_mutex_.lock();
    estimator_.clearState();
    estimator_mutex_.unlock();

    pose_output_file_.close();
}

// ============================================================
//  数据输入: IMU
// ============================================================

/**
 * @brief 接收一个 IMU 测量, 放入缓冲队列
 *
 * 调用频率: ~200Hz (由数据发布线程驱动)
 * 线程安全: 通过 buf_mutex_ 保护
 */
void System::PubImuData(double timestamp_sec,
                        const Vector3d& gyroscope,
                        const Vector3d& accelerometer)
{
    // 检查时间戳单调递增
    if (timestamp_sec <= last_imu_time_)
    {
        cerr << "[IMU] WARNING: Timestamp disorder!" << endl;
        return;
    }
    last_imu_time_ = timestamp_sec;

    // 构造 IMU 消息并入队
    auto msg = make_shared<ImuMessage>();
    msg->timestamp = timestamp_sec;
    msg->gyroscope = gyroscope;
    msg->accelerometer = accelerometer;

    buf_mutex_.lock();
    imu_buf_.push(msg);
    buf_mutex_.unlock();

    // 通知后端线程: 有新数据到达
    data_ready_cond_.notify_one();
}

// ============================================================
//  数据输入: 图像 (包含前端特征追踪)
// ============================================================

/**
 * @brief 接收一帧图像, 执行前端特征追踪, 将特征放入缓冲队列
 *
 * 前端流程:
 *   1. 跳过第一帧 (无光流速度)
 *   2. 频率控制 (按 FREQ 参数限制发布频率)
 *   3. 调用 FeatureTracker 进行 KLT 光流追踪
 *   4. 打包特征数据为 FeatureMessage
 *   5. 推入 feature_buf_, 通知后端
 *   6. 2D 特征可视化
 */
void System::PubImageData(double timestamp_sec, Mat& image)
{
    // --- Step 1: 跳过第一帧 (没有光流信息) ---
    if (!is_first_feature_)
    {
        is_first_feature_ = true;
        return;
    }

    // --- Step 2: 初始化第一帧时间, 检测时间中断 ---
    if (is_first_image_)
    {
        is_first_image_ = false;
        first_image_time_ = timestamp_sec;
        last_image_time_ = timestamp_sec;
        return;
    }

    // 检测图像流是否中断 (间隔>1s 或时间倒退)
    if (timestamp_sec - last_image_time_ > 1.0 || timestamp_sec < last_image_time_)
    {
        cerr << "[Frontend] Image stream discontinuity! Resetting tracker." << endl;
        is_first_image_ = true;
        last_image_time_ = 0;
        pub_count_ = 1;
        return;
    }
    last_image_time_ = timestamp_sec;

    // --- Step 3: 频率控制 ---
    bool publish_this_frame = false;
    if (round(1.0 * pub_count_ / (timestamp_sec - first_image_time_)) <= FREQ)
    {
        publish_this_frame = true;
        // 定期重置计数器，避免浮点误差累积
        if (abs(1.0 * pub_count_ / (timestamp_sec - first_image_time_) - FREQ) < 0.01 * FREQ)
        {
            first_image_time_ = timestamp_sec;
            pub_count_ = 0;
        }
    }
    // 同步到全局变量 (FeatureTracker 内部依赖此标志决定是否检测新特征)
    PUB_THIS_FRAME = publish_this_frame;

    // --- Step 4: 前端特征追踪 (KLT 光流 + 新特征检测) ---
    trackerData[0].readImage(image, timestamp_sec);

    // 为新检测到的特征分配全局唯一 ID
    for (unsigned int i = 0;; i++)
    {
        bool completed = trackerData[0].updateID(i);
        if (!completed) break;
    }

    // --- Step 5: 打包特征消息并入队 ---
    if (publish_this_frame)
    {
        pub_count_++;

        auto feature_msg = make_shared<FeatureMessage>();
        feature_msg->timestamp = timestamp_sec;

        auto& undistorted_pts = trackerData[0].cur_un_pts;   // 去畸变归一化坐标
        auto& pixel_pts      = trackerData[0].cur_pts;       // 像素坐标
        auto& ids            = trackerData[0].ids;            // 特征 ID
        auto& velocities     = trackerData[0].pts_velocity;  // 像素速度

        for (unsigned int j = 0; j < ids.size(); j++)
        {
            // 只发布追踪超过 1 帧的特征 (有速度信息)
            if (trackerData[0].track_cnt[j] > 1)
            {
                feature_msg->points.push_back(Vector3d(undistorted_pts[j].x, undistorted_pts[j].y, 1.0));
                feature_msg->feature_ids.push_back(ids[j] * NUM_OF_CAM);
                feature_msg->pixel_u.push_back(pixel_pts[j].x);
                feature_msg->pixel_v.push_back(pixel_pts[j].y);
                feature_msg->velocity_x.push_back(velocities[j].x);
                feature_msg->velocity_y.push_back(velocities[j].y);
            }
        }

        // 跳过初始化阶段的第一帧
        if (!init_pub_)
        {
            init_pub_ = true;
        }
        else
        {
            buf_mutex_.lock();
            feature_buf_.push(feature_msg);
            buf_mutex_.unlock();
            data_ready_cond_.notify_one();
        }
    }

    // --- Step 6: 2D 可视化 ---
    visualizeFeatures(image);
}

// ============================================================
//  数据同步: 将 IMU 和图像按时间对齐
// ============================================================

/**
 * @brief 从缓冲区中取出时间对齐的测量对
 *
 * 对每个图像帧, 收集其前后时间段内的所有 IMU 数据,
 * 形成 (IMU序列, 特征消息) 的配对。
 *
 * 时间关系:
 *   IMU[0].t < IMU[1].t < ... < image.t < IMU[n].t
 *   ^--- 这些 IMU 数据用于该图像帧的预积分 ---^
 */
std::vector<System::Measurement> System::getMeasurements()
{
    vector<Measurement> measurements;

    while (true)
    {
        if (imu_buf_.empty() || feature_buf_.empty())
            return measurements;

        // IMU 数据还不够覆盖当前图像时间: 等待更多 IMU
        if (!(imu_buf_.back()->timestamp > feature_buf_.front()->timestamp + estimator_.td))
        {
            wait_count_++;
            return measurements;
        }

        // IMU 起始时间晚于图像时间: 丢弃该图像
        if (!(imu_buf_.front()->timestamp < feature_buf_.front()->timestamp + estimator_.td))
        {
            cerr << "[Sync] Dropping image (no preceding IMU data)" << endl;
            feature_buf_.pop();
            continue;
        }

        // 取出当前图像
        FeatureMessageConstPtr img_msg = feature_buf_.front();
        feature_buf_.pop();

        // 收集该图像时间之前的所有 IMU 数据
        vector<ImuMessageConstPtr> imu_batch;
        while (imu_buf_.front()->timestamp < img_msg->timestamp + estimator_.td)
        {
            imu_batch.push_back(imu_buf_.front());
            imu_buf_.pop();
        }
        // 包含刚好超过图像时间的那个 IMU (用于插值)
        imu_batch.push_back(imu_buf_.front());

        measurements.emplace_back(imu_batch, img_msg);
    }
    return measurements;
}

// ============================================================
//  后端优化线程
// ============================================================

/**
 * @brief 后端主循环: 等待数据 → IMU预积分 → 视觉优化 → 输出位姿
 *
 * 处理流程:
 *   1. 等待条件变量, 获取对齐的 (IMU, 图像) 数据
 *   2. 对每个 IMU 测量调用 processIMU() 进行预积分
 *   3. 将特征观测打包后调用 processImage() 进行优化
 *   4. 如果系统已初始化, 输出当前位姿到文件和轨迹
 */
void System::ProcessBackEnd()
{
    cout << "[Backend] Thread started." << endl;

    while (backend_running_)
    {
        // --- 等待对齐的测量数据 ---
        vector<Measurement> measurements;
        unique_lock<mutex> lock(buf_mutex_);
        data_ready_cond_.wait(lock, [&] {
            return (measurements = getMeasurements()).size() != 0;
        });
        lock.unlock();

        // --- 逐帧处理 ---
        estimator_mutex_.lock();
        for (auto& measurement : measurements)
        {
            auto& imu_batch  = measurement.first;
            auto& feature_msg = measurement.second;

            // ---- IMU 预积分 ----
            // 对两帧图像之间的每个 IMU 采样进行积分
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto& imu : imu_batch)
            {
                double t = imu->timestamp;
                double img_t = feature_msg->timestamp + estimator_.td;

                if (t <= img_t)
                {
                    // IMU 时间在图像之前: 直接积分
                    if (current_time_ < 0) current_time_ = t;
                    double dt = t - current_time_;
                    current_time_ = t;
                    dx = imu->accelerometer.x();
                    dy = imu->accelerometer.y();
                    dz = imu->accelerometer.z();
                    rx = imu->gyroscope.x();
                    ry = imu->gyroscope.y();
                    rz = imu->gyroscope.z();
                    estimator_.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                }
                else
                {
                    // IMU 时间跨过图像时刻: 线性插值到图像时刻
                    double dt_1 = img_t - current_time_;
                    double dt_2 = t - img_t;
                    current_time_ = img_t;
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu->accelerometer.x();
                    dy = w1 * dy + w2 * imu->accelerometer.y();
                    dz = w1 * dz + w2 * imu->accelerometer.z();
                    rx = w1 * rx + w2 * imu->gyroscope.x();
                    ry = w1 * ry + w2 * imu->gyroscope.y();
                    rz = w1 * rz + w2 * imu->gyroscope.z();
                    estimator_.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                }
            }

            // ---- 视觉特征处理 ----
            // 将 FeatureMessage 转换为 estimator 要求的格式:
            //   map<feature_id, vector<(camera_id, [x,y,z,u,v,vx,vy])>>
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image_features;
            for (unsigned int i = 0; i < feature_msg->points.size(); i++)
            {
                int feature_id = feature_msg->feature_ids[i] / NUM_OF_CAM;
                int camera_id  = feature_msg->feature_ids[i] % NUM_OF_CAM;

                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << feature_msg->points[i].x(),
                                   feature_msg->points[i].y(),
                                   feature_msg->points[i].z(),
                                   feature_msg->pixel_u[i],
                                   feature_msg->pixel_v[i],
                                   feature_msg->velocity_x[i],
                                   feature_msg->velocity_y[i];

                image_features[feature_id].emplace_back(camera_id, xyz_uv_velocity);
            }

            // 调用后端优化 (初始化 / 滑动窗口优化)
            TicToc t_process;
            estimator_.processImage(image_features, feature_msg->timestamp);

            // ---- 输出结果 ----
            if (estimator_.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            {
                Vector3d    position = estimator_.Ps[WINDOW_SIZE];
                Quaterniond rotation(estimator_.Rs[WINDOW_SIZE]);
                double stamp = estimator_.Headers[WINDOW_SIZE];

                // 保存轨迹用于 3D 可视化
                trajectory_.push_back(position);

                // 输出到文件 (TUM 格式: t x y z qw qx qy qz)
                pose_output_file_ << fixed << stamp << " "
                    << position.x() << " " << position.y() << " " << position.z() << " "
                    << rotation.w() << " " << rotation.x() << " " << rotation.y() << " " << rotation.z()
                    << endl;

                cout << "[Backend] t=" << fixed << stamp
                     << " pos=[" << position.transpose() << "]"
                     << " cost=" << t_process.toc() << "ms" << endl;
            }
        }
        estimator_mutex_.unlock();
    }
}

// ============================================================
//  2D 特征可视化
// ============================================================

void System::visualizeFeatures(const Mat& image)
{
    Mat display;
    cvtColor(image, display, COLOR_GRAY2RGB);

    if (!SHOW_TRACK) return;

    // 计算 FPS
    double cur_time = getTickCount() / getTickFrequency();
    if (prev_frame_time_ > 0)
        viz_fps_ = 0.9 * viz_fps_ + 0.1 * (1.0 / (cur_time - prev_frame_time_));
    prev_frame_time_ = cur_time;

    // 绘制特征点 (颜色按追踪时长映射)
    for (unsigned int j = 0; j < trackerData[0].cur_pts.size(); j++)
    {
        double ratio = min(1.0, 1.0 * trackerData[0].track_cnt[j] / WINDOW_SIZE);

        // HSV 色彩映射: 红色(新) -> 绿色(长期追踪)
        int hue = (int)(120 * ratio);
        Mat hsv(1, 1, CV_8UC3, Scalar(hue, 255, 255));
        Mat bgr;
        cvtColor(hsv, bgr, COLOR_HSV2BGR);
        Scalar color(bgr.at<Vec3b>(0,0)[0], bgr.at<Vec3b>(0,0)[1], bgr.at<Vec3b>(0,0)[2]);

        int radius = 3 + (int)(3 * ratio);
        circle(display, trackerData[0].cur_pts[j], radius, color, -1, LINE_AA);

        // 长期追踪的特征加白色外圈
        if (trackerData[0].track_cnt[j] > WINDOW_SIZE / 2)
            circle(display, trackerData[0].cur_pts[j], radius + 2, Scalar(255,255,255), 1, LINE_AA);
    }

    // 状态信息叠加
    string status = (estimator_.solver_flag == Estimator::SolverFlag::NON_LINEAR)
        ? "Tracking" : "Initializing";
    string info = status + " | Features: " + to_string(trackerData[0].cur_pts.size())
        + " | FPS: " + to_string((int)viz_fps_);

    rectangle(display, Point(5, 5), Point(400, 30), Scalar(0,0,0), -1);
    putText(display, info, Point(10, 23), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,255,0), 1, LINE_AA);

    imshow("Feature Tracking", display);
    waitKey(1);
}

// ============================================================
//  3D 可视化线程
// ============================================================

/**
 * @brief 绘制单个相机锥体 (frustum)
 */
void System::drawCameraFrustum(const Vector3d& position,
                               const Matrix3d& rotation,
                               bool is_current_frame)
{
    // 锥体大小和相机内参 (近似)
    const float sz = 0.15f;
    const float fx = 461.6f, fy = 460.3f, cx = 363.0f, cy = 248.1f;
    const float w = 752.0f, h = 480.0f;

    // 图像平面四角在相机坐标系下的方向, 乘以 sz 得到 3D 点
    Vector3d p1 = rotation * Vector3d((0-cx)/fx*sz, (0-cy)/fy*sz, sz) + position;
    Vector3d p2 = rotation * Vector3d((w-cx)/fx*sz, (0-cy)/fy*sz, sz) + position;
    Vector3d p3 = rotation * Vector3d((w-cx)/fx*sz, (h-cy)/fy*sz, sz) + position;
    Vector3d p4 = rotation * Vector3d((0-cx)/fx*sz, (h-cy)/fy*sz, sz) + position;

    if (is_current_frame)
        glColor4f(0.0f, 1.0f, 0.0f, 1.0f);   // 当前帧: 绿色
    else
        glColor4f(0.8f, 0.8f, 0.8f, 0.6f);   // 历史帧: 半透明白

    glLineWidth(2);
    glBegin(GL_LINES);
    // 相机中心到四角的连线
    glVertex3d(position[0], position[1], position[2]); glVertex3d(p1[0], p1[1], p1[2]);
    glVertex3d(position[0], position[1], position[2]); glVertex3d(p2[0], p2[1], p2[2]);
    glVertex3d(position[0], position[1], position[2]); glVertex3d(p3[0], p3[1], p3[2]);
    glVertex3d(position[0], position[1], position[2]); glVertex3d(p4[0], p4[1], p4[2]);
    // 四角形成的矩形
    glVertex3d(p1[0], p1[1], p1[2]); glVertex3d(p2[0], p2[1], p2[2]);
    glVertex3d(p2[0], p2[1], p2[2]); glVertex3d(p3[0], p3[1], p3[2]);
    glVertex3d(p3[0], p3[1], p3[2]); glVertex3d(p4[0], p4[1], p4[2]);
    glVertex3d(p4[0], p4[1], p4[2]); glVertex3d(p1[0], p1[1], p1[2]);
    glEnd();
}

/**
 * @brief 3D 可视化主循环
 *
 * 绘制内容:
 *   - 地面网格 (空间参考)
 *   - RGB 坐标轴
 *   - 轨迹 (渐变色)
 *   - 滑动窗口中的相机锥体
 *   - 3D 地图点
 */
void System::Draw()
{
    // 初始化 Pangolin 窗口
    pangolin::CreateWindowAndBind("MiniVIO 3D Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    gl_render_state_ = pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 384, 0.1, 1000),
        pangolin::ModelViewLookAt(-5, 0, 15, 7, 0, 0, 1.0, 0.0, 0.0));

    gl_display_ = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(gl_render_state_));

    while (!pangolin::ShouldQuit())
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        gl_display_.Activate(gl_render_state_);
        glClearColor(0.05f, 0.05f, 0.1f, 1.0f);

        // ---- 地面网格 ----
        glLineWidth(1);
        glColor4f(0.3f, 0.3f, 0.3f, 0.5f);
        glBegin(GL_LINES);
        for (int i = -20; i <= 20; i += 2)
        {
            glVertex3f(i, -20, 0); glVertex3f(i, 20, 0);
            glVertex3f(-20, i, 0); glVertex3f(20, i, 0);
        }
        glEnd();

        // ---- 坐标轴 (RGB = XYZ) ----
        glLineWidth(3);
        glColor3f(1,0.2f,0.2f); glBegin(GL_LINES); glVertex3f(0,0,0); glVertex3f(3,0,0); glEnd();
        glColor3f(0.2f,1,0.2f); glBegin(GL_LINES); glVertex3f(0,0,0); glVertex3f(0,3,0); glEnd();
        glColor3f(0.2f,0.2f,1); glBegin(GL_LINES); glVertex3f(0,0,0); glVertex3f(0,0,3); glEnd();

        // ---- 轨迹 (渐变色: 蓝 → 绿 → 红) ----
        glLineWidth(3);
        glBegin(GL_LINES);
        int n = trajectory_.size();
        for (int i = 0; i < n - 1; ++i)
        {
            float t = (float)i / max(1, n - 1);
            float r = (t < 0.5f) ? 0 : (t - 0.5f) * 2;
            float g = (t < 0.5f) ? t * 2 : 1 - (t - 0.5f) * 2;
            float b = (t < 0.5f) ? 1 - t * 2 : 0;
            glColor3f(r, g, b);
            glVertex3f(trajectory_[i].x(), trajectory_[i].y(), trajectory_[i].z());
            glVertex3f(trajectory_[i+1].x(), trajectory_[i+1].y(), trajectory_[i+1].z());
        }
        glEnd();

        // ---- 相机锥体 + 地图点 ----
        if (estimator_.solver_flag == Estimator::SolverFlag::NON_LINEAR)
        {
            for (int i = 0; i < WINDOW_SIZE + 1; ++i)
            {
                drawCameraFrustum(estimator_.Ps[i], estimator_.Rs[i], i == WINDOW_SIZE);
            }

            // 活跃地图点 (白色)
            glPointSize(3);
            glBegin(GL_POINTS);
            glColor4f(1, 1, 1, 0.8f);
            for (auto& pt : estimator_.point_cloud)
                glVertex3d(pt[0], pt[1], pt[2]);
            glEnd();

            // 边缘化地图点 (灰色)
            glPointSize(2);
            glBegin(GL_POINTS);
            glColor4f(0.5f, 0.5f, 0.6f, 0.5f);
            for (auto& pt : estimator_.margin_cloud)
                glVertex3d(pt[0], pt[1], pt[2]);
            glEnd();
        }

        pangolin::FinishFrame();
        usleep(5000);
    }
}
