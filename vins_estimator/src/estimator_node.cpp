/*
 * @Author: DMU zhangxianglong
 * @Date: 2024-07-09 22:09:51
 * @LastEditTime: 2024-07-22 16:00:45
 * @LastEditors: DMU zhangxianglong
 * @FilePath: /VINS-Mono-comment/vins_estimator/src/estimator_node.cpp
 * @Description: 
 */

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"

// 初始化估计器对象
Estimator estimator;

// 用于线程同步
std::condition_variable con;
// 当前时间
double current_time = -1;
// imu 缓存
queue<sensor_msgs::ImuConstPtr> imu_buf;
// 特征 缓存
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
//
queue<sensor_msgs::PointCloudConstPtr> relo_buf;

// 
int sum_of_wait = 0;

// 线程锁
std::mutex m_buf;
// 状态
std::mutex m_state;
std::mutex i_buf;
std::mutex m_estimator;

double latest_time;
// 中间量 P Q V
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
// 加速度计偏置
Eigen::Vector3d tmp_Ba;
// 陀螺仪偏置
Eigen::Vector3d tmp_Bg;
// 初始加速度和角速度
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
// 特征初始化
bool init_feature = 0;
bool init_imu = 1;
// 上一帧 imu 时间
double last_imu_t = 0;


// 从IMU测量值imu_msg和上一个PVQ递推得到下一个tmp_Q，tmp_P，tmp_V，中值积分
// 这部分结果只用于 rviz 显示
void predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    // 是否为第一个 imu 数据
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    
    // 时间迭代赋值
    double dt = t - latest_time;
    latest_time = t;

    // x y z 方向上的线性加速度
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    // 组装成 3维 向量
    Eigen::Vector3d linear_acceleration{dx, dy, dz};
    
    // 3 个角速度
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    // 组装向量
    Eigen::Vector3d angular_velocity{rx, ry, rz};
    

    // 为 IMU 中值积分做准备
    // 0 时刻加速度 世界坐标系下 去除偏置
    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;
    // 角速度平均值 去掉了偏置
    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    
    // 旋转的递推
    // Q_t+1 = Q_t*(w * dt)
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    // 1 时刻的加速度 变换到世界坐标系下 然后减去重力
    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

    // 加速度平均值
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    // 位移递推
    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;

    // 速度递推
    tmp_V = tmp_V + dt * un_acc;
    
    // 下次迭代
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator.Ps[WINDOW_SIZE];
    tmp_Q = estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());

}

// 获取IMU测量 和特征点 将测量组装成向量形式
std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> getMeasurements()
{   
    //vector[pair<vector[imu], feature>]
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
    while (true)
    {
        // IMU 和 feature 有一个是空 就返回
        if (imu_buf.empty() || feature_buf.empty())
            return measurements;
        
        // 如果 IMU 缓冲区中最后一个数据的时间戳是否大于特征缓冲区中第一个数据的时间戳加上一个偏移量
        // 如果大于那说明有够多的IMU数据，说明IMU数据包住了图像 那么不进入判断
        // back元素是最新的 front元素是最旧的
        if (!(imu_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            //ROS_WARN("wait for imu, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }
        // 如过IMU的第一个元素的时间还小于特征队列第一个元素时间 还是说明IMU数据包住的图像 那么不进入判断
        // 如果进入循环，说明第队首的帧图像太旧了，然后得移除
        if (!(imu_buf.front()->header.stamp.toSec() < feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            // 移除队首元素
            feature_buf.pop();
            // 停止本次循环 直接进行下一次
            continue;
        }

        // 取出第一个图像元素 并且移除
        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
        feature_buf.pop();
        
        // 取在图像之前的imu数据 这里存放了很多个IMU数据 所以命名为IMUs
        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        while (imu_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td)
        {   
            // emplace_back 是 C++11 引入的一个 STL 容器方法，用于在容器末尾直接构造元素。
            // 与 push_back 不同，emplace_back 允许在容器中原地构造元素，从而避免了不必要的临时对象创建和复制，通常能提高性能
            // 实际上是把这一阵图像 img_msg 对应的所以满足条件的IMU数据放进去
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        //这里把下一个imu_msg也放进去了,但没有pop，因此当前图像帧和下一图像帧会共用这个imu_msg
        IMUs.emplace_back(imu_buf.front());
        
        if (IMUs.empty())
            ROS_WARN("no imu between two image");
        // 组装成vector[pair<vector[imu], feature>]的形式
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{   
    // 判断时间戳
    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder!");
        return;
    }
    // 当前时间戳给了上一时刻
    last_imu_t = imu_msg->header.stamp.toSec();

    // 线程锁 将imu加入缓存队列
    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();
    
    last_imu_t = imu_msg->header.stamp.toSec();
    // 在这个作用域里互斥锁，避免了数据竞争
    {   
        std::lock_guard<std::mutex> lg(m_state);
        //从IMU测量值imu_msg和上一个PVQ递推得到下一个tmp_Q，tmp_P，tmp_V，中值积分
        predict(imu_msg);
        std_msgs::Header header = imu_msg->header;
        header.frame_id = "world";
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            // 发布 IMU 里程计
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
    }
}

// 特征回调函数
void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{   
    // 如果没初始化
    if (!init_feature)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }
    // 成员变量线程锁
    m_buf.lock();
    // 把特征加入队列尾部
    feature_buf.push(feature_msg);
    m_buf.unlock();
    // 确保了队列的线程安全和消息的同步处理
    con.notify_one();
}

// 判断是否需要重置
void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{   
    // 如果需要重置 
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        m_buf.lock();
        
        // 情况缓存队列里的内容
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();

        // 清空系统状态 重置系统状态
        m_estimator.lock();
        estimator.clearState();
        estimator.setParameter();
        m_estimator.unlock();
        current_time = -1;
        last_imu_t = 0;
    }
    return;
}

void relocalization_callback(const sensor_msgs::PointCloudConstPtr &points_msg)
{
    //printf("relocalization callback! \n");
    m_buf.lock();
    relo_buf.push(points_msg);
    m_buf.unlock();
}

// thread: visual-inertial odometry
// 视觉-惯性里程计
void process()
{   // 一直循环
    while (true)
    {
        std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
        
        // 创建一个名为 lk 的 std::unique_lock 对象，并立即锁定 m_buf 互斥体。
        // 这种锁定是独占的，意味着其他线程在 lk 存在期间无法锁定 m_buf。
        std::unique_lock<std::mutex> lk(m_buf);
        /*
        con 是一个 std::condition_variable 对象。
        con.wait(lk, ...) 会将当前线程置于等待状态，直到满足特定条件。
        第二个参数是一个 lambda 表达式，用于定义等待的条件。在这个例子中，条件是 (measurements = getMeasurements()).size() != 0。
        getMeasurements() 函数被调用，其返回值赋给 measurements。如果 measurements 的大小不为零，条件满足，线程将被唤醒。
        如果条件不满足，线程会释放互斥体 m_buf 并进入等待状态。一旦条件变量 con 被其他线程通知，
        当前线程将重新锁定互斥体 m_buf 并再次检查条件。
        */

        //等待上面两个接收数据完成就会被唤醒
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements()).size() != 0;
                 });
        lk.unlock();

        m_estimator.lock();
        
        // 遍历测量中的每个元素，
        for (auto &measurement : measurements)
        {   
            // 取出图像特征点
            auto img_msg = measurement.second;
            
            // 定义线速度 和 角速度
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            
            // 遍历每一帧IMU数据
            for (auto &imu_msg : measurement.first)
            {   
                // 这一帧imu的时间
                double t = imu_msg->header.stamp.toSec();
                // 这一组imu对应的图像的时间
                double img_t = img_msg->header.stamp.toSec() + estimator.td;


                // imu时间 < 图像的时间
                if (t <= img_t)
                {   
                    // current_time的初始值=-1
                    if (current_time < 0)
                    {
                        current_time = t;
                    }

                    double dt = t - current_time;
                    // 断言用于调试 定位出问题的位置
                    ROS_ASSERT(dt >= 0);
                    current_time = t;
                    
                    // 取出IMU中的数据
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    
                    // dt 是这一组IMU中两帧之间的时间差 dx ... 是IMU的读数
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);

                }
                else
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }
            
            // set relocalization frame
            sensor_msgs::PointCloudConstPtr relo_msg = NULL;
            while (!relo_buf.empty())
            {
                relo_msg = relo_buf.front();
                relo_buf.pop();
            }
            if (relo_msg != NULL)
            {
                vector<Vector3d> match_points;
                double frame_stamp = relo_msg->header.stamp.toSec();
                for (unsigned int i = 0; i < relo_msg->points.size(); i++)
                {
                    Vector3d u_v_id;
                    u_v_id.x() = relo_msg->points[i].x;
                    u_v_id.y() = relo_msg->points[i].y;
                    u_v_id.z() = relo_msg->points[i].z;
                    match_points.push_back(u_v_id);
                }
                Vector3d relo_t(relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], relo_msg->channels[0].values[2]);
                Quaterniond relo_q(relo_msg->channels[0].values[3], relo_msg->channels[0].values[4], relo_msg->channels[0].values[5], relo_msg->channels[0].values[6]);
                Matrix3d relo_r = relo_q.toRotationMatrix();
                int frame_index;
                frame_index = relo_msg->channels[0].values[7];
                estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);
            }

            ROS_DEBUG("processing vision data with stamp %f \n", img_msg->header.stamp.toSec());

            TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->channels[0].values[i] + 0.5;
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;
                double x = img_msg->points[i].x;
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                double p_u = img_msg->channels[1].values[i];
                double p_v = img_msg->channels[2].values[i];
                double velocity_x = img_msg->channels[3].values[i];
                double velocity_y = img_msg->channels[4].values[i];
                ROS_ASSERT(z == 1);
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
            }
            estimator.processImage(image, img_msg->header);

            double whole_t = t_s.toc();
            printStatistics(estimator, whole_t);
            std_msgs::Header header = img_msg->header;
            header.frame_id = "world";

            pubOdometry(estimator, header);
            pubKeyPoses(estimator, header);
            pubCameraPose(estimator, header);
            pubPointCloud(estimator, header);
            pubTF(estimator, header);
            pubKeyframe(estimator);
            if (relo_msg != NULL)
                pubRelocalization(estimator);
            //ROS_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(), ros::Time::now().toSec());
        }
        
        m_estimator.unlock();
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}

int main(int argc, char **argv)
{
    /*
    订阅IMU话题
    订阅特征
    订阅是否重置
    订阅位姿图，用于重定位
    */
    
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);
    estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif
    ROS_WARN("waiting for image and imu...");

    registerPub(n);

    // 订阅 IMU 数据
    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    
    // 订阅特征点数据
    ros::Subscriber sub_image = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    
    // 订阅重置
    ros::Subscriber sub_restart = n.subscribe("/feature_tracker/restart", 2000, restart_callback);
    // 订阅重定位
    ros::Subscriber sub_relo_points = n.subscribe("/pose_graph/match_points", 2000, relocalization_callback);

    // 启动一个线程
    std::thread measurement_process{process};

    // ros::spin(); 是 ROS（机器人操作系统）中的一个函数，用于在节点中进入一个事件处理循环。
    // 这个函数会阻塞当前线程，处理所有来自 ROS 的回调函数，直到节点关闭
    ros::spin();

    return 0;
}
