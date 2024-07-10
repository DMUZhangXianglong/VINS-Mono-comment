/*
 * @Author: DMU zhangxianglong
 * @Date: 2024-07-10 09:04:56
 * @LastEditTime: 2024-07-10 14:37:03
 * @LastEditors: DMU zhangxianglong
 * @FilePath: /VINS-Mono-comment/feature_tracker/src/parameters.cpp
 * @Description: 
 */
#include "parameters.h"
// 图像话题名
std::string IMAGE_TOPIC; 
// IMU话题名
std::string IMU_TOPIC;
// 相机名   
std::vector<std::string> CAM_NAMES; 
// 鱼眼相机掩膜
std::string FISHEYE_MASK; 
// 最大计数
int MAX_CNT; 
// 最小距离
int MIN_DIST; 
// 窗口尺寸
int WINDOW_SIZE; 
// 发布频率
int FREQ; 
// 阈值
double F_THRESHOLD;
// 显示跟踪 
int SHOW_TRACK;
// 双目跟踪 
int STEREO_TRACK; 
// 
int EQUALIZE; 
// 行数
int ROW; 
// 列数
int COL; 
// 焦距
int FOCAL_LENGTH;
// 鱼眼镜头 
int FISHEYE; 
// 发布此帧
bool PUB_THIS_FRAME; 

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    std::string VINS_FOLDER_PATH = readParam<std::string>(n, "vins_folder");

    fsSettings["image_topic"] >> IMAGE_TOPIC;
    fsSettings["imu_topic"] >> IMU_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    EQUALIZE = fsSettings["equalize"];
    FISHEYE = fsSettings["fisheye"];
    if (FISHEYE == 1)
        FISHEYE_MASK = VINS_FOLDER_PATH + "config/fisheye_mask.jpg";
    CAM_NAMES.push_back(config_file);

    WINDOW_SIZE = 20;
    STEREO_TRACK = false;
    FOCAL_LENGTH = 460;
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;

    fsSettings.release();


}
