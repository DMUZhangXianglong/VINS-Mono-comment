/*
 * @Author: DMU zhangxianglong
 * @Date: 2024-07-10 09:04:56
 * @LastEditTime: 2024-07-10 11:20:27
 * @LastEditors: DMU zhangxianglong
 * @FilePath: /VINS-Mono-comment/feature_tracker/src/parameters.h
 * @Description: 
 */
#pragma once
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
// 行数
extern int ROW;
// 列数
extern int COL;
// 焦距
extern int FOCAL_LENGTH;
// 相机个数
const int NUM_OF_CAM = 1; 

extern std::string IMAGE_TOPIC;
extern std::string IMU_TOPIC;
extern std::string FISHEYE_MASK;
// 相机名
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern int WINDOW_SIZE;
extern int FREQ; 
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;

// 读取参数
void readParameters(ros::NodeHandle &n);
