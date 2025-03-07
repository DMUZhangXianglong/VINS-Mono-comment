#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include "feature_tracker.h"

#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::ImageConstPtr> img_buf; // 图像指针队列

ros::Publisher pub_img, pub_match; // 图像发布和 匹配发布
ros::Publisher pub_restart;       // 重启发布

FeatureTracker trackerData[NUM_OF_CAM]; // 该类型的数组表示相机个数
double first_image_time; // 第一帧图像时间              
int pub_count = 1; // 发布图像计数器
bool first_image_flag = true; // 是否为第一帧图像标志位
double last_image_time = 0; // 上一帧图像时间戳
bool init_pub = 0; // 初始发布

// 回调函数 传入的类型是图像类型的消息 ROS格式
void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{   
    // 判断是否为第一帧
    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = img_msg->header.stamp.toSec();
        last_image_time = img_msg->header.stamp.toSec();
        return;
    }
    // detect unstable camera stream
    // 检测不稳定的相机数据 如果图像有问题则返回 
    if (img_msg->header.stamp.toSec() - last_image_time > 1.0 || img_msg->header.stamp.toSec() < last_image_time)
    {
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true; 
        last_image_time = 0;
        pub_count = 1;
        
        // 声明 std_msgs::Bool 类型变量并且设置为true
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);
        return;
    }

    // 设置上一帧图像时间戳
    last_image_time = img_msg->header.stamp.toSec();
    
    // frequency control
    // 控制发布频率 默认为10hz
    if (round(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time)) <= FREQ)
    {   
        // 设置发布为true
        PUB_THIS_FRAME = true;
        // reset the frequency control
        // 重置一下频率控制
        if (abs(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = img_msg->header.stamp.toSec();
            pub_count = 0;
        }
    }
    // 直接不发布
    else
        PUB_THIS_FRAME = false;
    
    // 定义一个智能指针 保存ROS msg的图像
    cv_bridge::CvImageConstPtr ptr;
    // 判断编码方式查看图像是否是灰度图像
    if (img_msg->encoding == "8UC1")
    {
        // 保存图像信息
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        // is_bigendian 是 img 的一个属性，用于表示该图像是够为大端
        img.is_bigendian = img_msg->is_bigendian;
        
        // 图像每行字节数
        img.step = img_msg->step;

        // 实际像素数据
        img.data = img_msg->data;

        // 设置图像编码方式为 灰度图像
        img.encoding = "mono8";

        // 将 ROS 消息转为 OpenCV 格式
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    // 如果不是 8UC1 直接转换
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    // 指针里的内容拿出来 放进 OpenCV Mat
    cv::Mat show_img = ptr->image;
    
    // 用于计时 
    TicToc t_r;

    // 根据相机数量遍历，默认是1个 这个循环只会执行1次
    for (int i = 0; i < NUM_OF_CAM; i++)
    {   
        ROS_DEBUG("processing camera %d", i);
        
        // 如果不是一个相机或者不是双目跟
        if (i != 1 || !STEREO_TRACK)
        {
            // 第 i 个相机读取图片
            trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), img_msg->header.stamp.toSec());
        }
        else
        {   
            // 判断是否需要均衡化
            if (EQUALIZE)
            {   
                // 
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
                trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
        }

// 默认为 0 也就是这段代码不编译
#if SHOW_UNDISTORTION
        trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif
    }

    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            // j != 1 或 !STEREO_TRACK。这表示，当 j 不等于 1 或者 STEREO_TRACK 为 false 时
            if (j != 1 || !STEREO_TRACK)
                // 如果 updateID 返回 true 那么 completed = true
                completed |= trackerData[j].updateID(i);
        // 如果 completed 为false则跳出循序
        if (!completed)
            break;
    }

   // 是否发布此帧 
   if (PUB_THIS_FRAME)
   {
        // 发布次数自增
        pub_count++;
        // 声明 ROS 消息格式
        /*
        sensor_msgs::ChannelFloat32 是 ROS（Robot Operating System）中的一个消息类型，
        它在 sensor_msgs 包中定义，用于表示一组浮点数数组。
        ChannelFloat32 通常用于附加到点云消息中，提供每个点的附加信息，例如强度、颜色或其他自定义数据。
        */
        sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_point;
        sensor_msgs::ChannelFloat32 u_of_point;
        sensor_msgs::ChannelFloat32 v_of_point;
        sensor_msgs::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::ChannelFloat32 velocity_y_of_point;

        // 赋值
        feature_points->header = img_msg->header;
        feature_points->header.frame_id = "world";

        // 
        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    geometry_msgs::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    feature_points->points.push_back(p);
                    id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    u_of_point.values.push_back(cur_pts[j].x);
                    v_of_point.values.push_back(cur_pts[j].y);
                    velocity_x_of_point.values.push_back(pts_velocity[j].x);
                    velocity_y_of_point.values.push_back(pts_velocity[j].y);
                }
            }
        }

        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(velocity_y_of_point);
        ROS_DEBUG("publish %f, at %f", feature_points->header.stamp.toSec(), ros::Time::now().toSec());
        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
            pub_img.publish(feature_points);

        if (SHOW_TRACK)
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
            //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat stereo_img = ptr->image;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                    cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                    //draw speed line
                    /*
                    Vector2d tmp_cur_un_pts (trackerData[i].cur_un_pts[j].x, trackerData[i].cur_un_pts[j].y);
                    Vector2d tmp_pts_velocity (trackerData[i].pts_velocity[j].x, trackerData[i].pts_velocity[j].y);
                    Vector3d tmp_prev_un_pts;
                    tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
                    tmp_prev_un_pts.z() = 1;
                    Vector2d tmp_prev_uv;
                    trackerData[i].m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
                    cv::line(tmp_img, trackerData[i].cur_pts[j], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(255 , 0, 0), 1 , 8, 0);
                    */
                    //char name[10];
                    //sprintf(name, "%d", trackerData[i].ids[j]);
                    //cv::putText(tmp_img, name, trackerData[i].cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
                }
            }
            //cv::imshow("vis", stereo_img);
            //cv::waitKey(5);
            pub_match.publish(ptr->toImageMsg());
        }
    }
    ROS_INFO("whole feature tracker processing costs: %f", t_r.toc());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_tracker");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);

    for (int i = 0; i < NUM_OF_CAM; i++)
        // 从参数文件读取相机参数
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);

    if(FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
            {
                ROS_INFO("load mask fail");
                ROS_BREAK();
            }
            else
                ROS_INFO("load mask success");
        }
    }
    // 订阅原始图像数据
    ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);
    
    // 发布跟踪的特征点
    pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000);

    // 发布跟踪特征点图 用于 rviz 显示
    pub_match = n.advertise<sensor_msgs::Image>("feature_img",1000);

    // 发布重置
    pub_restart = n.advertise<std_msgs::Bool>("restart",1000);
    /*
    if (SHOW_TRACK)
        cv::namedWindow("vis", cv::WINDOW_NORMAL);
    */
    ros::spin();
    return 0;
}


// new points velocity is 0, pub or not?
// track cnt > 1 pub?