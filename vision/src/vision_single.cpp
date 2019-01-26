//
// Created by ryan on 1/24/19.
//

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8MultiArray.h>
#include <vision/VisionNodeConfig.h>
#include <dynamic_reconfigure/server.h>

#include <opencv2/core.hpp>

#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>

#include <vision/Process.h>

using namespace vision;

bool NEW_FRAME = false;
cv::Mat SOURCE_FRAME = cv::Mat();
bool NEW_CAMERA_INFO = false;
cv::Mat DIST = cv::Mat();
cv::Mat CAMERA_MAT = cv::Mat();
Config CONFIG;

void imageCallback(const sensor_msgs::Image::ConstPtr &msg) {
    ROS_INFO("New source frame");
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        cv_ptr->image.copyTo(SOURCE_FRAME);
        NEW_FRAME = true;
    } catch(cv_bridge::Exception &e) {

    }
}

void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg) {
    cv::Mat dist(msg.get()->D);
    dist.copyTo(DIST);

    double cameraMatrix[9];
    for(int i = 0; i < 9; i++) {
        cameraMatrix[i] = msg.get()->K[i];
    }
    cv::Mat cameraMat(3,3, cv::DataType<double>::type, cameraMatrix);
    cameraMat.copyTo(CAMERA_MAT);

    NEW_CAMERA_INFO = true;
}

void updateConfig(Config& config, VisionNodeConfig dynConfig) {
    config.hsvLow = cv::Scalar(dynConfig.low_h_param,   dynConfig.low_s_param,  dynConfig.low_v_param);
    config.hsvHigh = cv::Scalar(dynConfig.high_h_param, dynConfig.high_s_param, dynConfig.high_v_param);
    config.epsilon = dynConfig.epsilon_param;
}

void reconfigureCallback(vision::VisionNodeConfig& config, uint32_t level) {
    ROS_INFO_STREAM("Reconfigure Request: " <<
                                            "(" << config.low_h_param << "," << config.low_s_param << "," << config.low_v_param << ")" <<
                                            "(" << config.high_h_param << "," << config.high_s_param << "," << config.high_v_param << ")"
    );

    updateConfig(CONFIG, config);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vision");
    ros::NodeHandle n("~");

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    // Dynamic reconfigure setup
    dynamic_reconfigure::Server<vision::VisionNodeConfig> server;
    dynamic_reconfigure::Server<vision::VisionNodeConfig>::CallbackType dynCfgCB;
    dynCfgCB = boost::bind(&reconfigureCallback, _1, _2);
    server.setCallback(dynCfgCB);


    // Pub/Sub Setup
    ros::Subscriber sourceFrameSub = n.subscribe("/camera/image_raw", 5, imageCallback);
    ros::Subscriber cameraInfoSub = n.subscribe("/camera/camera_info", 5, cameraInfoCallback);

    ros::Publisher processedFramePub = n.advertise<sensor_msgs::Image>("processed_image", 10);
    ros::Publisher destFramePub = n.advertise<sensor_msgs::Image>("dest_image", 10);
    ros::Publisher distancePub = n.advertise<std_msgs::Float32>("distance", 10);

    //Odometry Setup
    ros::Publisher odomPub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odomBroadcaster;

    ros::Time currentTime = ros::Time::now();
    ros::Time lastTime = ros::Time::now();

    //Set up config, from ros param
    CONFIG.hsvLow = cv::Scalar(37,30,93);
    CONFIG.hsvHigh = cv::Scalar(92,255,255);
    CONFIG.closeSE = cv::Size(5,5);
    CONFIG.openSE = cv::Size(3,3);
    CONFIG.epsilon = 1;
    CONFIG.blobMax = 10;
    CONFIG.maxDistance = 5.0 * 0.0254;

    Process process(&CONFIG);

    ros::Rate loopRate(50);
    while(ros::ok()) {
        //Get new frame from camera
        if(NEW_FRAME && NEW_CAMERA_INFO) {
            CAMERA_MAT.copyTo(CONFIG.cameraMatrix);
            DIST.copyTo(CONFIG.distCoeffs);

            ROS_DEBUG("Got new frame");
            //Process frame
            cv::Mat destFrame;
            cv::Mat processedFrame;
            std::vector<TargetInfo> results = process.ProcessFrame(SOURCE_FRAME, destFrame, processedFrame);

            //Publish target data
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", destFrame).toImageMsg();
            destFramePub.publish(msg);
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", processedFrame).toImageMsg();
            processedFramePub.publish(msg);

            NEW_FRAME = false;


        } else {
            ROS_DEBUG("No frame available");
        }
        ros::spinOnce();
        loopRate.sleep();
    }

}

