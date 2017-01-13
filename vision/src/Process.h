//
// Created by ryan on 1/5/17.
//

#ifndef VISION_PROCESS_H
#define VISION_PROCESS_H

#include <opencv2/core.hpp>
#if CV_MAJOR_VERSION != 3
#error "Requires OpenCV 3"
#endif

#include "Target.h"

namespace vision {

    typedef struct {
        cv::Mat distCoeffs;
        cv::Mat cameraMatrix;

        cv::Scalar hsvLow;
        cv::Scalar hsvHigh;
        cv::Size closeSE;//5x5
        cv::Size openSE; //2x2
        double epsilon; //5

        int blobMax; //10
        double maxDistance;
    } Config;

    typedef struct {
        double yaw;
        double pitch;
        double roll;
    } Rotation;

    typedef struct {
        int polyIndex;
        TargetID targetID;
        double compositeScore;
        double distance;

        cv::Point3f pose;
        Rotation rotation;
    } TargetInfo;

    class Process {
    public:
        Process(Config *config);
        std::vector<TargetInfo> ProcessFrame(cv::Mat source, cv::Mat &dest);

        static void RotationMatrixToYPR(cv::Mat &rotation,
                                        double &yaw, double &pitch, double &roll);
    private:
        Config *m_config;

        double ratioToScore(double ratio);
        double scoreRectangularity(double contourArea, cv::RotatedRect &boundingBox);
        double scoreAspectRatio(cv::RotatedRect &boundingBox, Target target);

        bool nearBorder(std::vector<cv::Point> points, cv::Size imageSize, int borderSize);
    };
}



#endif //VISION_PROCESS_H
