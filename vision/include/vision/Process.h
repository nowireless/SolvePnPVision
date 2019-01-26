//
// Created by ryan on 1/5/17.
//

#ifndef VISION_PROCESS_H
#define VISION_PROCESS_H

#include <Eigen/Eigen>
#include <opencv2/core.hpp>
#if CV_MAJOR_VERSION != 3
#error "Requires OpenCV 3"
#endif

#include "Target.h"

namespace vision {

    typedef std::vector<cv::Point> Contour;

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

    struct TargetInfo {
        int polyIndex;
        TargetID targetID;
        double compositeScore;
        cv::RotatedRect boundingBox;

        TargetInfo() : targetID(TargetID::kNotATarget){}
    };

    typedef struct {
        TargetInfo left;
        TargetInfo right;

        double distance;
        cv::Point3f pose;
        Rotation rotation;
    } TargetPair;

    class Process {
    public:
        Process(Config *config);
        std::vector<TargetPair> ProcessFrame(cv::Mat source, cv::Mat &dest, cv::Mat &processed);

        static void RotationMatrixToYPR(cv::Mat &rotation,
                                        double &yaw, double &pitch, double &roll);
    private:
        Config *m_config;

        std::vector<Contour> featureExtraction(cv::Mat source, cv::Mat &dest, cv::Mat &processed);
        std::vector<TargetInfo> targetIdentification(std::vector<Contour> polyContours, cv::Mat &dest, cv::Mat &processed);
        std::vector<TargetPair> findTargetPairs(std::vector<TargetInfo> foundTargets, cv::Mat &dest);
        std::vector<TargetPair> dataExtraction(std::vector<Contour> polyContours, std::vector<TargetPair> targetPairs, cv::Mat &dest);

        // TODO all of the following functions can be made status

        double ratioToScore(double ratio);
        double scoreRectangularity(double contourArea, cv::RotatedRect &boundingBox);
        double scoreAspectRatio(cv::RotatedRect &boundingBox, Target target);

        bool nearBorder(std::vector<cv::Point> points, cv::Size imageSize, int borderSize);

        double normalizeAngle(const double value, const double start, const double end);

        double targetAngle(cv::RotatedRect box);

        void drawBoxLine(cv::Mat image, cv::Point center, double length, double angle, cv::Scalar color);

        double distanceScore(TargetInfo left, TargetInfo right);
        double pointToPointDistance(cv::Point a, cv::Point b);
        cv::Point getMidPoint(cv::Point a, cv::Point b);
        double longSide(cv::Size size);
        double shortSide(cv::Size size);
        Eigen::Vector3f transform(Eigen::Vector3f point, double h, double k, double theta);
    };
}



#endif //VISION_PROCESS_H
