//
// Created by ryan on 1/5/17.
//
#include <math.h>
#include <algorithm>

#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include "Process.h"
#include "Target.h"

using namespace cv;
using namespace std;

typedef vector<Point> Contour;

namespace vision {

    /**
     *
     * @param config
     * @return
     */
    Process::Process(Config *config) {
        m_config = config;
    }

    /**
     *
     * @param source
     * @param info
     * @return
     */
    std::vector<TargetInfo> Process::ProcessFrame(Mat source, Mat &dest) {
        /*
         * Image Processing
         * ================
         * Threshold
         * Blur
         * Remove small objects
         * Reconstruction/fix blobs
         * Find Contours of blobs
         */

        Mat frame;
        source.copyTo(frame);
        source.copyTo(dest);

        //Convert to HSV color space
        ROS_DEBUG("Converting to HSV");
        cvtColor(frame, frame, CV_BGR2HSV);

        //Threshold, find pixels in range
        ROS_DEBUG_STREAM("Thresholding image to: " << m_config->hsvLow << m_config->hsvHigh);
        inRange(frame, m_config->hsvLow, m_config->hsvHigh, frame);

        /*
         * Morphology
         * First a closing operation is preformed to help group blobs that are close together
         * into a single blob
         * Then a opening operation is preformed to help remove isolated noisy areas.
         * https://stackoverflow.com/questions/30369031/
         * remove-spurious-small-islands-of-noise-in-an-image-python-opencv
         */
        Mat closeSE = getStructuringElement(MORPH_RECT, Size(5,5));
        morphologyEx(frame, frame, MORPH_CLOSE, closeSE);
        Mat openSE = getStructuringElement(MORPH_RECT, Size(2,2));
        morphologyEx(frame, frame, MORPH_OPEN, openSE);

        frame.copyTo(dest);
        cvtColor(dest,dest, CV_GRAY2BGR);

        //Find contours
        vector<Contour> contours;
        vector<Vec4i> hierarchy;
        findContours(frame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

        //Find Polygon contours
        vector<Contour> polyContours;
        for(int i = 0; i < contours.size(); i++) {
            Contour polyContour;
            //NOTE the true at the end to signal that the resulting contour is closed
            approxPolyDP(contours[i], polyContour, m_config->epsilon, true);

            //If a point is near the border of the image ignore the contour,
            //as there may be issues with a partial target being detected.
            //This could cause bad pose estimation.
            if(nearBorder(polyContour, source.size(), 10)) {
                continue;
            }

            //Check to see if the contour has a valid number of sides, if so add it
            int sides = polyContour.size();
            if(sides == 4) {
                //Rectangle
                polyContours.push_back(polyContour);
            } else {

            }
        }


        //Sort Contours from largest to smallest
        sort(polyContours.begin(), polyContours.end(),
             [](Contour a, Contour b) { return contourArea(a) > contourArea(b); }
        );
        ROS_DEBUG("Found blobs %lu", polyContours.size());

        /*
         * Target Identification
         * ==============
         * Check each blob against each possible target, and
         * score each blob accordingly. And select the best
         * target for data extraction
         */
        vector<TargetInfo> foundTargets;
        for(int i = 0; i < polyContours.size() && i < m_config->blobMax; i++) {
            ROS_DEBUG("Blob %i", i);
            int size = polyContours[i].size();
            double area = contourArea(polyContours[i]);
            RotatedRect boundingBox = minAreaRect(polyContours[i]);

            double rectScore = scoreRectangularity(area, boundingBox);
            double aspectRatioScore = scoreAspectRatio(boundingBox, Target::kHORIZONTAL_TARGET);

            ROS_DEBUG("Blob %i Score: Rect %f AR %f", i, rectScore, aspectRatioScore);
            if(rectScore >= 50 && aspectRatioScore >= 20) {
                TargetInfo info;
                info.polyIndex = i;

                if(boundingBox.size.height > boundingBox.size.width) {
                    info.targetID = TargetID::kVertical;
                } else {
                    info.targetID = TargetID::kHorizontal;
                }
                info.compositeScore = (rectScore * 0.5) + (aspectRatioScore * 0.5);
                foundTargets.push_back(info);
                drawContours(dest, polyContours, i, Scalar(0,0,255), 2);
            }
        }

        ROS_DEBUG("Found targets %lu", foundTargets.size());

        /*
         * Data extraction
         * ===============
         * Get target position relative to robot.
         */

        vector<TargetInfo> results;

        //If no targets are found, no point in proceeding
        if(foundTargets.size() == 0) return results;

        //Consider the largest traget
        Mat rvec(3,1, DataType<double>::type);
        Mat tvec(3,1, DataType<double>::type);
        for(int i = 0; i < foundTargets.size(); i++) {
            TargetInfo info = foundTargets[i];
            vector<Point3f> objectPoints;
            //X - forward   - blue
            //Y - left      - green
            //Z - up        - red

            switch(info.targetID) {
            case TargetID::kHorizontal:
                objectPoints.push_back(Point3f(0,-6,-2));//Bottom Left
                objectPoints.push_back(Point3f(0, 6,-2));//Bottom Right
                objectPoints.push_back(Point3f(0, 6, 2));//Top Right
                objectPoints.push_back(Point3f(0,-6, 2));//Top Left
                break;
            case TargetID::kVertical:
                objectPoints.push_back(Point3f(0,-2,-6));//Bottom Left
                objectPoints.push_back(Point3f(0, 2,-6));//Bottom Right
                objectPoints.push_back(Point3f(0, 2, 6));//Top Right
                objectPoints.push_back(Point3f(0,-2, 6));//Top Left
                break;
            default:
                ROS_WARN("Can not handle TargetID %i", info.targetID);
                continue;
            }

            //Find Extreme Corners of target, this is done to pair the object and image points
            //such that the corners align.

            //Calculate the read formed by each point. The point with the largest area is
            //the the bottom right corner, and the point with the least area is top left
            //corner.
            double area[4];
            for(int i = 0; i < 4; i++) {
                area[i] = polyContours[info.polyIndex][i].x * polyContours[info.polyIndex][i].y;
            }

            int maxAreaPos = 0;
            int minAreaPos = 0;
            for(int i = 0; i < 4; i++) {
                if(area[maxAreaPos] < area[i]) {
                    maxAreaPos = i;
                }
                if(area[minAreaPos] > area[i]) {
                    minAreaPos = i;
                }
            }

            Point2f extremeBR = polyContours[info.polyIndex][maxAreaPos];
            Point2f extremeTL = polyContours[info.polyIndex][minAreaPos];

            //The top right and bottom left corners remain, their indexs need to be found.
            int remainingCorners[2];
            int j = 0;
            for(int i = 0; i < 4; i++) {
                if(i == maxAreaPos || i == minAreaPos) continue;
                remainingCorners[j++] = i;
            }

            Point2f extremeBL;
            Point2f extremeTR;

            //The bottom left corner will always be left of the top right corner
            if(polyContours[info.polyIndex][remainingCorners[0]].x < polyContours[info.polyIndex][remainingCorners[1]].x) {
                extremeBL = polyContours[info.polyIndex][remainingCorners[0]];
                extremeTR = polyContours[info.polyIndex][remainingCorners[1]];
            } else {
                extremeBL = polyContours[info.polyIndex][remainingCorners[1]];
                extremeTR = polyContours[info.polyIndex][remainingCorners[0]];
            }

            vector<Point2f> imagePoints;
            imagePoints.push_back(extremeBR);
            imagePoints.push_back(extremeBL);
            imagePoints.push_back(extremeTL);
            imagePoints.push_back(extremeTR);

            //NOTE: SolvePnPRansac makes things worse
            solvePnP(objectPoints, imagePoints, m_config->cameraMatrix, m_config->distCoeffs, rvec, tvec);

            //Need to adjust the translation vector from solvePnP to achieve correct results
            //https://stackoverflow.com/questions/18637494/camera-position-in-world-coordinate-from-cvsolvepnp
            Mat rotationMatrix;
            Rodrigues(rvec, rotationMatrix);
            rotationMatrix = rotationMatrix.t();
            Mat tvecNew = -rotationMatrix * tvec;

            double x, y, z;

            x = tvecNew.at<double>(0,0);
            y = tvecNew.at<double>(1,0);
            z = tvecNew.at<double>(2,0); //In the future this may not be needed
            double distance = sqrt((x*x) + (y*y) + (z*z));
            ROS_DEBUG("Distance %f", distance);

            if(distance > m_config->maxDistance) {
                ROS_WARN("Target too far way, ignoring");
                continue;
            }

            //Not sure if this function is correct
            double yaw, pitch, roll;
            RotationMatrixToYPR(rotationMatrix, yaw, pitch, roll);

            info.pose.x = x;
            info.pose.y = y;
            info.pose.z = z;
            info.rotation.yaw   = yaw;
            info.rotation.pitch = pitch;
            info.rotation.roll  = roll;
            results.push_back(info);

            //Project target coordinate axis on to image
            vector<Point3d> axis;
            //X, Y, Z
            axis.push_back(Point3d(0, 0, 0));   //Orgin
            axis.push_back(Point3d(12, 0, 0));  //X
            axis.push_back(Point3d(0, 12, 0));  //Y
            axis.push_back(Point3d(0, 0, 12));  //Z

            vector<Point2d> drawp;
            projectPoints(axis, rvec, tvec, m_config->cameraMatrix, m_config->distCoeffs, drawp);

            line(dest, drawp[0], drawp[1], Scalar(255, 0, 0), 4); //X-Blue
            line(dest, drawp[0], drawp[2], Scalar(0, 255, 0), 4); //Y-Green
            line(dest, drawp[0], drawp[3], Scalar(0, 0, 255), 4); //Z-Red
        }
        return results;
    }

    bool Process::nearBorder(std::vector<cv::Point> points, cv::Size imageSize, int borderSize) {
        for(int i = 0; i < points.size(); i++) {
            Point point = points[i];
            if(point.x <= borderSize) return true;
            if(point.y <= borderSize) return true;
            if((imageSize.width - point.x) <= borderSize) return true;
            if((imageSize.height - point.y) <= borderSize) return true;
        }
        return false;
    }

    void Process::RotationMatrixToYPR(cv::Mat &rotation,
                                      double &yaw, double &pitch, double &roll) {
        yaw = atan(rotation.at<double>(1,0)/rotation.at<double>(0,0));
        pitch = atan(-rotation.at<double>(2,0)/sqrt(pow(rotation.at<double>(2,1),2) + pow(rotation.at<double>(2,2),2)));
        yaw = atan(rotation.at<double>(2,1)/rotation.at<double>(2,2));
    }

    /**
     *
     * @param ratio
     * @return Score between 0-100
     */
    double Process::ratioToScore(double ratio) {
        return max(0.0, min(100.0*(1.0 - fabs(1.0 - ratio)), 100.0));
    }

    /**
     *
     * @param contourArea
     * @param boundingBox
     * @return Score between 0-100
     */
    double Process::scoreRectangularity(double contourArea, RotatedRect &boundingBox) {
        double boundingArea = boundingBox.size.area();
        if(boundingArea != 0) {
            return 100 * (contourArea/ boundingArea);
        } else {
            return 0;
        }
    }

    /**
     *
     * @param boundingBox
     * @param target
     * @return Score between 0-100
     */
    double Process::scoreAspectRatio(RotatedRect &boundingBox, Target target) {
        double rectShort = min(boundingBox.size.height, boundingBox.size.height);
        double rectLong  = max(boundingBox.size.width,  boundingBox.size.width);

        if(boundingBox.size.width > boundingBox.size.height) {
            //Particle is wider than it is tall, divide long by short
            return ratioToScore((rectLong / rectShort) / target.aspectRatio());
        } else {
            //Particle is taller than it is wide, divide short by long
            return ratioToScore((rectShort / rectLong) / target.aspectRatio());
        }
    }

}