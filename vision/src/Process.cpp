//
// Created by ryan on 1/5/17.
//
#include <math.h>
#include <algorithm>

#include <ros/ros.h>

#include <Eigen/Eigen>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <vision/Process.h>
#include <vision/Target.h>
#include <sstream>


using namespace cv;
using namespace std;

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
    std::vector<TargetPair> Process::ProcessFrame(Mat source, Mat &dest, Mat &processed) {
        vector<Contour> polyContours = featureExtraction(source, dest, processed);
        ROS_DEBUG("Found blobs %lu", polyContours.size());

        vector<TargetInfo> foundTargets = targetIdentification(polyContours, dest, processed);
        ROS_DEBUG("Found targets %lu", foundTargets.size());

        for(auto target : foundTargets) {
            Point2f vertices[4];
            target.boundingBox.points(vertices);

            drawContours(dest, polyContours, target.polyIndex, Scalar(0,0,255), 2);
            drawContours(processed, polyContours, target.polyIndex, Scalar(0,0,255), 2);
            for (int i = 0; i < 4; i++) {
                line(processed, vertices[i], vertices[(i + 1) % 4], Scalar(255, 0, 0), 2);
            }

            std::stringstream ss;
            ss << target.compositeScore;
            cv::putText(processed, ss.str(), target.boundingBox.center, 1.0, FONT_HERSHEY_PLAIN, cv::Scalar(255,255,255));
        }

        vector<TargetPair> targetPairs = findTargetPairs(foundTargets, dest);
        ROS_DEBUG("Found targets pairs %lu", targetPairs.size());

        for (auto pair : targetPairs) {
            line(processed, pair.left.boundingBox.center, pair.right.boundingBox.center, Scalar(0, 255, 255), 2);
        }

        return dataExtraction(polyContours, targetPairs, dest);
    }

    /**
     *
     * @param source
     * @param dest
     * @return
     */
    vector<Contour> Process::featureExtraction(cv::Mat source, cv::Mat &dest, Mat &processed) {
        ROS_DEBUG("Feature extraction start");
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
        source.copyTo(processed);

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
        ROS_INFO_STREAM("Contours: " << contours.size());

        // Commented out due to how episilon interactes with low resolution (small) targets
        //Find Polygon contours
        vector<Contour> resultContours;
        for(int i = 0; i < contours.size(); i++) {
            Contour contour = contours[i];
//            //NOTE the true at the end to signal that the resulting contour is closed
//            approxPolyDP(contours[i], polyContour, m_config->epsilon, true);
//
//            drawContours(processed, contours, i, Scalar(0,255,255), 2);
//
//
            //If a point is near the border of the image ignore the contour,
            //as there may be issues with a partial target being detected.
            //This could cause bad pose estimation.
            if(nearBorder(contour, source.size(), 10)) {
                ROS_INFO("Skipping contour as it is near border");
                continue;
            }

            const double minArea = 200;
            double area = contourArea(contour);
            if (area < minArea) {
                ROS_INFO("Skipping contour as it is too small");
                continue;
            }
//
//            //Check to see if the contour has a valid number of sides, if so add it
////            int sides = polyContour.size();
////            if(sides == 4) {
//                //Rectangle
//                polyContours.push_back(polyContour);
////            } else {
//
////            }
            resultContours.push_back(contour);
        }


        //Sort Contours from largest to smallest
        sort(contours.begin(), contours.end(),
             [](Contour a, Contour b) { return contourArea(a) > contourArea(b); }
        );

        ROS_DEBUG("Feature extraction end");

        return resultContours;
    }

    vector<TargetInfo> Process::targetIdentification(std::vector<Contour> polyContours, cv::Mat &dest, Mat &processed) {
        ROS_DEBUG("Target Identification start");
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
            // int size = polyContours[i].size();
            double area = contourArea(polyContours[i]);
            RotatedRect boundingBox = minAreaRect(polyContours[i]);

            double rectScore = scoreRectangularity(area, boundingBox);
            // The left and right targets have the same aspect ratio
            double aspectRatioScore = scoreAspectRatio(boundingBox, Target::kLEFT_TILT_TARGET);

            double angle = targetAngle(boundingBox);

            // This is used to reject vertical targets
            const double minTilt = 2;

            ROS_DEBUG("Blob %i Score: Rect %f AR %f", i, rectScore, aspectRatioScore);
            if(rectScore >= 50 && aspectRatioScore >= 20 && minTilt < fabs(angle)) {
                TargetInfo info;
                info.polyIndex = i;
                info.boundingBox = boundingBox;

                ROS_INFO_STREAM("Angle" <<  angle);
                if (angle > 0) {
                    // Tilted counter clock wise, right target
                    info.targetID = TargetID::kRightTilt;
                } else {
                    // Tilted clock wise, left target
                    info.targetID = TargetID::kLeftTilt;
                }
                info.compositeScore = (rectScore * 0.5) + (aspectRatioScore * 0.5);
                foundTargets.push_back(info);
            }
        }
        ROS_DEBUG("Target Identification end");
        return foundTargets;
    }

    vector<TargetPair> Process::findTargetPairs(std::vector<TargetInfo> foundTargets, cv::Mat &dest) {
        ROS_INFO("Find target pairs start");
        vector<TargetPair> results;

        for(auto target : foundTargets) {
            switch (target.targetID) {
            case TargetID::kNotATarget:
                ROS_INFO("Not a target target");
                break;
            case TargetID::kLeftTilt:
                ROS_INFO("Left tilt target");
                break;
            case TargetID::kRightTilt:
                ROS_INFO("Right tilt target");
                break;
            default:
                ROS_INFO("Unknown target");
                break;

            }
        }

        // Remove unintresting targets
        foundTargets.erase(remove_if(foundTargets.begin(), foundTargets.end(), [](TargetInfo info) {
            bool remove =  !(info.targetID == TargetID::kLeftTilt || info.targetID == TargetID::kRightTilt);
            ROS_INFO_STREAM("Remove: " << remove);
            return remove;
        }), foundTargets.end());


        ROS_INFO_STREAM("Target count " << foundTargets.size());

        // If there are less than 2 foudn targets there is no point in proceeding
        if (foundTargets.size() < 2) {
            ROS_INFO("Less than 2 targets found");
            return results;
        }

        //Sort Targets from left to right x position (sort as increasing x)
        sort(foundTargets.begin(), foundTargets.end(),
             [](TargetInfo a, TargetInfo b) { return a.boundingBox.center.x < b.boundingBox.center.x; }
        );

        for (auto target : foundTargets) {
            ROS_INFO_STREAM(target.targetID << ", " <<target.boundingBox.center.x);
        }

        // Try to find a matching pair
        // A matching pair is found when the left side target is a right tilted target and then the next
        // target is a left tilt target.


        // Start with left most target
        // It is assumed that only left and right ta
        auto it = foundTargets.begin();
        while(it != foundTargets.end()) {
            ROS_INFO("Finding pair");
            TargetInfo leftSide = *it;
            ROS_INFO_STREAM("Left: " << leftSide.targetID << ", contour index: " << leftSide.polyIndex);

            it++; // Advance to next target
            if (leftSide.targetID != TargetID::kRightTilt) {
                ROS_INFO_STREAM("Left side target is not tilted right");
                continue; // Try again
            }


            TargetInfo rightSide = *it;
            ROS_INFO_STREAM("Right: " << rightSide.targetID << ", contour index: " << rightSide.polyIndex);
            if (rightSide.targetID != TargetID::kLeftTilt) {
                // This is not a right side target, lets try this process again with the left one being this target
                ROS_INFO_STREAM("Left side target is not tilted left");
                continue;
            }

            // Its a right target advance iterator for next iteration of this loop
            it++;


            // Now try to determine if this a valid target pair.
            // This can be done by scoring the distance between the 2 targets centers
            double distance = pointToPointDistance(leftSide.boundingBox.center, rightSide.boundingBox.center);

            double leftRatio = longSide(leftSide.boundingBox.size) / distance;
            double rightRatio = longSide(rightSide.boundingBox.size) / distance;

            const double goalRatio = 0.5; // TODO find
            double leftScore = ratioToScore(goalRatio / leftRatio);
            double rightScore = ratioToScore(goalRatio / rightRatio);

            ROS_INFO_STREAM("Left ratio: " << leftRatio << ", score: " << leftScore);
            ROS_INFO_STREAM("Right ratio: " << rightRatio << ", score: " << rightScore);


            const double minScore = 1; // TOOD find
            if (leftScore > minScore && rightScore > minScore) {
                ROS_INFO_STREAM("Found a target pair!");
                // We found a target pair!
                TargetPair pair;
                pair.left = leftSide;
                pair.right = rightSide;
                results.push_back(pair);
            }
        }

        ROS_INFO("Find target pairs end");

        return results;
    }

    vector<TargetPair> Process::dataExtraction(vector<Contour> polyContours, vector<TargetPair> targetPairs, Mat &dest) {
        ROS_DEBUG("Data extraction start");
        /**
         * Data extraction
         * ===============
         * Get target position relative to camera.
         */
        vector<TargetPair> results;

        //If no targets are found, no point in proceeding
        if(targetPairs.size() == 0) return results;

        // Use corner points of the bounding boxes are points to supply to solvePnP

        Mat rvec(3,1, DataType<double>::type);
        Mat tvec(3,1, DataType<double>::type);
        for(auto pair : targetPairs) {
            TargetInfo left = pair.left;
            TargetInfo right = pair.right;

            //X - forward   - blue
            //Y - left      - green
            //Z - up        - red

            // TODO: Would more points be better for Solve PNP?

            // Figure out object points
            vector<Point3f> objectPoints;

            // Top left
            Eigen::Vector3f objectTL;
            objectTL << -1, 2.75, 1;
            objectTL = transform(objectTL, -11.313/2, 0, -14.5 * M_PI / 180.0);
            objectPoints.push_back(Point3f(0, objectTL[0], objectTL[1]));

            // Top Right
            Eigen::Vector3f objectTR;
            objectTR << 1, 2.75, 1;
            objectTR = transform(objectTR, 11.313/2, 0, 14.5 * M_PI / 180.0);
            objectPoints.push_back(Point3f(0, objectTR[0], objectTR[1]));

            // Bottom Left
            Eigen::Vector3f objectBL;
            objectBL << -1, -2.75, 1;
            objectBL = transform(objectBL, -11.313/2, 0, -14.5 * M_PI / 180.0);
            objectPoints.push_back(Point3f(0, objectBL[0], objectBL[1]));

            // Bottom Right
            Eigen::Vector3f objectBR;
            objectBR << 1, -2.75, 1;
            objectBR = transform(objectBR, 11.313/2, 0, 14.5 * M_PI / 180.0);
            objectPoints.push_back(Point3f(0, objectBR[0], objectBR[1]));


            ROS_INFO_STREAM("Object Top Left: " << objectTL);
            ROS_INFO_STREAM("Object Top Right: " << objectTR);
            ROS_INFO_STREAM("Object Bottom Left: " << objectBL);
            ROS_INFO_STREAM("Object Bottom Right: " << objectBR);

            // Figure out image points
             vector<Point2f> imagePoints;
            // imagePoints.push_back(extremeBR);
            // imagePoints.push_back(extremeBL);
            // imagePoints.push_back(extremeTL);
            // imagePoints.push_back(extremeTR);


            // TODO: Is it okay to invert the bounding box angle? If we did then the Y axis would also need to inverted
            double leftAngle = left.boundingBox.angle * M_PI / 180.0;
            double leftLength = longSide(left.boundingBox.size);
            double leftShort = shortSide(left.boundingBox.size);

            double rightAngle = right.boundingBox.angle * M_PI / 180.0;
            double rightLength = longSide(right.boundingBox.size);
            double rightShort = shortSide(right.boundingBox.size);

            // Top Left
            Eigen::Vector3f imageTL;
            imageTL << -leftShort / 2.0, leftLength/2.0, 1;
            imageTL = transform(imageTL, left.boundingBox.center.x, left.boundingBox.center.y, leftAngle);
            imagePoints.push_back(Point2f(imageTL[0], imageTL[1]));

            // Top Right
            Eigen::Vector3f imageTR;
            imageTR << rightShort / 2.0, rightLength/2.0, 1;
            imageTR = transform(imageTL, right.boundingBox.center.x, right.boundingBox.center.y, rightAngle);
            imagePoints.push_back(Point2f(imageTR[0], imageTR[1]));

            // Bottom Left
            Eigen::Vector3f imageBL;
            imageBL<< -leftShort / 2.0, -leftLength/2.0, 1;
            imageBL = transform(imageBL, left.boundingBox.center.x, left.boundingBox.center.y, leftAngle);
            imagePoints.push_back(Point2f(imageBL[0], imageBL[1]));

            // Bottom Right
            Eigen::Vector3f imageBR;
            imageBR << rightShort / 2.0, -rightLength/2.0, 1;
            imageBR = transform(imageBL, right.boundingBox.center.x, right.boundingBox.center.y, rightAngle);
            imagePoints.push_back(Point2f(imageBR[0], imageBR[1]));

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

            pair.pose.x = x;
            pair.pose.y = y;
            pair.pose.z = z;
            pair.rotation.yaw   = yaw;
            pair.rotation.pitch = pitch;
            pair.rotation.roll  = roll;
            results.push_back(pair);

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
        ROS_DEBUG("Data extraction end");

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

    /**
     * Normalize angle between 2 bounds
     * @param value Angle
     * @param start Start andgle, Ex: 0, -180
     * @param end End angle, Ex: 360, 180
     * @return Normalized angle
     */
    double Process::normalizeAngle(const double value, const double start, const double end) {
        const double width = end - start;
        const double offsetValue = value - start;

        return (offsetValue - (floor(offsetValue/width) * width)) + start;
    }

    /**
     * Calculate the angle (in degrees) that the target is tilted. 0 degress means that the target is facing straight up.
     * @param box Bounding box of target
     * @return Positive angles clockwise, negative angles counter clockwise
     */
    double Process::targetAngle(cv::RotatedRect box) {
        // How Min Area Rect calculates rotated rect: https://namkeenman.wordpress.com/2015/12/18/open-cv-determine-angle-of-rotatedrect-minarearect/
        double angle = box.angle;
        if (box.size.width > box.size.height) {
            angle += 90;
        }
        return normalizeAngle(angle, -180, 180);
    }

    /**
     *
     * @param image
     * @param center
     * @param length
     * @param angle
     * @param color
     */
    void Process::drawBoxLine(cv::Mat image, cv::Point center, double length, double angle, cv::Scalar color) {
        cv::Point endPoint;
        endPoint.x = center.x + 100.0 * cos(angle * M_PI / 180);
        endPoint.y = center.y + 100.0 * sin(angle * M_PI / 180);
        line(image, center, endPoint, color, 2);
    }

    double Process::distanceScore(TargetInfo left, TargetInfo right) {
//        // TODO :This may not be the best way to determine distance between targets.
//
//        double ratioLeft;
//        double ratioRight;
//        double ratio =  (ratioLeft + ratioRight) / 2.0;
//        return ratioToScore()
    }

    double Process::pointToPointDistance(cv::Point a, cv::Point b) {
        double delx = b.x - a.x;
        double dely = b.y - a.y;
        return sqrt(delx * delx + dely * dely);
    }

    cv::Point Process::getMidPoint(cv::Point a, cv::Point b) {
        return cv::Point((a.x + b.x) / 2.0, (a.y + b.y) / 2.0);
    }

    double Process::longSide(cv::Size size) {
        if (size.height > size.width) {
            return size.height;
        }
        return size.width;
    }

    double Process::shortSide(cv::Size size) {
        if (size.height < size.width) {
            return size.height;
        }
        return size.width;
    }

    Eigen::Vector3f Process::transform(Eigen::Vector3f point, double h, double k, double theta) {
        std::cout <<  "Point: " << point << std::endl;
        Eigen::Matrix3f rotate;
        rotate << cos(theta), -sin(theta), 0,
                  sin(theta), cos(theta),  0,
                  0,          0,           1;

        std::cout <<  "Rotate: " << rotate << std::endl;

        Eigen::Matrix3f translate;
        translate << 1, 0, h,
                     0, 1, k,
                     0, 0, 1;
        std::cout <<  "Translate: " << translate << std::endl;

        auto result = (translate * rotate) * point;
        return result;

    }


}