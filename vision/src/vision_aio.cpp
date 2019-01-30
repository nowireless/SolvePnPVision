#include <ros/ros.h>

#include <librealsense2/rs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#include <iostream>
#include <exception>

#include <cscore.h>

#include <vision/Process.h>


using namespace cv;
using namespace std;
using namespace vision;

cv::Mat DIST = cv::Mat();
cv::Mat CAMERA_MAT = cv::Mat();

cv::Mat frame_to_mat(const rs2::frame& f);
cv::Mat depth_frame_to_meters(const rs2::pipeline& pipe, const rs2::depth_frame& f);

int main(int argc, char** argv) {
    ros::init(argc, argv, "vision");
    ros::NodeHandle n("~");

    //Set up config, from ros param
    Config CONFIG;
    CONFIG.hsvLow = cv::Scalar(37,30,93);
    CONFIG.hsvHigh = cv::Scalar(92,255,255);
    CONFIG.closeSE = cv::Size(5,5);
    CONFIG.openSE = cv::Size(3,3);
    CONFIG.epsilon = 1;
    CONFIG.blobMax = 10;
    CONFIG.maxDistance = 5.0 * 0.0254;

    Process process(&CONFIG);

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

    // Start streaming with default recommended configuration
    auto config = pipe.start(cfg);
    auto profile = config.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    cout << "Height: " << profile.height() << " Width: " << profile.width() << endl;

    rs2::align align_to(RS2_STREAM_COLOR);

    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);

    cs::CvSource cvsource{"cvsource", cs::VideoMode::kMJPEG, profile.height(), profile.width(), 30};
    cs::MjpegServer cvMjpegServer{"cvhttpserver", 8083};
    cvMjpegServer.SetSource(cvsource);


    while (ros::ok()) {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
 
        // Make sure the frames are spatially aligned
        // data = align_to.process(data);

        // rs2::frame depth = data.get_depth_frame().apply_filter(color_map);
        rs2::frame color = data.get_color_frame();

        // Query frame size (width and height)
        const int w = color.as<rs2::video_frame>().get_width();
        const int h = color.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat image(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
        // cv::cvtColor(image, image, CV_RGB2BGR);

        ros::Time begin = ros::Time::now();
        CAMERA_MAT.copyTo(CONFIG.cameraMatrix);
        DIST.copyTo(CONFIG.distCoeffs);

        ROS_DEBUG("Got new frame");
        //Process frame
        cv::Mat destFrame;
        cv::Mat processedFrame;
        std::vector<TargetPair> results = process.ProcessFrame(image, destFrame, processedFrame);

        cvsource.PutFrame(processedFrame);
        ros::Time end = ros::Time::now();

        ROS_INFO_STREAM("Time (ms): " << (end - begin).toSec() * 1000.0);

        // Update the window with new data
        imshow(window_name, image);

        ros::spinOnce();
    }
}

// Convert rs2::frame to cv::Mat
cv::Mat frame_to_mat(const rs2::frame& f)
{
    using namespace cv;
    using namespace rs2;

    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if (f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        auto r = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
        cvtColor(r, r, CV_RGB2BGR);
        return r;
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
}

// Converts depth frame to a matrix of doubles with distances in meters
cv::Mat depth_frame_to_meters(const rs2::pipeline& pipe, const rs2::depth_frame& f)
{
    using namespace cv;
    using namespace rs2;

    Mat dm = frame_to_mat(f);
    dm.convertTo(dm, CV_64F);
    auto depth_scale = pipe.get_active_profile()
        .get_device()
        .first<depth_sensor>()
        .get_depth_scale();
    dm = dm * depth_scale;
    return dm;
}