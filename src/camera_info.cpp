#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdlib.h>
#include <stdio.h>

using namespace cv;

/// Global variables
int threshold_value = 0;
int threshold_type = 3;;
int const max_value = 255;
int const max_type = 4;
int const max_BINARY_value = 255;

Mat image_cam, image_cam_gray, binary, dst;
// char* window_name = "Threshold Demo";

// char* trackbar_type = "Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted";
// char* trackbar_value = "Value";

void cameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        // transform imagic topic to an OpenCV-compatible data structure using cv_bridge
        image_cam = cv_bridge::toCvShare(msg, "bgr8")->image;
        namedWindow("view", WINDOW_AUTOSIZE);
        imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
        /// Convert the image to Gray
        cvtColor( image_cam, image_cam_gray, CV_BGR2GRAY);
        threshold( image_cam_gray, binary, 3, 255, THRESH_BINARY);
        namedWindow("Output", WINDOW_AUTOSIZE);
        imshow("Output", binary);
        waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_info");
    ros::NodeHandle n;
    namedWindow("view");
    startWindowThread();
    image_transport::ImageTransport it(n);

    image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, cameraCallback);

    ros::spin();
    destroyWindow("view");
    return 0;
}