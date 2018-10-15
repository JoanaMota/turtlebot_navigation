#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"
#include <cv_bridge/cv_bridge.h>
#include <stdlib.h>
#include <stdio.h>

using namespace cv;
using namespace std;

/// Global variables
int threshold_value = 0;
int threshold_type = 3;;
int const max_value = 255;
int const max_type = 4;
int const max_BINARY_value = 255;

Mat image_cam, image_cam_gray, binary, dst, cdst, cdstP;
// char* window_name = "Threshold Demo";

// char* trackbar_type = "Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted";
// char* trackbar_value = "Value";

void cameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        // transform imagic topic to an OpenCV-compatible data structure using cv_bridge
        image_cam = cv_bridge::toCvShare(msg, "bgr8")->image;
        // namedWindow("view", WINDOW_AUTOSIZE);
        imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
        // Convert the image to Gray
        cvtColor( image_cam, image_cam_gray, CV_BGR2GRAY);
        threshold( image_cam_gray, binary, 3, 255, THRESH_BINARY);
        // namedWindow("Output", WINDOW_AUTOSIZE);
        // imshow("Output", binary);

        // Edge detection
        Canny(binary, dst, 50, 200, 3);
        // Copy edges to the images that will display the results in BGR
        cvtColor(dst, cdst, COLOR_GRAY2BGR);
        cdstP = cdst.clone();
        // Standard Hough Line Transform
        vector<Vec2f> lines; // will hold the results of the detection
        HoughLines(dst, lines, 1, CV_PI/180, 150, 0, 0 ); // runs the actual detection
        // Draw the lines
        for( size_t i = 0; i < lines.size(); i++ )
        {
            float rho = lines[i][0], theta = lines[i][1];
            Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0 + 1000*(-b));
            pt1.y = cvRound(y0 + 1000*(a));
            pt2.x = cvRound(x0 - 1000*(-b));
            pt2.y = cvRound(y0 - 1000*(a));
            line( cdst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
        }
        // Probabilistic Line Transform
        vector<Vec4i> linesP; // will hold the results of the detection
        HoughLinesP(dst, linesP, 1, CV_PI/180, 50, 50, 10 ); // runs the actual detection
        // Draw the lines
        for( size_t i = 0; i < linesP.size(); i++ )
        {
            Vec4i l = linesP[i];
            line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
        }
        // Show results
        imshow("Source", binary);
        imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst);
        imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP);
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