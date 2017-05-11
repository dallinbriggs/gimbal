#include "ros/ros.h"
#include "std_msgs/String.h"

//#include <sstream>

#include <iostream>
//#include "opencv2/opencv.hpp"
//#include "opencv2/videoio.hpp"
//#include <fstream>
#include <string>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/UInt16.h>


#include <visp_bridge/image.h>
#include <visp_bridge/camera.h>

#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpImageIo.h>

#include <visp/vpImage.h>
#include <visp/vpV4l2Grabber.h>


using namespace cv;
//using namespace std;

ros::Publisher gimbal_yaw_pub;
ros::Publisher gimbal_pitch_pub;
int angle_yaw = 0;
int angle_pitch = 130;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    Mat cv_image, image_hsv, image_orange;

    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    double largest_area = 0;
    int largest_area_iter = 0;
    Moments contour_moments;
    int cx, cy;

    int erosion_size = 5;
    int dilation_size = 5;
    Mat element_erosion = getStructuringElement( MORPH_ELLIPSE,
                                                 Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                                 Point( erosion_size, erosion_size ) );

    Mat element_dilate = getStructuringElement( MORPH_ELLIPSE,
                                                Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                                Point( dilation_size, dilation_size ) );
    try
    {
        cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv_image = image_ptr->image;
        flip(cv_image, cv_image, 0);
        cvtColor(cv_image, image_hsv, CV_BGR2HSV);
        inRange(image_hsv, Scalar(0,178,111),Scalar(57,255,255),image_orange);
        GaussianBlur(image_orange,image_orange, Size(15,15), 5,5 );
        erode( image_orange,image_orange, element_erosion );
        dilate(image_orange,image_orange, element_dilate);
        findContours(image_orange,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE,Point(0,0));
        if(!contours.empty())
        {
            for(int i = 0; i < contours.size(); i++)
            {
                double area = contourArea(contours[i], false);
                if (area > largest_area)
                {
                    largest_area = area;
                    largest_area_iter = i;
                }
            }

            contour_moments = moments(contours[largest_area_iter],true);
            cx = int(contour_moments.m10/contour_moments.m00);
            cy = int(contour_moments.m01/contour_moments.m00);


            int left_b = 494;
            int right_b = 794;
            int upper_b = 369;
            int lower_b = 595;
            float P_yaw = 2;
            float P_pitch = 2;

            if(cx < left_b)
            {
                angle_yaw = angle_yaw + 1*P_yaw;
                std_msgs::UInt16 angle_msg_yaw;
                angle_msg_yaw.data = int(angle_yaw);
                if(angle_yaw > 90)
                {
                    angle_yaw = 90;
                    angle_msg_yaw.data = angle_yaw;
                }
                gimbal_yaw_pub.publish(angle_msg_yaw);
            }
            if(cx > right_b)
            {
                angle_yaw = angle_yaw - 1*P_yaw;
                std_msgs::UInt16 angle_msg_yaw;
                angle_msg_yaw.data = int(angle_yaw);
                if(angle_yaw < 0)
                {
                    angle_yaw = 0;
                    angle_msg_yaw.data = angle_yaw;
                }
                gimbal_yaw_pub.publish(angle_msg_yaw);
            }

            if(cy < lower_b)
            {
                angle_pitch = angle_pitch + 1*P_pitch;
                std_msgs::UInt16 angle_msg_pitch;
                angle_msg_pitch.data = int(angle_pitch);
                if(angle_pitch > 130)
                {
                    angle_pitch = 130;
                    angle_msg_pitch.data = angle_pitch;
                }
                gimbal_pitch_pub.publish(angle_msg_pitch);
            }
            if(cy > upper_b)
            {
                angle_pitch = angle_pitch - 1*P_pitch;
                std_msgs::UInt16 angle_msg_pitch;
                angle_msg_pitch.data = int(angle_pitch);
                if(angle_pitch < 40)
                {
                    angle_pitch = 40;
                    angle_msg_pitch.data = angle_pitch;
                }
                gimbal_pitch_pub.publish(angle_msg_pitch);
            }

            line(image_orange, Point(left_b,0), Point(left_b, image_orange.rows), Scalar(200,200,200), 5, LINE_8, 0);
            line(image_orange, Point(right_b,0), Point(right_b, image_orange.rows), Scalar(200,200,200), 5, LINE_8, 0);
            line(image_orange, Point(0,upper_b), Point(image_orange.cols, upper_b), Scalar(200,200,200), 5, LINE_8, 0);
            line(image_orange, Point(0, lower_b), Point(image_orange.cols, lower_b), Scalar(200,200,200), 5, LINE_8, 0);
            circle(image_orange, Point(cx, cy), 100, Scalar(200,200,200), CV_FILLED, LINE_8, 0);
        }
        imshow("view", image_orange);
        waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    ROS_INFO("Image size: [%i]", cv_image.cols);
    ROS_INFO("Image size: [%i]", cv_image.rows);

}

void visp_image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    sensor_msgs::Image image_msg;
    image_msg = *msg;
    vpImage<vpRGBa> image;
    image = visp_bridge::toVispImageRGBa(*msg);
    vpV4l2Grabber g;
//    g.open(image);
    try {
#if defined(VISP_HAVE_X11)
        vpDisplayX d(image);
#elif defined(VISP_HAVE_GDI)
        vpDisplayGDI d(image);
#ielf defined(VISP_HAVE_OPENCV)
        vpDisplayOpenCV d(image);
#elif defined(VISP_HAVE_GTK)
        vpDisplayGTK d(image);
#elif defined(VISP_HAVE_D3D9)
        vpDisplayD3d d(image);
#else
        std::cout << "No image viewer is available..." << std::endl;
#endif
//        while(1) {
//            g.acquire(image);
//            vpDisplay::display(image);
//            vpDisplay::flush(image);
//            if (vpDisplay::getClick(image, false)) break;
//        }

    }
    catch(vpException e) {
        std::cout << "Catch an exception: " << e << std::endl;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracker");
    ros::NodeHandle nh;

    cv::namedWindow("view");
    cv::startWindowThread();

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, imageCallback);
//    image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, visp_image_callback);

    gimbal_yaw_pub = nh.advertise<std_msgs::UInt16>("gimbal_yaw", 1);
    gimbal_pitch_pub = nh.advertise<std_msgs::UInt16>("gimbal_pitch", 1);


    ros::spin();
    cv::destroyWindow("view");


    return 0;
}

