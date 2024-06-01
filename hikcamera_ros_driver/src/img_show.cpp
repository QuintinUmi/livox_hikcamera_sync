#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <iostream>
#include <boost/bind.hpp>

#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"        

#include "image_transport/image_transport.h"

#include "MvCameraControl.h"
#include "CameraParams.h"

#include "hikcamera.h"
#include "kbhit.h"

bool n_exit = false;

void ImageReceiveCallBack(sensor_msgs::ImageConstPtr pImg){

    cv_bridge::CvImageConstPtr cvImage = cv_bridge::toCvShare(pImg);
    // cv::Mat outputImg;
    // cv::cvtColor(cvImage->image, outputImg, cv::COLOR_BGR2RGB);
    // cv::imshow("ImageShow",  outputImg);
    cv::imshow("ImageShow",  cvImage->image);
}

void KeyInputCallBack(std_msgs::Int8::ConstPtr ascii){

    if(ascii->data == 27){
        n_exit = true;
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "img_show");
    ros::NodeHandle rosHandle;

    ros::Subscriber imgSub = rosHandle.subscribe("/hikcamera/img", 1, ImageReceiveCallBack);
    ros::Subscriber key_input = rosHandle.subscribe("/hikcamera/key_input", 10, KeyInputCallBack);

    cv::namedWindow("ImageShow", cv::WINDOW_NORMAL);

    ros::Rate loop_rate(10);

    while(ros::ok()){

        ros::spinOnce();
        loop_rate.sleep();
        cv::waitKey(1);

        if(n_exit){
            break;
        }
    }

    return 0;
}
