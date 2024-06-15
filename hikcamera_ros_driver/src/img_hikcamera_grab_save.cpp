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


using namespace hikcamera_opr;

#define MAX_IMAGE_INDEX 200

bool g_exit = false;

void KeyInput_CallBack(std_msgs::Int8::ConstPtr key_ascii, ros::NodeHandle rosHandle, image_transport::Publisher* imgPub,cv::Mat* cvImage){

    cv::String filePath, fileName;
    rosHandle.param("image_file_save_path", filePath, std::string("~/"));

    int imgIndex = 0;
    
    if(key_ascii->data == 10){
        // printf("\n----------------------------------------------------------------------------------%d\n", key_ascii->data);
        while(imgIndex <= MAX_IMAGE_INDEX){
            imgIndex ++;
            std::string tempIndex = std::to_string(imgIndex);
            fileName = filePath + tempIndex + ".png";
            if(cv::imread(fileName).empty() && !cvImage->empty()){
                // std::cout << fileName <<"\n";
                // getchar();
                // cv::namedWindow("ShowSavedImage", cv::WINDOW_NORMAL);
                // cv::imshow("ShowSavedImage", *cvImage);
                // cv::waitKey(0); 
                cv::Mat outputImg;
                cv::cvtColor(*cvImage, outputImg, cv::COLOR_BGR2RGB);
                cv::imwrite(fileName, outputImg, {cv::IMWRITE_PNG_COMPRESSION, 0}); 
                imgPub->publish(cv_bridge::CvImage(std_msgs::Header(), "rgb8", outputImg).toImageMsg());      
                
                break;
            }
            // }
        }
    }
    if(key_ascii->data == 27){
        g_exit = true;
    }
            
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "msg_camera_grab");
    ros::NodeHandle rosHandle;
    HikCamera hikCamera(rosHandle, 0);
    
    int nRet = MV_OK;

    cv::Mat cvImage;
    sensor_msgs::Image imgOneFrame;
    sensor_msgs::ImagePtr imgMsg;
    CAMERA_INFO cameraInfo = hikCamera.initDevice();

    cv::String filePath, fileName;

    image_transport::ImageTransport imgIt(rosHandle);
    //topic name is /camera_front/image_color,the publish message queue size is 1.
    image_transport::Publisher imgStreamPub = imgIt.advertise("/hikcamera/img_stream", 1);
    image_transport::Publisher imgPub = imgIt.advertise("/hikcamera/img", 1);

    // ros::Publisher imgPub = rosHandle.advertise<sensor_msgs::Image>("/hikcamera/img", 100);
    ros::Publisher msgPub = rosHandle.advertise<std_msgs::String>("/hikcamera/std_msgs", 100);
    ros::Subscriber keySub = rosHandle.subscribe<std_msgs::Int8>("/hikcamera/key_input", 10, 
                                                                boost::bind(&KeyInput_CallBack, _1, rosHandle, &imgPub, &cvImage));

    nRet = hikCamera.setCameraParam();
    if(MV_OK != nRet){
        printf("There is something wrong with hikcamera parameters setting!\n");
        getchar();
    }
    cameraInfo = hikCamera.start_grab();

    void *pUser = cameraInfo.pUser;
    unsigned int nDataSize = cameraInfo.nDataSize;
    MV_FRAME_OUT* stImageInfo;
    

    // int loopRate;
    // rosHandle.param("ros-image-publish-rate", loopRate, 100);
    // ros::Rate loop_rate(loopRate);
    while(ros::ok()){

        ros::spinOnce();
        
        hikCamera.freeFrameCache();

        if(g_exit){
            break;
        }

        cvImage = hikCamera.grabOneFrame2Mat();
        if(cvImage.empty())
        {
        	continue;
        }
        imgMsg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", cvImage).toImageMsg();
        
        imgStreamPub.publish(imgMsg);        
        // loop_rate.sleep();
    }

    // hikCamera.stop_grab();


    return 0;
}
