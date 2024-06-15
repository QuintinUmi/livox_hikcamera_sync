#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <iostream>

#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"        

#include "image_transport/image_transport.h"

#include "MvCameraControl.h"
#include "CameraParams.h"

#include "hikcamera.h"


using namespace hikcamera_opr;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "msg_camera_grab");
    ros::NodeHandle rosHandle;
    HikCamera hikCamera(rosHandle, 0);
    
    std::string image_publish_topic;
    rosHandle.param("image_publish_topic", image_publish_topic, std::string("/hikcamera/img_stream"));

    int nRet = MV_OK;
    sensor_msgs::Image imgOneFrame;
    CAMERA_INFO cameraInfo = hikCamera.initDevice();

    image_transport::ImageTransport imgIt(rosHandle);
    image_transport::Publisher imgPub = imgIt.advertise(image_publish_topic, 2);

    // ros::Publisher imgPub = rosHandle.advertise<sensor_msgs::Image>("/hikcamera/img", 100);
    ros::Publisher msgPub = rosHandle.advertise<std_msgs::String>("/hikcamera/std_msgs", 100);

    nRet = hikCamera.setCameraParam();
    if(MV_OK != nRet){
        printf("Error Occurred in hikcamera parameters setting!\n");
        getchar();
    }
    cameraInfo = hikCamera.start_grab();

    void *pUser = cameraInfo.pUser;
    unsigned int nDataSize = cameraInfo.nDataSize;
    MV_FRAME_OUT* stImageInfo;
    sensor_msgs::ImagePtr imgMsg;

    int loopRate;
    rosHandle.param("ros_image_publish_rate", loopRate, 100);
    ros::Rate loop_rate(loopRate);
    
    

    while(ros::ok()){

        ros::spinOnce();
        imgMsg = hikCamera.grabOneFrame2ROS();
        if(!imgMsg)
        {
        	continue;
        }
        // hikCamera.freeFrameCache();

        imgPub.publish(imgMsg);
        
        loop_rate.sleep();
    }

    // hikCamera.stop_grab();


    return 0;
}
