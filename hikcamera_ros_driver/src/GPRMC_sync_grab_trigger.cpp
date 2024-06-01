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


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "msg_camera_grab");
    ros::NodeHandle rosHandle;
    HikCamera hikCamera(rosHandle, 0);


    std::string image_publish_topic;
    std::string GPRMC_sync_serial_path;
    rosHandle.param("image_publish_topic", image_publish_topic, std::string("/hikcamera/img_stream"));
    rosHandle.param("GPRMC_sync_serial_path", GPRMC_sync_serial_path, std::string("/hikcamera/img_stream"));

    
    int nRet = MV_OK;
    sensor_msgs::Image imgOneFrame;
    CAMERA_INFO cameraInfo = hikCamera.camera_init();

    image_transport::ImageTransport imgIt(rosHandle);
    image_transport::Publisher imgPub = imgIt.advertise("/hikcamera/img_stream", 1);

    // ros::Publisher imgPub = rosHandle.advertise<sensor_msgs::Image>("/hikcamera/img", 100);
    ros::Publisher msgPub = rosHandle.advertise<std_msgs::String>("/hikcamera/std_msgs", 100);

    nRet = hikCamera.setCameraParam();
    if(MV_OK != nRet){
        printf("There is something wrong with hikcamera parameters setting!\n");
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
    
    int fd = open(GPRMC_sync_serial_path.c_str(), O_RDWR);
    _GPRMC_TIME_STAMP_ *GPRMC_ptr = (_GPRMC_TIME_STAMP_*)mmap(NULL, sizeof(_GPRMC_TIME_STAMP_), PROT_READ | PROT_WRITE,
                                    MAP_SHARED, fd, 0);

    while(ros::ok()){

        ros::spinOnce();
        imgMsg = hikCamera.grabOneFrame2ROS_sync(GPRMC_ptr);
        if(!imgMsg)
        {
        	continue;
        }
        // hikCamera.freeFrameCache();

        imgPub.publish(imgMsg);
        
        loop_rate.sleep();
    }

    munmap(GPRMC_ptr, sizeof(_GPRMC_TIME_STAMP_) * 5);

    hikCamera.stop_grab();

    
    return 0;
}
