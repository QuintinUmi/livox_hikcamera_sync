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
    ros::init(argc, argv, "GPRMC_sync_grab_trigger");
    ros::NodeHandle rosHandle;
    HikCameraSync cam_h(rosHandle, 0);


    std::string image_publish_topic;
    std::string GPRMC_serial_name;
    rosHandle.param("image_publish_topic", image_publish_topic, std::string("/hikcamera/img_stream"));
    rosHandle.param("GPRMC_serial_name", GPRMC_serial_name, std::string("/dev/ttyACM0"));

    int trigger_rate;
    rosHandle.param("trigger_rate", trigger_rate, 10);

    int initRes = 0;
    // initRes |= cam_h.setExposureTime(30000000);
    initRes |= cam_h.setExposureTime(0);
    initRes |= cam_h.setGprmcTransmitTime(BR115200);
    initRes |= cam_h.initTimeSync(trigger_rate, GPRMC_serial_name, BR115200, P_8N1, "none");
    initRes |= cam_h.initCameraSettingSync(image_publish_topic);
    initRes |= cam_h.startSyncFrameGrab();

    if(initRes != 0) {
        return -1;
    }
    
    while(ros::ok() && ros::master::check()) {
        ros::Rate(50).sleep();
    }

    
    return 0;
}
