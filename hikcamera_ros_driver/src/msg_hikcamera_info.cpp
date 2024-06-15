#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <iostream>

#include <std_msgs/String.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"        

#include "MvCameraControl.h"
#include "CameraParams.h"

#include "hikcamera.h"


using namespace hikcamera_opr;

sensor_msgs::CameraInfo getCameraInfo(ros::NodeHandle rosHandle)
{
    cv::String cameraIntrinsicsPath;
    struct _cameraInfoMat
    {
        int width;
        int height;
        cv::Mat cameraMatrix;
        cv::Mat newCameraMatrix;
        cv::Mat disCoffes;

    }cameraInfoMat;
    
    
    sensor_msgs::CameraInfo cameraInfo;

    rosHandle.param("camera_instrinsics_path_yaml", cameraIntrinsicsPath, cv::String("~/caliberation_param.yaml"));
    cv::FileStorage fs(cameraIntrinsicsPath, cv::FileStorage::READ);
    fs["imageWidth"] >> cameraInfoMat.width;
    fs["imageHeight"] >> cameraInfoMat.height;
    fs["cameraMatrix"] >> cameraInfoMat.cameraMatrix;
    fs["newCameraMatrix"] >> cameraInfoMat.newCameraMatrix;
    fs["disCoffes"] >> cameraInfoMat.disCoffes;
    fs.release();

    std::vector<double> D{cameraInfoMat.disCoffes.at<double>(0), 
                        cameraInfoMat.disCoffes.at<double>(1), 
                        cameraInfoMat.disCoffes.at<double>(2), 
                        cameraInfoMat.disCoffes.at<double>(3), 
                        cameraInfoMat.disCoffes.at<double>(4)};
    boost::array<double, 9> K = {
        cameraInfoMat.cameraMatrix.at<double>(0, 0), cameraInfoMat.cameraMatrix.at<double>(0, 1), cameraInfoMat.cameraMatrix.at<double>(0, 2),
        cameraInfoMat.cameraMatrix.at<double>(1, 0), cameraInfoMat.cameraMatrix.at<double>(1, 1), cameraInfoMat.cameraMatrix.at<double>(1, 2),
        cameraInfoMat.cameraMatrix.at<double>(2, 0), cameraInfoMat.cameraMatrix.at<double>(2, 1), cameraInfoMat.cameraMatrix.at<double>(2, 2)
    };
    boost::array<double, 9> R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    boost::array<double, 12> P = {
        cameraInfoMat.newCameraMatrix.at<double>(0, 0), cameraInfoMat.newCameraMatrix.at<double>(0, 1), cameraInfoMat.newCameraMatrix.at<double>(0, 2), 0,
        cameraInfoMat.newCameraMatrix.at<double>(1, 0), cameraInfoMat.newCameraMatrix.at<double>(1, 1), cameraInfoMat.newCameraMatrix.at<double>(1, 2), 0, 
        cameraInfoMat.newCameraMatrix.at<double>(2, 0), cameraInfoMat.newCameraMatrix.at<double>(2, 1), cameraInfoMat.newCameraMatrix.at<double>(2, 2), 0
    };

    cameraInfo.width = cameraInfoMat.width;
    cameraInfo.height = cameraInfoMat.height;
    cameraInfo.distortion_model = "plumb_bob";
    cameraInfo.D = D;
    cameraInfo.K = K;
    cameraInfo.P = P;
    cameraInfo.R = R;
    cameraInfo.binning_x = 0;
    cameraInfo.binning_y = 0;
    cameraInfo.header.frame_id = "";  
    cameraInfo.header.stamp = ros::Time::now();
    cameraInfo.header.stamp.nsec = 0;

    return cameraInfo;

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "msg_camera_info");
    ros::NodeHandle rosHandle;
    
    int nRet = MV_OK;

    sensor_msgs::CameraInfo cameraInfo;
    ros::Publisher infoPub = rosHandle.advertise<sensor_msgs::CameraInfo>("/hikcamera/info", 1000);

    int loopRate = 10;
    ros::Rate loop_rate(loopRate);

    while(ros::ok()){

        cameraInfo = getCameraInfo(rosHandle);
        infoPub.publish(cameraInfo);
        ros::spinOnce();
        loop_rate.sleep();
    }

    // hikCamera.stop_grab();


    return 0;
}
