#ifndef HIKCAMERA_H_
#define HIKCAMERA_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <fcntl.h>
#include <sys/ipc.h>
#include <sys/mman.h>

#include "MvCameraControl.h"

#include "hikcameraDataType.h"


class HikCamera
{
    public:

        struct _HIKCAMERA_PARAM_{

            int width;
            int height;
            int Offset_x;
            int Offset_y;
            bool FrameRateEnable;
            int FrameRate;

            int TriggerMode;
            int LineSelector;
            int LineMode;
            int LineSource;

            bool StrobeEnable;
            int StrobeLineDelay;
            int StrobeLinePreDelay;

            int ExposureAuto;
            int ExposureTimeUpper;
            int ExposureTimeLower;
            int ExposureTime;

            float Gain;
            int GainAuto;

            int bayerCvtQuality;

        };

    public:

        HikCamera();
        HikCamera(ros::NodeHandle &nodeHandle, int cameraIndex);
        HikCamera(int width, int height, int Offset_x, int Offset_y, bool FrameRateEnable = true, int FrameRate = 80, int ExposureTime = 5000, 
                    int GainAuto = 2, int bayerCvtQuality = 1, bool undistortion = false, double alpha = 1.0);
        ~HikCamera();


        bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);


        CAMERA_INFO camera_init();


        int setCameraParam();
        int setCameraParam(int width, int height, int Offset_x, int Offset_y, bool FrameRateEnable, int FrameRate, int ExposureTime, 
                    int GainAuto, int bayerCvtQuality = 1);


        bool setCameraIntrinsics(ros::NodeHandle &nodeHandle);
        bool setCameraIntrinsics(cv::String cameraIntrinsicsPath);
        bool setCameraIntrinsics(int imageWidth, int imageHeight, cv::Mat cameraMatrix, cv::Mat disCoffes = cv::Mat());


        CAMERA_INFO start_grab();

        sensor_msgs::ImagePtr grabOneFrame2ROS_sync(_GPRMC_TIME_STAMP_ *GPRMC_ptr);

        sensor_msgs::ImagePtr grabOneFrame2ROS();
        sensor_msgs::ImagePtr grabOneFrame2ROS(bool undistortion, int interpolation = 1);

        cv::Mat grabOneFrame2Mat();
        cv::Mat grabOneFrame2Mat(bool undistortion, int interpolation = 1);

        cv::Mat getNewCameraMatrix();

        int freeFrameCache();

        void stop_grab();

        void printParam();


    private:

        ros::NodeHandle rosHandle;

        void *camHandle;
        int camIndex;

        CAMERA_INFO cameraInfo;

        int nRet;

        _HIKCAMERA_PARAM_ hikcamera_param;

        cv::String cameraIntrinsicsPath;
        bool undistortion;
        int interpolation;

        cv::Size imageSize;
        cv::Mat cameraMatrix;
        cv::Mat disCoffes;
        double alpha;
        cv::Mat newCameraMatrix;
        cv::Size newImageSize;
        cv::Mat map1;
        cv::Mat map2;

        _GPRMC_TIME_STAMP_ GPRMC_ptr_;
};


#endif 
