#ifndef HIKCAMERA_H_
#define HIKCAMERA_H_

#include <condition_variable>
#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <fcntl.h>
#include <sys/ipc.h>
#include <sys/mman.h>

#include "MvCameraControl.h"

#include "hikcameraDataType.h"
#include "shm_handler/shm_handler.h"
#include "timesync/timesync.h"


using namespace shm_handler;
using namespace livox_ros;

namespace hikcamera_opr
{
    class HikCamera
    {
        public:

            typedef struct HIKCAMERA_PARAM{

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

            }HIKCAMERA_PARAM;

        public:

            HikCamera();
            HikCamera(ros::NodeHandle &nodeHandle, int cameraIndex);
            HikCamera(int width, int height, int Offset_x, int Offset_y, bool FrameRateEnable = true, int FrameRate = 80, int ExposureTime = 5000, 
                        int GainAuto = 2, int bayerCvtQuality = 1, bool undistortion = false, double alpha = 1.0);
            virtual ~HikCamera();


            bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);


            CAMERA_INFO initDevice();

            int setCameraParam();
            int setCameraParam(int width, int height, int Offset_x, int Offset_y, bool FrameRateEnable, int FrameRate, int ExposureTime, 
                        int GainAuto, int bayerCvtQuality = 1);


            bool setCameraIntrinsics(ros::NodeHandle &nodeHandle);
            bool setCameraIntrinsics(cv::String cameraIntrinsicsPath);
            bool setCameraIntrinsics(int imageWidth, int imageHeight, cv::Mat cameraMatrix, cv::Mat disCoffes = cv::Mat());


            CAMERA_INFO start_grab();

            sensor_msgs::ImagePtr grabOneFrame2ROS();
            sensor_msgs::ImagePtr grabOneFrame2ROS(bool undistortion, int interpolation = 1);

            cv::Mat grabOneFrame2Mat();
            cv::Mat grabOneFrame2Mat(bool undistortion, int interpolation = 1);
            
            ros::NodeHandle getRosHandler();
            cv::Mat getNewCameraMatrix();

            int freeFrameCache();

            void stop_grab();

            void printParam();

        protected:
            CAMERA_INFO cameraInfo;

            HIKCAMERA_PARAM hikcamera_param;

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

            bool isResize;
            cv::Size imageReize;

        private:

            ros::NodeHandle rosHandle;

            void *camHandle;
            int camIndex;

            int nRet;

    };


    class HikCameraSync : public HikCamera {
        
        public:
            using CLK = std::chrono::high_resolution_clock;
            using NS = std::chrono::nanoseconds;

            typedef struct TIME_STAMP
            {
                int64_t seq;
                int64_t base_time;
            }TIME_STAMP;

            struct FramePacket {
                uint32_t seq;
                std::shared_ptr<MV_FRAME_OUT> frame;
                CLK::time_point rcv_time;
                uint64_t sync_time_stamp;
                volatile bool is_sync_base;

                FramePacket(const MV_FRAME_OUT& frame_out, uint32_t frame_seq, CLK::time_point frame_rcv_time)
                    : frame(std::make_shared<MV_FRAME_OUT>(frame_out)), seq(frame_seq), rcv_time(frame_rcv_time), is_sync_base(false) {}
            };

            typedef void (*PublishCb)(FramePacket frame_pkg, uint64_t time_stamp, void* client_data);

            HikCameraSync() : HikCamera(),
                            start_get_frame_wt_(false), exit_get_frame_wt_(false),
                            start_queue_process_wt_(false), exit_queue_process_wt_(false)
                            {initValuable();};

            HikCameraSync(ros::NodeHandle &nodeHandle, int cameraIndex) : HikCamera(nodeHandle, cameraIndex),
                            start_get_frame_wt_(false), exit_get_frame_wt_(false),
                            start_queue_process_wt_(false), exit_queue_process_wt_(false)
                            {initValuable();};

            HikCameraSync(int width, int height, int Offset_x, int Offset_y, bool FrameRateEnable = true, int FrameRate = 80, int ExposureTime = 5000, 
                        int GainAuto = 2, int bayerCvtQuality = 1, bool undistortion = false, double alpha = 1.0)
                        : HikCamera(width, height, Offset_x, Offset_y, FrameRateEnable, FrameRate, ExposureTime, 
                        GainAuto, bayerCvtQuality , undistortion, alpha),
                            start_get_frame_wt_(false), exit_get_frame_wt_(false),
                            start_queue_process_wt_(false), exit_queue_process_wt_(false)
                            {initValuable();};

            ~HikCameraSync() override = default;

            int initTimeSync(uint32_t freq, const std::string& device_name, uint8_t baudrate_index, uint8_t parity, const std::string& shm_name = "None");
            int initCameraSettingSync(std::string publish_topic = "/hikcamera/img_stream");
            int startSyncFrameGrab();
            int stopSyncFrameGrab();

            int setExposureTime(uint64_t exposure_time_ns) {
                offset_exposure_time_ = exposure_time_ns / 2;
                ROS_INFO("Set offset_exposure_time = %ld", offset_exposure_time_);
                return 0;
            };
            int setGprmcTransmitTime(uint8_t baudrate_index = BR115200) {
                int GPRMC_MAX_LENGTH = 66;
                if (baudrate_index < BRUnkown) {
                    constexpr int bitsPerCharacter = 10;
                    double timePerCharacterSeconds = static_cast<double>(bitsPerCharacter) / baudRateValues[static_cast<int>(baudrate_index)];
                    double maxTransmissionTimeSeconds = GPRMC_MAX_LENGTH * timePerCharacterSeconds;
                    auto maxTransmissionTimeNanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::duration<double>(maxTransmissionTimeSeconds)
                    );
                    offset_serial_time_ = maxTransmissionTimeNanoseconds;
                    ROS_INFO("Set offset_serial_time = %ld", offset_serial_time_.count());
                    return 0;
                } else {
                    ROS_ERROR("Invalid baudrate index!");
                    return 1;
                }
            }


        private:

            uint32_t freq_;
            NS ideal_interval_;
            uint64_t offset_exposure_time_;

            std::shared_ptr<ShmHandler<TIME_STAMP>> shm_;

            TimeSync *timesync_;
            TimeSyncConfig timesync_config_;
            std::mutex config_mutex_;
            NS offset_serial_time_;


            std::deque<FramePacket> queue_;
            uint32_t seq_;
            uint32_t base_seq_;
            uint64_t base_time_stamp_;
            uint64_t gps_time_stamp_;
            uint64_t last_gps_time_stamp_;
            volatile bool is_gps_update_;
            CLK::time_point base_time_;
            CLK::time_point gps_rcv_time_;
            CLK::time_point last_gps_rcv_time_;
            CLK::time_point frame_rcv_time_;
            double stamp_scale_;
            std::mutex sync_mtx_;
            volatile bool sync_flag_;

            std::shared_ptr<std::thread> sync_status_query_wt_;
            volatile bool exit_sync_status_query_wt_;
            volatile bool start_sync_status_query_wt_;

            std::shared_ptr<std::thread> get_frame_wt_;
            volatile bool exit_get_frame_wt_;
            volatile bool start_get_frame_wt_;

            std::shared_ptr<std::thread> queue_process_wt_;
            volatile bool exit_queue_process_wt_;
            volatile bool start_queue_process_wt_;

            image_transport::Publisher pub_handler_;
            PublishCb pub_cb_;
            void *client_data_;

            std::mutex queue_mtx_;
            std::condition_variable queue_cond_var_;

            void initValuable();

            int32_t SetPublishCb(PublishCb cb, void *data) {
            if ((cb != nullptr) || (data != nullptr)) {
                    pub_cb_ = cb;
                    client_data_ = data;
                    return 0;
                } else {
                    return -1;
                }
            }

            static void ReceiveSyncTimeCallback(uint64_t gps_time, 
                                                CLK::time_point gps_rcv_time, 
                                                void *client_data);
            void SyncStatusQueryLoop();
            
            void GetFrameWorkThread();
            void QueueProcessWorkThread();
            static void PublishCallBack(FramePacket frame_pkg, uint64_t time_stamp, void* client_data);
    };
}




#endif 
