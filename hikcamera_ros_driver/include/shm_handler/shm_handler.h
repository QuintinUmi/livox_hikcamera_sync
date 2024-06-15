#ifndef _SHM_HANDLER_H_
#define _SHM_HANDLER_H_

#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <chrono>

#include <functional>
#include <thread>
#include <atomic>
#include <cstdint>
#include <sys/mman.h>
#include <cstring>
#include <iostream>


namespace shm_handler
{
    enum class RWFLAG{
        SHM_READ,
        SHM_WRITE
    };


    template<typename T>
    class ShmHandler {
        
        public:
 
            using ShmUpdateCallback = std::function<void(const T&)>;

            ShmHandler(const std::string& shm_name, RWFLAG rw_flag, ShmUpdateCallback cb = [](const T&) {});
            ~ShmHandler();

            void write(const T& timestamp);
            T read();

        private:
            void queryShmPoll();

            std::string shm_name_;
            RWFLAG rw_flag_;
            int shm_fd_;
            T* shm_ptr_;
            std::thread polling_thread_;
            std::atomic<bool> running_;
            ShmUpdateCallback update_cb_;
    };

}

#include "shm_handler.tpp"


#endif