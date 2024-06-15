#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <chrono>

#include "shm_handler.h"

using namespace shm_handler;

template<typename T>
ShmHandler<T>::ShmHandler(const std::string& shm_name, RWFLAG rw_flag, ShmUpdateCallback update_cb)
    : shm_name_(shm_name), rw_flag_(rw_flag), shm_fd_(-1), shm_ptr_(nullptr), running_(true), update_cb_(update_cb) {
    shm_fd_ = shm_open(shm_name_.c_str(), O_CREAT | O_RDWR, 0666);
    if (shm_fd_ == -1) {
        perror("shm_open");
        throw std::runtime_error("Failed to open shared memory");
    }

    if (ftruncate(shm_fd_, sizeof(T)) == -1) {
        perror("ftruncate");
        throw std::runtime_error("Failed to set size of shared memory");
    }

    shm_ptr_ = static_cast<T*>(mmap(nullptr, sizeof(T), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0));
    if (shm_ptr_ == MAP_FAILED) {
        perror("mmap");
        throw std::runtime_error("Failed to map shared memory");
    }

    if (rw_flag_  == RWFLAG::SHM_READ) {
        polling_thread_ = std::thread(&ShmHandler::poll, this);
    }
}

template<typename T>
ShmHandler<T>::~ShmHandler() {
    running_ = false;
    if (polling_thread_.joinable()) {
        polling_thread_.join();
    }

    if (munmap(shm_ptr_, sizeof(T)) == -1) {
        perror("munmap");
    }

    if (close(shm_fd_) == -1) {
        perror("close");
    }

    if (shm_unlink(shm_name_.c_str()) == -1) {
        perror("shm_unlink");
    }
}

template<typename T>
void ShmHandler<T>::write(const T& data) {
    if (rw_flag_ != RWFLAG::SHM_WRITE) {
        throw std::runtime_error("Instance is not a writer");
    }
    std::memcpy(shm_ptr_, &data, sizeof(T));
}

template<typename T>
T ShmHandler<T>::read() {
    if (rw_flag_ != RWFLAG::SHM_READ) {
        throw std::runtime_error("Instance is not a reader");
    }
    T data;
    std::memcpy(&data, shm_ptr_, sizeof(T));
    return data;
}

template<typename T>
void ShmHandler<T>::queryShmPoll() {
    T last_data = *shm_ptr_;
    while (running_) {
        if (std::memcmp(&last_data, shm_ptr_, sizeof(T)) != 0) {
            last_data = *shm_ptr_;
            callback_(last_data);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 高速轮询，每10毫秒检查一次
    }
}

