//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "timesync.h"

#include <stdint.h>
#include <string.h>
#include <chrono>
#include <cstdio>
#include <functional>
#include <thread>
#include <iostream>
#include <vector>
#include <sstream>

namespace livox_ros {
using namespace std;

TimeSync::TimeSync()
    : exit_poll_state_(false),
      start_poll_state_(false),
      exit_poll_data_(false),
      start_poll_data_(false) {
  fsm_state_ = kOpenDev;
  uart_ = nullptr;
  comm_ = nullptr;
  fn_cb_ = nullptr;
  client_data_ = nullptr;
  rx_bytes_ = 0;
}

TimeSync::~TimeSync() { DeInitTimeSync(); }

int32_t TimeSync::InitTimeSync(const TimeSyncConfig &config) {
  config_ = config;

  if (config_.dev_config.type == kCommDevUart) {
    uint8_t baudrate_index = config_.dev_config.config.uart.baudrate;
    uint8_t parity_index = config_.dev_config.config.uart.parity;

    if ((baudrate_index < BRUnkown) && (parity_index < ParityUnkown)) {
      uart_ = new UserUart(baudrate_index, parity_index);
    } else {
      printf("Uart parameter error, please check the configuration file!\n");
      return -1;
    }
  } else {
    printf("Device type not supported, now only uart is supported!\n");
    return -1;
  }

  config_.protocol_config.type = kGps;
  comm_ = new CommProtocol(config_.protocol_config);

  t_poll_state_ =
      std::make_shared<std::thread>(std::bind(&TimeSync::PollStateLoop, this));
  t_poll_data_ =
      std::make_shared<std::thread>(std::bind(&TimeSync::PollDataLoop, this));

  return 0;
}

int32_t TimeSync::DeInitTimeSync() {
  StopTimesync();

  if (uart_) delete uart_;
  if (comm_) delete comm_;

  fn_cb_ = nullptr;
  client_data_ = nullptr;
  return 0;
}

void TimeSync::StopTimesync() {
  start_poll_state_ = false;
  start_poll_data_ = false;
  exit_poll_state_ = true;
  exit_poll_data_ = true;
  if (t_poll_state_) {
    t_poll_state_->join();
    t_poll_state_ = nullptr;
  }

  if (t_poll_data_) {
    t_poll_data_->join();
    t_poll_data_ = nullptr;
  }
}

void TimeSync::PollStateLoop() {
  while (!start_poll_state_) {
    /* waiting to start */
  }

  while (!exit_poll_state_) {
    if (fsm_state_ == kOpenDev) {
      FsmOpenDev();
    } else if (fsm_state_ == kPrepareDev) {
      FsmPrepareDev();
    } else if (fsm_state_ == kCheckDevState) {
      FsmCheckDevState();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}


uint64_t TimeSync::ProcessRMCData(const char* rmc, uint32_t rmc_length) {
    std::string rmc_data(rmc, rmc_length);

    printf("] received GPRMC data: %s\n", rmc_data.c_str());

    std::vector<std::string> tokens;
    std::string token;
    std::stringstream tokenStream(rmc_data);
    while (std::getline(tokenStream, token, ',')) {
        tokens.push_back(token);
    }

    if (tokens.size() < 10) {
        std::cerr << "Invalid RMC data format." << std::endl;
        return 1;
    }

    std::string time_str = tokens[1];
    std::string date_str = tokens[9];

    
    try {
        int hours = stoi(time_str.substr(0, 2));
        int minutes = stoi(time_str.substr(2, 2));
        int seconds = stoi(time_str.substr(4, 2));
        int milliseconds = 0;
        
        if (time_str.size() > 6) {
            milliseconds = stoi(time_str.substr(7, 2)) * 10; 
        }

        int day = stoi(date_str.substr(0, 2));
        int month = stoi(date_str.substr(2, 2));
        int year = stoi(date_str.substr(4, 2)) + 2000; 
        
        std::tm t = {};
        t.tm_year = year - 1900; 
        t.tm_mon = month - 1;    
        t.tm_mday = day;
        t.tm_hour = hours;
        t.tm_min = minutes;
        t.tm_sec = seconds;

        std::time_t time_epoch = timegm(&t);

        if (time_epoch == -1) {
            std::cerr << "Failed to convert time to epoch." << std::endl;
            return 1;
        }

        uint64_t timestamp_ns = static_cast<uint64_t>(time_epoch) * 1000000000 + milliseconds * 1000000;
        
        return timestamp_ns;
    } catch (const std::invalid_argument& e) {
        std::cerr << "Invalid argument: " << e.what() << std::endl;
    } catch (const std::out_of_range& e) {
        std::cerr << "Out of range: " << e.what() << std::endl;
    }

    return 1;
}
void TimeSync::PollDataLoop() {
  while (!start_poll_data_) {
    /* waiting to start */
  }

  while (!exit_poll_data_) {
    if (uart_->IsOpen()) {
      uint32_t get_buf_size;
      uint8_t *cache_buf = comm_->FetchCacheFreeSpace(&get_buf_size);
      if (get_buf_size) {
        auto gps_rcv_time = std::chrono::high_resolution_clock::now();;
        uint32_t read_data_size;
        read_data_size = uart_->Read((char *)cache_buf, get_buf_size);
        if (read_data_size) {
          comm_->UpdateCacheWrIdx(read_data_size);
          rx_bytes_ += read_data_size;
          CommPacket packet;
          memset(&packet, 0, sizeof(packet));
          while ((kParseSuccess == comm_->ParseCommStream(&packet))) {
            if (((fn_cb_ != nullptr) || (client_data_ != nullptr))) {
              if ((strstr((const char *)packet.data, "$GPRMC")) ||
                      (strstr((const char *)packet.data , "$GNRMC"))){
                printf("At time [%ld", gps_rcv_time.time_since_epoch().count());
                uint64_t gps_time_ns = ProcessRMCData((const char *)packet.data, packet.data_len);
                if (gps_time_ns == 1) {
                  continue;
                }
                fn_cb_(gps_time_ns, gps_rcv_time, client_data_);
                printf("RMC data parse success!. Gps_time_ns [%ld]\n", gps_time_ns);
              }
            }
          }
        }
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
}

void TimeSync::FsmTransferState(uint8_t new_state) {
  if (new_state < kFsmDevUndef) {
    fsm_state_ = new_state;
  }
  transfer_time_ = chrono::steady_clock::now();
}

void TimeSync::FsmOpenDev() {
  if (!uart_->IsOpen()) {
    if (!uart_->Open(config_.dev_config.name)) {
      FsmTransferState(kPrepareDev);
    }
  } else {
    FsmTransferState(kPrepareDev);
  }
}

void TimeSync::FsmPrepareDev() {
  chrono::steady_clock::time_point t = chrono::steady_clock::now();
  chrono::milliseconds time_gap =
      chrono::duration_cast<chrono::milliseconds>(t - transfer_time_);
  /** delay some time when device is opened, 4s */
  if (time_gap.count() > 3000) {
    FsmTransferState(kCheckDevState);
  }
}

void TimeSync::FsmCheckDevState() {
  static uint32_t last_rx_bytes = 0;
  static chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::milliseconds time_gap =
      chrono::duration_cast<chrono::milliseconds>(t2 - t1);

  if (time_gap.count() > 2000) { /* period : 2.5s */
    if (last_rx_bytes == rx_bytes_) {
      uart_->Close();
      FsmTransferState(kOpenDev);
      printf("Uart is disconnected, close it\n");
    }
    last_rx_bytes = rx_bytes_;
    t1 = t2;
  }
}

}  // namespace livox_ros
