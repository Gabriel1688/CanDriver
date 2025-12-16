// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once


#include <stdexcept>
#include <string>
#include "common.h"
#include <functional>
#include <map>
#include <pthread.h>
#include <atomic>
#include <mutex>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <memory>
#include <condition_variable>


namespace openarm::canbus {

// Base socket management class
class CANSocket_Ex {
public:
    explicit CANSocket_Ex(const std::string& interface);
    ~CANSocket_Ex();

    // Disable copy, enable move
    CANSocket_Ex(const CANSocket_Ex&) = delete;
    CANSocket_Ex& operator=(const CANSocket_Ex&) = delete;
    CANSocket_Ex(CANSocket_Ex&&) = default;
    CANSocket_Ex& operator=(CANSocket_Ex&&) = default;

    // File descriptor access for Python bindings
    int get_socket_fd() const { return socket_fd_; }
    const std::string& get_interface() const { return interface_; }
    bool is_initialized() const { return socket_fd_ >= 0; }

    // Direct frame operations for Python bindings
    ssize_t read_raw_frame(void* buffer, size_t buffer_size);
    ssize_t write_raw_frame(const void* buffer, size_t frame_size);

    // write can_frame or canfd_frame
    bool write_can_frame(const can_frame& frame);

    // read can_frame or canfd_frame
    bool read_can_frame(can_frame& frame);

    // check if data is available for reading (non-blocking)
    bool is_data_available(int timeout_us = 100);

protected:
    bool initialize_socket(const std::string& interface);
    void cleanup();
    void run();
    static void* EntryOfThread(void* argv);

    /*
     * Publish incomingPacketHandler client message to observer.
     * Observers get only messages that originated
     * from clients with IP address identical to
     * the specific observer requested IP
     */
    void handlereceivedMsg(const char * msg, size_t msgSize);
    std::string address;
    int port;
    int socket_fd_;
    std::string interface_;
    pthread_t thread_id;
    bool isConnected_;
};

}  // namespace openarm::canbus
