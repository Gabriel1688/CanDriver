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

struct client_observer_t {
    int id = 0;
    std::function<void(const can_frame& frame)> incomingPacketHandler = nullptr;
    std::function<void(const std::string & ret)> disconnectionHandler = nullptr;
    bool completed=false;
};
// Base socket management class
class CANSocket {
public:
    explicit CANSocket(const std::string& interface);
    ~CANSocket();

    // Disable copy, enable move
    CANSocket(const CANSocket&) = delete;
    CANSocket& operator=(const CANSocket&) = delete;
    CANSocket(CANSocket&&) = default;
    CANSocket& operator=(CANSocket&&) = default;

    // File descriptor access for Python bindings
    int get_socket_fd() const { return socket_fd_; }
    const std::string& get_interface() const { return interface_; }
    bool is_initialized() const { return socket_fd_ >= 0; }

    // Direct frame operations for Python bindings
    ssize_t read_raw_frame(void* buffer, size_t buffer_size);
    ssize_t write_raw_frame(const void* buffer, size_t frame_size);

    // write can_frame or canfd_frame
    bool write_can_frame(can_frame_ex& frame);

    // read can_frame or canfd_frame
    bool read_can_frame(can_frame_ex& frame);

    // check if data is available for reading (non-blocking)
    bool is_data_available(int timeout_us = 100);
    void subscribe(const int32_t deviceId, const client_observer_t & observer);
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
    void handlereceivedMsg(const can_frame_ex& frame, size_t msgSize);
    std::string address;
    int port;
    int socket_fd_;
    std::string interface_;
    pthread_t thread_id;
    bool isConnected_;
    std::mutex _subscribersMtx;
    std::map<int32_t, client_observer_t> _subscribers;
};

}  // namespace openarm::canbus
