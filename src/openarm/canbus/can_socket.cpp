#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <iostream>
#include <openarm/canbus/can_socket.hpp>
#include "spdlog/spdlog.h"

namespace openarm::canbus {

#define MAX_PACKET_SIZE 1500

CANSocket::CANSocket(const std::string& interface)
    : socket_fd_(-1), interface_(interface) {
    address="127.0.0.1";
    port = 1180;
    if (!initialize_socket(interface)) {
        throw std::runtime_error("Socket error: Failed to initialize socket for interface");
    }
}

CANSocket::~CANSocket() { cleanup(); }

bool CANSocket::initialize_socket(const std::string& interface) {
    // Create socket
    struct sockaddr_in _server;
    try {
        socket_fd_ = socket(AF_INET , SOCK_STREAM , 0);
        const int inetSuccess = inet_aton(address.c_str(), &_server.sin_addr);

        if(!inetSuccess) { // inet_addr failed to parse address
            // if hostname is not in IP strings and dots format, try resolve it
            struct hostent *host;
            struct in_addr **addrList;
            if ( (host = gethostbyname( address.c_str() ) ) == nullptr){
                throw std::runtime_error("Failed to resolve hostname");
            }
            addrList = (struct in_addr **) host->h_addr_list;
            _server.sin_addr = *addrList[0];
        }
        _server.sin_family = AF_INET;
        _server.sin_port = htons(port);
    } catch (const std::runtime_error& error) {
        std::cout << "server is already closed"<< error.what() << std::endl;
        return false;
    }

    const int connectResult = connect(socket_fd_ , (struct sockaddr *)&_server , sizeof(_server));
    if (connectResult ==  -1) {
        std::cout << "server is already closed, "<< strerror(errno) << std::endl;
        return false;
    }
    isConnected_ = true;
    if(pthread_create(&thread_id, nullptr, EntryOfThread,this) != 0) {
    }
//    _isClosed = false;
    return true;
}

/*static*/
void* CANSocket::EntryOfThread(void* argv)
{
    CANSocket* client = static_cast<CANSocket*>(argv);
    client->run();
    return (void*) client;
}

/*
 * Receive server packets, and notify observers
 */
void CANSocket::run() {
    /* Disable socket blocking */
    fcntl(socket_fd_, F_SETFL, O_NONBLOCK);

    /* Initialize variables for epoll */
    struct epoll_event ev;

    int epfd = epoll_create(255);
    ev.data.fd = socket_fd_;
    ev.events = EPOLLIN;
    epoll_ctl(epfd, EPOLL_CTL_ADD, socket_fd_ , &ev);

    struct epoll_event events[256];

    while (isConnected_)
    {
        int ready = epoll_wait(epfd, events, 256, 20);  //20 milliseconds
        if (ready < 0)
        {
            perror("epoll_wait error.");
            return ;
        }
        else if (ready == 0) {
            /* timeout, no data coming */
            continue;
        }
        else {
            for (int i = 0; i < ready; i++)
            {
                if (events[i].data.fd == socket_fd_)
                {
                    can_frame_ex frame;
                    ssize_t bytes_read = read(socket_fd_, &frame, sizeof(frame));
                    if (bytes_read < 1) {
                        std::string errorMsg;
                        if (bytes_read == 0) {
                            errorMsg = "Server closed connection";
                        } else {
                            errorMsg = strerror(errno);
                        }
                        isConnected_ = false;
                        //publishServerDisconnected(errorMsg);
                        return;
                    } else {
                        handlereceivedMsg(frame,bytes_read);
                    }
                }
            }
        }

    }
}

void CANSocket::subscribe(const int32_t deviceId, const client_observer_t & observer) {
    std::lock_guard<std::mutex> lock(_subscribersMtx);
    _subscribers.insert(std::make_pair(deviceId,observer));
}

void CANSocket::handlereceivedMsg(const can_frame_ex& msg, size_t msgSize) {
    can_frame frame;
    frame.can_id = __builtin_bswap32(msg.FrameId);
    frame.can_dlc = msgSize -5;  //substract header and frameId, get only the payload
    memcpy(frame.data,msg.data,frame.can_dlc);

    spdlog::info("------> {0:04x} : {1:02x}", frame.can_id, fmt::join(frame.data, " "));

    // call the subscribers for the notification.
    // TODO: currently, take frameId as can_id. it is a limitation to be resolved.
    //       only handle response with frameId = 0x7ff(response for para/refresh command)
    if( frame.can_id == 0x7ff) {
        frame.can_id = frame.data[0];  // data[0] is the low byte of CAN ID.
        auto it = _subscribers.find(frame.can_id);
        if(it != _subscribers.end()) {
            it->second.incomingPacketHandler(frame);
        }
    }
}

void CANSocket::cleanup() {
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
}

ssize_t CANSocket::read_raw_frame(void* buffer, size_t buffer_size) {
    return read(socket_fd_, buffer, buffer_size);
}

ssize_t CANSocket::write_raw_frame(const void* buffer, size_t frame_size) {
    return write(socket_fd_, buffer, frame_size);
}

bool CANSocket::write_can_frame(can_frame_ex& frame) {

    spdlog::info("<------ {0:04x} : {1:02x}", frame.FrameId, fmt::join(frame.data, " "));
    frame.FrameId = __builtin_bswap32(frame.FrameId);
    return write(socket_fd_, &frame, sizeof(frame)) == sizeof(frame);
}

bool CANSocket::read_can_frame(can_frame_ex& frame) {
    ssize_t bytes_read = read(socket_fd_, &frame, sizeof(frame));
    return bytes_read == sizeof(frame);
}

bool CANSocket::is_data_available(int timeout_us) {
    fd_set read_fds;
    struct timeval timeout;

    FD_ZERO(&read_fds);
    FD_SET(socket_fd_, &read_fds);

    timeout.tv_sec = timeout_us / 1000000;
    timeout.tv_usec = (timeout_us % 1000000);

    int result = select(socket_fd_ + 1, &read_fds, nullptr, nullptr, &timeout);

    return (result > 0 && FD_ISSET(socket_fd_, &read_fds));
}

}  // namespace openarm::canbus
