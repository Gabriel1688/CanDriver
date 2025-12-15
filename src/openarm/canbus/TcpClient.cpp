#include "openarm/canbus/TcpClient.h"

#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <iostream>

#define MAX_PACKET_SIZE 1500
std::condition_variable cv_completed;
std::mutex receivesMutex;
bool completed;


TcpClient::TcpClient() {
    _isConnected = false;
    _isClosed = true;
}

TcpClient::~TcpClient() {
    close();
}
void TcpClient::Start() {
    if(pthread_create(&thread_id, nullptr, EntryOfThread,this) != 0) {
    }
}

/*static*/
void* TcpClient::EntryOfThread(void* argv)
{
    TcpClient* client = static_cast<TcpClient*>(argv);
    client->run();
    return (void*) client;
}

bool TcpClient::connectTo(const std::string & address, int port) {
    struct sockaddr_in _server;
    try {
        _sockfd = socket(AF_INET , SOCK_STREAM , 0);

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

    const int connectResult = connect(_sockfd , (struct sockaddr *)&_server , sizeof(_server));
    if (connectResult ==  -1) {
        std::cout << "server is already closed, "<< strerror(errno) << std::endl;
        return false;
    }
    _isConnected = true;
    _isClosed = false;
    return true;
}

void TcpClient::sendMsg(uint32_t messageID, const uint8_t* data, uint8_t dataSize, bool response, int32_t* status) {
    //const char * msg, size_t size
    //Need to build message based on the messageId/data.
    const size_t numBytesSent = send(_sockfd, data, dataSize, 0);
    if (numBytesSent < 0 ) { // send failed
        std::cout << "client is already closed"<<strerror(errno) << std::endl;
    }

    if (numBytesSent < dataSize) { // not all bytes were sent
        char errorMsg[100];
        sprintf(errorMsg, "Only %lu bytes out of %lu was sent to server", numBytesSent, dataSize);
        std::cout << errorMsg << std::endl;
    }
    std::cout << "Send msg to server: " << std::endl;
    for(int i = 0 ; i< numBytesSent; i ++ )
    {
        std::cout << "data [" << i <<"] = " << std::showbase << std::hex  << (unsigned int) data[i] << std::endl;
    }
    //block to get response from Device
    if(response){
        std::unique_lock<std::mutex> lock(receivesMutex);
        completed = false;
    }
}

bool TcpClient::receiveMsg(uint32_t messageID, const uint8_t* data, uint8_t dataSize, uint8_t* recvBuf, uint8_t recvDataSize, int32_t* status) {
    //const char * msg, size_t size
    //Need to build message based on the messageId/data.
    const size_t numBytesSent = send(_sockfd, data, dataSize, 0);
    if (numBytesSent < 0 ) { // send failed
        std::cout << "client is already closed"<<strerror(errno) << std::endl;
    }

    if (numBytesSent < dataSize) { // not all bytes were sent
        char errorMsg[100];
        sprintf(errorMsg, "Only %lu bytes out of %lu was sent to server", numBytesSent, dataSize);
        std::cout << errorMsg << std::endl;
    }
    std::cout << "Send msg to server: " << std::endl;
    for(int i = 0 ; i< numBytesSent; i ++ )
    {
        std::cout << "data [" << i <<"] = " << std::showbase << std::hex  << (unsigned int) data[i] << std::endl;
    }
    //block to get response from Device
    {
        std::unique_lock<std::mutex> lock(receivesMutex);
        completed = false;
        bool result = cv_completed.wait_for(lock, std::chrono::milliseconds{20}, [] { return completed ; });
        if(result == true) {
            //fill the received data in recvBuf, set the recvDataSize
        }
        return result;

    }
}

void TcpClient::subscribe(const int32_t deviceId, const client_observer_t & observer) {
    std::lock_guard<std::mutex> lock(_subscribersMtx);
    _subscribers.insert(std::make_pair(deviceId,observer));
}

/*
 * Publish incomingPacketHandler client message to observer.
 * Observers get only messages that originated
 * from clients with IP address identical to
 * the specific observer requested IP
 */
void TcpClient:: handlereceivedMsg(const char * msg, size_t msgSize) {
    std::lock_guard<std::mutex> lock(_subscribersMtx);
    //parse the Can Id before sending to subscriber.
    //TODO:: need to define the frame structure.
    const int32_t deviceId = 1;

    std::map<int32_t, std::vector<CanFrame>>::iterator itmap = receivedCanFrame.find(deviceId);
    if(itmap != receivedCanFrame.end()) {
        CanFrame dataFrame;
        memcpy(dataFrame.data,msg,msgSize);
        dataFrame.length = msgSize;
        itmap->second.emplace_back(dataFrame);
    }
    // need to handle the notification which does not need to notify
    // if( response for the ge message)
    {
        std::unique_lock<std::mutex> lock(receivesMutex);
        completed = true;
        cv_completed.notify_one();
    }
    // else call the subscribers for the notification.
    auto it = _subscribers.find(deviceId);
    if(it != _subscribers.end()) {
        it->second.incomingPacketHandler(msg,msgSize);
    }
}

void TcpClient::publishServerMsg(const char * msg, size_t msgSize) {
    std::lock_guard<std::mutex> lock(_subscribersMtx);
    //parse the device Id before sending to subscriber.
    //TODO:: need to define the frame structure.
    const int32_t deviceId =1;
    std::map<int32_t, client_observer_t>::iterator itmap = _subscribers.find(deviceId);
    if(itmap != _subscribers.end()) {
        itmap->second.incomingPacketHandler(msg,msgSize);
    }

    {
        std::unique_lock<std::mutex> lock(receivesMutex);
        completed = true;
    }
    cv_completed.notify_one();
}

/*
 * Publish server disconnection to observer.
 * Observers get only notify about clients
 * with IP address identical to the specific
 * observer requested IP
 */
void TcpClient::publishServerDisconnected(const std::string & ret) {
    std::lock_guard<std::mutex> lock(_subscribersMtx);
    for (const auto &subscriber : _subscribers) {
        if (subscriber.second.disconnectionHandler) {
            subscriber.second.disconnectionHandler(ret);
        }
    }
}

/*
 * Receive server packets, and notify observers
 */
void TcpClient::run() {
    /* Disable socket blocking */
    fcntl(_sockfd, F_SETFL, O_NONBLOCK);

    /* Initialize variables for epoll */
    struct epoll_event ev;

    int epfd = epoll_create(255);
    ev.data.fd = _sockfd;
    ev.events = EPOLLIN;
    int ret = epoll_ctl(epfd, EPOLL_CTL_ADD, _sockfd , &ev);

    struct epoll_event events[256];

    while (_isConnected )
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
                if (events[i].data.fd == _sockfd)
                {
                    char msg[MAX_PACKET_SIZE];
                    memset(msg,0,MAX_PACKET_SIZE);
                    const size_t numOfBytesReceived = recv(_sockfd, msg, MAX_PACKET_SIZE, 0);
                    if (numOfBytesReceived < 1) {
                        std::string errorMsg;
                        if (numOfBytesReceived == 0) {
                            errorMsg = "Server closed connection";
                        } else {
                            errorMsg = strerror(errno);
                        }
                        _isConnected = false;
                        publishServerDisconnected(errorMsg);
                        return;
                    } else {
                        handlereceivedMsg(msg,numOfBytesReceived);
                    }
                }
            }
        }

    }
}

bool TcpClient::close(){
    if (_isClosed) {
        std::cout << "client is already closed" << std::endl;
        return false;
    }
    _isConnected = false;
    void *result;
    if (pthread_join(thread_id, &result) != 0) {
        perror("Failed to join thread 1");
        return false;
    }

    const bool closeFailed = (::close(_sockfd) == -1);
    if (closeFailed) {

        std::cout << "failed to close socket, error " << strerror(errno) << std::endl;
        return false;
    }
    _isClosed = true;
    return true;
}
