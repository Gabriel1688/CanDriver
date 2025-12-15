#pragma once

#include <string>
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

struct client_observer_t {
    int id = 0;
    std::function<void(const char * msg, size_t size)> incomingPacketHandler = nullptr;
    std::function<void(const std::string & ret)> disconnectionHandler = nullptr;
    bool completed=false;
};
struct CanFrame {
    unsigned char data[8] = {0};
    int length;
};

class TcpClient
{
private:
    int _sockfd;
    std::atomic<bool> _isConnected;
    std::atomic<bool> _isClosed;
    std::mutex _subscribersMtx;
    std::map<int32_t, client_observer_t> _subscribers;
    pthread_t thread_id;
    std::map<int, std::vector<CanFrame> > receivedCanFrame;

    void run();
    static void* EntryOfThread(void* argv);
    void publishServerMsg(const char * msg, size_t msgSize);
    void handlereceivedMsg(const char * msg, size_t msgSize);
    void publishServerDisconnected(const std::string& ret);

public:
    TcpClient();
    ~TcpClient();
    void Start();
    bool connectTo(const std::string & address, int port);

    /*
     * Sends a CAN message.
     *
     * @param[in] messageID the CAN ID to send
     * @param[in] data      the data to send (0-8 bytes)
     * @param[in] dataSize  the size of the data to send (0-8 bytes)
     * @param[in] response  waiting for the response
     * @param[out] status   Error status variable. 0 on success.
     */
    void sendMsg(uint32_t messageID, const uint8_t* data, uint8_t dataSize, bool response,  int32_t* status);

    /*
     * Get data from remote CAN device.
     *
     * @param[in] messageID the CAN ID to send
     * @param[in] data      the data to send (0-8 bytes)
     * @param[in] dataSize  the size of the data to send (0-8 bytes)
     * @param[out] recvBuf   the data received from remote device (0-8 bytes)
     * @param[out] recvDataSize  the size of the data to send (0-8 bytes)
     * @param[out] status   Error status variable. 0 on success.
     */
    bool receiveMsg(uint32_t messageID, const uint8_t* data, uint8_t dataSize, uint8_t* recvBuf, uint8_t recvDataSize, int32_t* status);
    void subscribe(const int32_t deviceId, const client_observer_t & observer);
    bool isConnected() const { return _isConnected; }
    bool close();
};
