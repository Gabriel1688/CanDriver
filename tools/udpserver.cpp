#include <iostream>
#include <string>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <netdb.h>
#include <sys/uio.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <fstream>
#include <map>
#include <array>
#include <utility>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <unistd.h>
#include "spdlog/spdlog.h"
#include "spdlog/fmt/ostr.h"

using namespace std;

std::map<int, std::array<uint8_t,13> > responses;
int main(int argc, char *argv[])
{
    //grab the port number
    int port = 1180;
    //setup a socket and connection tools
    sockaddr_in servAddr;
    bzero((char*)&servAddr, sizeof(servAddr));
    servAddr.sin_family = AF_INET;
    servAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servAddr.sin_port = htons(port);

    //open datagram oriented socket with internet address
    //also keep track of the socket descriptor
    int _sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(_sockfd < 0) {
        cerr << "Error establishing the server socket" << endl;
        exit(0);
    }
    /* Disable socket blocking */
    fcntl(_sockfd, F_SETFL, O_NONBLOCK);
    //bind the socket to its local address
    int bindStatus = bind(_sockfd, (struct sockaddr*) &servAddr, sizeof(servAddr));
    if(bindStatus < 0) {
        cerr << "Error binding socket to local address" << endl;
        exit(0);
    }
    cout << "Waiting for a client to connect..." << endl;
    //client address
    sockaddr_in clientAddr;
    socklen_t clientAddrSize = sizeof(clientAddr);
    //also keep track of the amount of data sent as well
    uint8_t response[]={0x11, 0x70, 0x81, 0x7F, 0xF8, 0x01, 0x1C, 0x1B};
    responses.insert( std::make_pair<int, std::array<uint8_t,13>>(0xFC,{0x08, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC}));
    responses.insert( std::make_pair<int, std::array<uint8_t,13>>(0xFD,{0x08, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD}));
    responses.insert( std::make_pair<int, std::array<uint8_t,13>>(0xFE,{0x08, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}));
    responses.insert( std::make_pair<int, std::array<uint8_t,13>>(0x7FF,{0x08, 0x00, 0x00, 0x07, 0xFF, 0x00, 0x00, 0x33, 0x07, 0x00, 0x00, 0x00, 0x00}));
    responses.insert( std::make_pair<int, std::array<uint8_t,13>>(0xCC,{0x08, 0x00, 0x00, 0x07, 0xFF, 0x00, 0x00, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00}));
    responses.insert( std::make_pair<int, std::array<uint8_t,13>>(0xFF, {0x08, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0x7F, 0xF0, 0x10, 0x33, 0x37, 0xFF}));
    responses.insert( std::make_pair<int, std::array<uint8_t,13>>(0xFF, {0x08, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x08, 0x13}));
    responses.insert( std::make_pair<int, std::array<uint8_t,13>>(0xFF, {0x08, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x08, 0x13}));

    /* Initialize variables for epoll */
    struct epoll_event ev;
    int epfd = epoll_create(2);
    ev.data.fd = _sockfd;
    ev.events = EPOLLIN;
    int ret = epoll_ctl(epfd, EPOLL_CTL_ADD, _sockfd , &ev);
    struct epoll_event events[2];
    bool _isConnected = true;
    while(_isConnected) {
            int ready = epoll_wait(epfd, events, 2, -1);  //20 milliseconds
            if (ready < 0)  {
              perror("epoll_wait error.");
              exit(0);
            }
            else if (ready == 0) {
              /* timeout, no data coming */
              continue;
            }
            else {
              for (int i = 0; i < ready; i++)   {
                if (events[i].data.fd == _sockfd)  {
                  uint8_t msg[13];
                  memset(msg,0,13);

                  //handle the new connection with client address
                  const size_t numOfBytesReceived = recvfrom(_sockfd, msg, 13, 0,(struct sockaddr *) &clientAddr, (socklen_t *) &clientAddrSize);
                  if (numOfBytesReceived < 1) {
                    std::string errorMsg;
                    if (numOfBytesReceived == 0) {
                      errorMsg = "Server closed connection";
                    } else {
                      errorMsg = strerror(errno);
                    }
                    _isConnected = false;
                  } else {
                    spdlog::info("------> {0:02x} ",fmt::join(msg, " "));

                    responses.insert( std::make_pair<int, std::array<uint8_t,13>>(0xFC,{0x08, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC}));
                    responses.insert( std::make_pair<int, std::array<uint8_t,13>>(0xFD,{0x08, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD}));
                    responses.insert( std::make_pair<int, std::array<uint8_t,13>>(0xFE,{0x08, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}));
                    responses.insert( std::make_pair<int, std::array<uint8_t,13>>(0x33,{0x08, 0x00, 0x00, 0x07, 0xFF, 0x00, 0x00, 0x33, 0x07, 0x00, 0x00, 0x00, 0x00}));
                    responses.insert( std::make_pair<int, std::array<uint8_t,13>>(0xCC,{0x08, 0x00, 0x00, 0x07, 0xFF, 0x00, 0x00, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00}));
                    responses.insert( std::make_pair<int, std::array<uint8_t,13>>(0xFF, {0x08, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0x7F, 0xF0, 0x10, 0x33, 0x37, 0xFF}));

                    unsigned int command = 0x0;
                    if( ( msg[12] == 0xFC ) || ( msg[12] == 0xFD ) || ( msg[12] == 0xFE ) ) {
                      command = msg[12];
                    }
                    else if (msg[7] == 0x33) {
                      command = 0x33;
                    }
                    else if (msg[7] == 0xCC){
                      command = 0xCC;
                    }
                    else if ( (msg[5] == 0x7F) && (msg[6] == 0xFF)){
                      command = 0xFF;
                    }

                    auto itmap = responses.find(command);
                    if(itmap != responses.end()) {
                      if( (command == 0xcc ) || (command == 0x33 )) {
                        itmap->second[05] = msg[5] + 0x10;  // low byte of canId
                      }
                      else {
                        itmap->second[04] = msg[4] + 0x10; // low byte of canId
                      }
                      sendto(_sockfd, reinterpret_cast<uint8_t*>(&itmap->second), 13, 0,(struct sockaddr*)&clientAddr, sizeof(struct sockaddr_in));
                      spdlog::info("<------ {0:02x}", fmt::join(itmap->second, " "));
                    }
                  }
                }
              }
            }
    }
    //we need to close the socket descriptors after we're all done
    close(_sockfd);
    cout << "Connection closed..." << endl;
    return 0;
}