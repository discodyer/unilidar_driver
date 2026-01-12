/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#pragma once

#include <cstdint>
#include <vector>
#include <cstring>

#ifdef _WIN32
#include <winsock2.h>
#include <windows.h>
typedef int socklen_t;
#else
#include <arpa/inet.h>
#include <unistd.h>
#define closesocket close
#endif

/**
 * @brief UDP Handler
 */
class UDPHandler
{
public:
    UDPHandler(unsigned short port = 9000);
    virtual ~UDPHandler();

    int CreateSocket();
    void Close();
    int Bind();

    int Send(const char *buf, int size, char *ip, unsigned short port);
    int Recv(char *buf, int bufsize, sockaddr_in *from);

    int SetRecvTimeout(int sec);
    int SetSendTimeout(int sec);

private:
    int sockfd_ = 0;
    unsigned short udp_port_ = 0;
    
#ifdef _WIN32
    static bool wsa_initialized_;
    static int wsa_ref_count_;
#endif
};

/**
 * @brief Transform a Data Struct to UDP buffer
 *
 * @tparam DataStruct an ImuUnitree or ScanUnitree
 * @param data
 * @param buffer
 * @param msgType
 * @return uint32_t the total bytes sent through udp
 */
template <typename DataStruct>
uint32_t dataStructToUDPBuffer(const DataStruct &data, uint32_t msgType, char *buffer);
