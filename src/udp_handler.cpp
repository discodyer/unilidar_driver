/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "udp_handler.h"
#include "unitree_lidar_protocol.h"
#include <stdio.h>

#ifdef _WIN32
#include <winsock2.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/time.h>
#define SOCKET int
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#endif

#ifdef _WIN32
// Static members for WSA initialization
bool UDPHandler::wsa_initialized_ = false;
int UDPHandler::wsa_ref_count_ = 0;
#endif

// Constructor
UDPHandler::UDPHandler(unsigned short port)
    : sockfd_(0), udp_port_(port)
{
}

// Destructor
UDPHandler::~UDPHandler()
{
    Close();
}

// CreateSocket
int UDPHandler::CreateSocket()
{
#ifdef _WIN32
    // Initialize Winsock if not already initialized
    if (!wsa_initialized_)
    {
        WSADATA wsaData;
        int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
        if (result != 0)
        {
            printf("[UDPHandler] WSAStartup failed with error: %d\n", result);
            return -1;
        }
        wsa_initialized_ = true;
        printf("[UDPHandler] Winsock initialized successfully.\n");
    }
    wsa_ref_count_++;
#endif

    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    
#ifdef _WIN32
    if (sockfd_ == (int)INVALID_SOCKET)
    {
        int error = WSAGetLastError();
        printf("[UDPHandler] create udp socket failed. WSA Error: %d\n", error);
        return -1;
    }
#else
    if (sockfd_ == -1)
    {
        printf("[UDPHandler] create udp socket failed.\n");
        return -1;
    }
#endif
    
    printf("[UDPHandler] create udp socket success.\n");
    return 0;
}

// Close
void UDPHandler::Close()
{
#ifdef _WIN32
    if (sockfd_ != 0 && sockfd_ != (int)INVALID_SOCKET)
    {
        closesocket(sockfd_);
        sockfd_ = 0;
        udp_port_ = 0;
        
        // Decrement reference count and cleanup WSA if this is the last instance
        wsa_ref_count_--;
        if (wsa_ref_count_ <= 0 && wsa_initialized_)
        {
            WSACleanup();
            wsa_initialized_ = false;
            wsa_ref_count_ = 0;
            printf("[UDPHandler] Winsock cleaned up.\n");
        }
    }
#else
    if (sockfd_ > 0)
    {
        closesocket(sockfd_);
        sockfd_ = 0;
        udp_port_ = 0;
    }
#endif
}

// Bind
int UDPHandler::Bind()
{
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    
    addr.sin_family = AF_INET;
    addr.sin_port = htons(udp_port_);
    addr.sin_addr.s_addr = INADDR_ANY;
    
    int ret = bind(sockfd_, (struct sockaddr*)&addr, sizeof(addr));
    
    if (ret == 0)
    {
        printf("[UDPHandler] bind udp port success. port %d.\n", udp_port_);
        return 0;
    }
    else
    {
#ifdef _WIN32
        int error = WSAGetLastError();
        printf("[UDPHandler] bind udp port failed. WSA Error: %d\n", error);
#else
        printf("[UDPHandler] bind udp port failed.\n");
#endif
        return -1;
    }
}

// Send
int UDPHandler::Send(const char *buf, int size, char *ip, unsigned short port)
{
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = inet_addr(ip);
    
    int sent = 0;
    while (sent < size)
    {
        int ret = sendto(sockfd_, buf + sent, size - sent, 0, 
                        (struct sockaddr*)&addr, sizeof(addr));
        if (ret < 1)
        {
#ifdef _WIN32
            int error = WSAGetLastError();
            printf("[UDPHandler] sendto failed. WSA Error: %d\n", error);
#endif
            break;
        }
        sent += ret;
    }
    
    return sent;
}

// Recv
int UDPHandler::Recv(char *buf, int bufsize, sockaddr_in *from)
{
    if (from != nullptr) {
        socklen_t fromlen = sizeof(sockaddr_in);
        int ret = recvfrom(sockfd_, buf, bufsize, 0, 
                          (struct sockaddr*)from, &fromlen);
        
#ifdef _WIN32
        if (ret < 0) {
            int error = WSAGetLastError();
            if (error != WSAETIMEDOUT && error != WSAEWOULDBLOCK) {
                printf("[UDPHandler] recvfrom failed. WSA Error: %d\n", error);
            }
        }
#endif
        
        return ret;
    } else {
        // When from is nullptr, pass nullptr for both address and length
        int ret = recvfrom(sockfd_, buf, bufsize, 0, nullptr, nullptr);
        
#ifdef _WIN32
        if (ret < 0) {
            int error = WSAGetLastError();
            if (error != WSAETIMEDOUT && error != WSAEWOULDBLOCK) {
                printf("[UDPHandler] recvfrom failed. WSA Error: %d\n", error);
            }
        }
#endif
        
        return ret;
    }
}

// SetRecvTimeout
int UDPHandler::SetRecvTimeout(int sec)
{
#ifdef _WIN32
    DWORD timeout = sec * 1000;  // Convert to milliseconds
    int ret = setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, 
                        (const char*)&timeout, sizeof(timeout));
#else
    struct timeval timeout;
    timeout.tv_sec = sec;
    timeout.tv_usec = 0;
    int ret = setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, 
                        &timeout, sizeof(timeout));
#endif
    
    if (ret < 0)
    {
        printf("[UDPHandler] set udp recv timeout failed.");
        return -1;
    }
    
    printf("[UDPHandler] set udp recv timeout success. %d sec.\n", sec);
    return 0;
}

// SetSendTimeout
int UDPHandler::SetSendTimeout(int sec)
{
#ifdef _WIN32
    DWORD timeout = sec * 1000;  // Convert to milliseconds
    int ret = setsockopt(sockfd_, SOL_SOCKET, SO_SNDTIMEO, 
                        (const char*)&timeout, sizeof(timeout));
#else
    struct timeval timeout;
    timeout.tv_sec = sec;
    timeout.tv_usec = 0;
    int ret = setsockopt(sockfd_, SOL_SOCKET, SO_SNDTIMEO, 
                        &timeout, sizeof(timeout));
#endif
    
    if (ret < 0)
    {
        printf("[UDPHandler] set udp send timeout failed.\n");
        return -1;
    }
    
    printf("[UDPHandler] set udp send timeout success.\n");
    return 0;
}

// Template specialization for LidarImuData
template <>
uint32_t dataStructToUDPBuffer(
    const unilidar_sdk2::LidarImuData &data, uint32_t msgType, char *buffer)
{
    // Write header: msgType (4 bytes) + payload size (4 bytes)
    uint32_t *header = (uint32_t*)buffer;
    header[0] = msgType;
    header[1] = sizeof(unilidar_sdk2::LidarImuData);  // 56 bytes
    
    // Copy the LidarImuData structure
    memcpy(buffer + 8, &data, sizeof(unilidar_sdk2::LidarImuData));
    
    // Return total buffer size
    return 64;  // 8 bytes header + 56 bytes data
}
