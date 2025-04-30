#include "capancdt/proximity_connection.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <chrono>
#include <sstream>
#include <stdexcept>

namespace proximity_sensor
{
/*************************************
* ProximitySocketConnection
**************************************/

ProximitySocketConnection::ProximitySocketConnection(const std::string& ip, int port)
{
    this->ip = ip;
    this->port = port;
    this->sock = -1;
}

ProximitySocketConnection::~ProximitySocketConnection()
{
    if (this->sock != -1) {
        ::close(this->sock);
    }
}

void ProximitySocketConnection::connect()
{
    if (this->sock != -1) {
        throw SocketCreationError("Socket already connected");
    }

    struct sockaddr_in server;
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        throw SocketCreationError("Unable to create a socket");
    }

    // Setup server struct
    server.sin_family = AF_INET;
    server.sin_port = htons(this->port);

    // Covert the IP address from string to binary form
    if (inet_pton(AF_INET, this->ip.c_str(), &server.sin_addr) <= 0) {
        throw SocketCreationError("Invalid IP address format");
    }
    
    // Connect to the server
    if (::connect(sockfd, (struct sockaddr *)&server, sizeof(server)) < 0) {
        close(sockfd);
        throw SocketCreationError("Unable to connect to the proximity sensor");
    }

    // Switch to non-blocking mode
    int flags = fcntl(sockfd, F_GETFL, 0);

    if (flags < 0) {
        close(sockfd);
        throw SocketCreationError("Unable to set non-blocking mode");
    }

    if (fcntl(sockfd, F_SETFL, flags | O_NONBLOCK) < 0) {
        close(sockfd);
        throw SocketCreationError("Failed to set socket to non-blocking mode after connect");
    }

    this->sock = sockfd;
}

void ProximitySocketConnection::disconnect()
{
    if (this->sock != -1) {
        close(this->sock);
        this->sock = -1;
    }
}

ssize_t ProximitySocketConnection::recv(char* buffer, size_t size)
{
    if (this->sock == -1) {
        return 0;
    }
    return ::recv(this->sock, buffer, size, 0);
}

/*************************************
* ProximityMockConnection
**************************************/

void ProximityMockConnection::connect()
{
    if (this->connect_mock != nullptr) {
        this->connect_mock();
    }
}

void ProximityMockConnection::disconnect()
{
    // No action needed for mock
}

void ProximityMockConnection::set_recv_data(char* data, ssize_t data_size, int errno_override)
{
    std::lock_guard<std::mutex> guard(this->recv_lock);
    this->recv_data = data;
    this->recv_size = data_size;
    this->errno_override = errno_override;
}

ssize_t ProximityMockConnection::recv(char* buffer, size_t size)
{
    std::lock_guard<std::mutex> guard(this->recv_lock);
    errno = this->errno_override;

    if (this->recv_size > 0) {
        memcpy(buffer, this->recv_data, this->recv_size);
    }
    
    return this->recv_size;
}

} // namespace proximity_sensor