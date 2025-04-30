#include <gtest/gtest.h>
#include <memory>
#include <thread>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include "capancdt/proximity_connection.h"

using namespace proximity_sensor;

struct ProximityMockConnectionTest: ::testing::Test {
    ProximityMockConnection conn;
};

TEST_F(ProximityMockConnectionTest, MockConnect) {
    EXPECT_NO_THROW(
        conn.connect()
    ) << "Connection should not throw an exception";
}

TEST_F(ProximityMockConnectionTest, MockDisconnect) {
    EXPECT_NO_THROW(
        conn.disconnect()
    ) << "Disconnection should not throw an exception";
}

TEST_F(ProximityMockConnectionTest, MockRecv) {
    char data[15] = {0};
    data[0] = '1';
    data[1] = '2';
    data[2] = '3';
    data[3] = 'x'; // not included in recv

    // set '1', '2', '3' as recv data
    conn.set_recv_data(data, 3);

    char buffer[15] = {0};
    EXPECT_EQ(conn.recv(buffer, sizeof(buffer)), 3) << "recv() should return 3 bytes received";
    EXPECT_EQ(buffer[0], '1') << "recv() should set the first byte";
    EXPECT_EQ(buffer[1], '2') << "recv() should set the second byte";
    EXPECT_EQ(buffer[2], '3') << "recv() should set the third byte";
    EXPECT_EQ(buffer[3], '\0') << "recv() should not affect the rest of the buffer";

    data[0] = 'a';
    data[1] = 'b';
    data[2] = 'c'; // not included in recv

    // Set 'a' and 'b' as new recv data
    conn.set_recv_data(data, 2);

    EXPECT_EQ(conn.recv(buffer, sizeof(buffer)), 2) << "recv() should return 2 bytes received";
    EXPECT_EQ(buffer[0], 'a') << "recv() should set the first byte";
    EXPECT_EQ(buffer[1], 'b') << "recv() should set the second byte";
    EXPECT_EQ(buffer[2], '3') << "Third byte is unchanged from first recv()";
    EXPECT_EQ(buffer[3], '\0') << "Fourth byte is unchanged from first recv()";
}

TEST_F(ProximityMockConnectionTest, MockRecvError) {
    char data[15] = {0};
    data[0] = 'x';
    conn.set_recv_data(data, 1);

    conn.set_recv_data(nullptr, -1, EAGAIN);

    char buffer[15] = {0};
    EXPECT_EQ(conn.recv(buffer, sizeof(buffer)), -1) << "recv() should return -1 on error";
    EXPECT_EQ(errno, EAGAIN) << "recv() should set errno to EAGAIN";
    EXPECT_EQ(buffer[0], '\0') << "recv() should return none for the buffer";
}

// Create a server stub that listens on an ephermal port (assigned to the passed port variable)
int create_server_stub(uint16_t& port) {
    int fd = ::socket(AF_INET, SOCK_STREAM, 0);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0; // ephemeral port

    if (::bind(fd, (sockaddr*)&addr, sizeof(addr)) != 0) {
        close(fd);
        return -1;
    }
  
    socklen_t len = sizeof(addr);
    if (::getsockname(fd, (sockaddr*)&addr, &len) != 0) {
        close(fd);
        return -1;
    }
    port = ntohs(addr.sin_port);
  
    if (::listen(fd, 1) != 0) {
        close(fd);
        return -1;
    }
    return fd;
}

// Accepts one connection, writes <data,len>, then closes both sides.
// Optionally sleep <time_sleep_ms> before sending data after accepting the connection
void server_stub_write_and_close(int listen_fd, const char* data, size_t len, int time_sleep_ms = 0) {
    // Accept one connection on the listening socket
    sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    int client_fd = ::accept(listen_fd, (sockaddr*)&client_addr, &client_len);

    // If accept fails, nothing to do
    if (client_fd < 0) {
        close(listen_fd);
        return;
    }

    if (time_sleep_ms > 0.0)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(time_sleep_ms));
    }

    // Write all bytes
    ssize_t written = 0;
    while (written < (ssize_t)len) {
        ssize_t n = ::write(client_fd, data + written, len - written);
        if (n <= 0) break;
        written += n;
    }

    // Close client side and listen socket
    close(client_fd);
    close(listen_fd);
}

// Accepts one connection, then closes immediately.
void server_stub_close_immediately(int listen_fd) {
    sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);

    int client_fd = ::accept(listen_fd, (sockaddr*)&client_addr, &client_len);
    if (client_fd >= 0) close(client_fd);
    
    close(listen_fd);
}

// Test: we send "HELLO" and expect to recv exactly 5 bytes.
TEST(ProximitySocketConnectionTest, RecvData) {
    uint16_t port;
    int listen_fd = create_server_stub(port);
    ASSERT_GE(listen_fd, 0) << "failed to create listener";

    // Start server (pass by value [=])
    static constexpr char PAYLOAD[] = "HELLO";
    std::thread server([=] {
        server_stub_write_and_close(listen_fd, PAYLOAD, sizeof(PAYLOAD)-1);
    });
    
    // Create client and connect
    ProximitySocketConnection client("127.0.0.1", port);
    EXPECT_NO_THROW(client.connect());
    // Wait a second to ensure the data is sent from the server after accepting the connection
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Recv data
    char buf[16] = {};
    ssize_t n = client.recv(buf, sizeof(buf));
    EXPECT_EQ(n, (ssize_t)(sizeof(PAYLOAD)-1));
    EXPECT_STREQ(buf, "HELLO");
    EXPECT_NO_THROW(client.disconnect());

    // Wait for server to finish
    server.join();
}

// Test: The socket is non-blocking and we get EAGAIN errno when no data is available to recv
TEST(ProximitySocketConnectionTest, RecvDataWhenNoDataAvailable) {
    uint16_t port;
    int listen_fd = create_server_stub(port);
    ASSERT_GE(listen_fd, 0) << "failed to create listener";

    // Start server (pass by value [=])
    static constexpr char PAYLOAD[] = "HELLO";
    std::thread server([=] {
        // server accepts connection and sleeps for 1 second before sending data
        server_stub_write_and_close(listen_fd, PAYLOAD, sizeof(PAYLOAD)-1, 1000);
    });
    
    // Create client and connect
    ProximitySocketConnection client("127.0.0.1", port);
    EXPECT_NO_THROW(client.connect());

    // Recv data
    char buf[16] = {};
    ssize_t n = client.recv(buf, sizeof(buf));
    EXPECT_EQ(n, -1);
    EXPECT_EQ(errno, EAGAIN) << "recv() should return EAGAIN when no data is available";
    EXPECT_NO_THROW(client.disconnect());

    // Wait for server to finish
    server.join();
}

// Test: server accepts then closes immediately → recv() should return 0 (EOF).
TEST(ProximitySocketConnectionTest, RecvEof) {
    uint16_t port;
    int listen_fd = create_server_stub(port);
    ASSERT_GE(listen_fd, 0) << "failed to create listener";

    // Start server (pass by value [=])
    std::thread server([=] {
        server_stub_close_immediately(listen_fd);
    });

    // Create client and connect
    ProximitySocketConnection client("127.0.0.1", port);
    EXPECT_NO_THROW(client.connect());
    // Wait a second to ensure the data is sent from the server after accepting the connection
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Recv data
    char buf[8];
    ssize_t n = client.recv(buf, sizeof(buf));
    EXPECT_EQ(n, 0) << "EOF should be signaled by 0 bytes read";
    EXPECT_NO_THROW(client.disconnect());

    // Wait for server to finish
    server.join();
}

// Test: connecting to a non‑listening port throws SocketCreationError
TEST(ProximitySocketConnectionTest, ConnectFail) {
    // Using invalid port
    ProximitySocketConnection client("127.0.0.1", 54321);
    EXPECT_THROW(client.connect(), SocketCreationError);
}