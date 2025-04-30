#ifndef PROXIMITY_CONNECTION_H
#define PROXIMITY_CONNECTION_H

#include <exception>
#include <string>
#include <sys/types.h>  // for ssize_t
#include <mutex>        // For thread-safe operations (e.g., std::mutex, std::lock_guard)

namespace proximity_sensor{

class SocketCreationError: public std::exception
{
  public:
    explicit SocketCreationError(const std::string& message) : message(message) {}
    virtual const char* what() const noexcept override
    {
        return message.c_str();
    }
  private:  
    std::string message;
};


class ProximityConnection
{
    public:
        /*
         * Default destructor
         */
        virtual ~ProximityConnection() = default;
        /*
         * Receive data from the socket
         *
         * @param buffer The buffer to store the received data
         * @param size The size of the buffer
         * @return The number of bytes received
         */
        virtual ssize_t recv(char* buffer, size_t size) = 0;
        /*
         * Connect to the socket
         *
         * @raises SocketCreationError if the socket cannot be created
         */
        virtual void connect() = 0;
        /*
         * Close the connection
         */
        virtual void disconnect() = 0;
};

class ProximitySocketConnection : public ProximityConnection
{
    public:
        ProximitySocketConnection(const std::string& ip, int port);
        ~ProximitySocketConnection() override;
        ssize_t recv(char* buffer, size_t size) override;
        void connect() override;
        void disconnect() override;

    private:
        int sock = -1;
        int port = -1;
        std::string ip;
};

class ProximityMockConnection : public ProximityConnection
{
    public:
        ProximityMockConnection() = default;
        ssize_t recv(char* buffer, size_t size) override;
        void connect() override;
        void disconnect() override;
        /*
         * Set the number of bytes to take from the given buffer on the next recv call
         *
         * @param data The data to copy into the recv buffer
         * @param data_size The size of the data to copy
         * @param errno_override The error code to set in case of an error
         * @note To mock an error, set the data_size to 0 or -1 and set errno to the desired error
         */
        void set_recv_data(char* data, ssize_t data_size, int errno_override = 0);
        /*
        * Set the function to call when connect() is called
        *
        * @param connect_mock The function to call
        */
        void (*connect_mock)() = nullptr;
    private:
        mutable std::mutex recv_lock;
        char* recv_data;
        ssize_t recv_size;
        int errno_override;
};

} // namespace proximity_sensor

#endif // PROXIMITY_CONNECTION_H