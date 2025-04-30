// ProximitySensor.cpp

#include "capancdt/proximity_sensor.h"
#include "capancdt/proximity_connection.h"
#include "capancdt/proximity_sensor_message.h"
#include <chrono>            // For time-related operations (e.g., std::chrono::high_resolution_clock)
#include <thread>            // For thread operations (e.g., std::this_thread::sleep_for)
#include <cstdint>           // For fixed-width integer types (e.g., uint32_t)
#include <cstring>           // For memory operations (e.g., std::memcpy)
#include <memory>            // For smart pointers (e.g., std::unique_ptr, std::make_unique)
#include <sstream>           // For string stream operations (e.g., std::ostringstream)
#include <vector>            // For dynamic arrays (e.g., std::vector)
#include <mutex>             // For thread-safe operations (e.g., std::mutex, std::lock_guard)
#include <stdexcept>         // For exceptions (e.g., std::invalid_argument, std::out_of_range)
#include <atomic>            // For atomic operations (e.g., std::atomic<bool>)


namespace proximity_sensor
{
/*
 * Get the current time in seconds since the epoch using a steady clock.
 * A steady clock is used to avoid issues with system time changes and is 
 * suitable for measuring intervals.
 */
double current_time_seconds()
{
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()
    ).count();
}

/*************************************
* ProximityDistance
**************************************/

bool ProximityDistance::valid() const {
    return this->type == ProximityDistanceType::VALID;
}

ProximityDistance ProximityDistance::from_channel_value(OptionalChannelValue v, double measuring_range_micrometers)
{
    ProximityDistance distance;

    if (!v.set)
    {
        distance.raw_value = 0;
        distance.micrometers = -1;
        distance.reading_number = 0;
        distance.type = ProximityDistanceType::NOT_AVAILABLE;
        distance.error_message = "No valid channel value available";
        return distance;
    }

    distance.raw_value = v.channel_value.raw_value;
    distance.micrometers = v.channel_value.to_micrometers(measuring_range_micrometers);
    distance.reading_number = v.channel_value.sample_number;
    distance.type = ProximityDistanceType::VALID;

    return distance;
}

std::ostream& operator<<(std::ostream& os, ProximityDistance const& c)
{
    if (c.type == ProximityDistanceType::NOT_AVAILABLE)
    {
        os << "ProximityDistance(NOT_AVAILABLE)";
        return os;
    }
    os << "ProximityDistance(micrometers: " << c.micrometers 
       << ", raw_value: 0x" << std::hex << std::uppercase << static_cast<uint32_t>(c.raw_value) 
       << ", reading_number: " << std::dec << c.reading_number << ")";
    return os;
}

/*************************************
* ProximitySensorStatus
**************************************/

bool ProximitySensorStatus::running() const {
    return status == ProximityStatus::RUNNING || status == ProximityStatus::STARTING;
}

/*************************************
* ProximitySensor: public
**************************************/

ProximitySensor::ProximitySensor(ProximityConnection* conn, double sensor_rate_hz, double read_timeout)
{
    if (sensor_rate_hz <= 0) {
        throw std::invalid_argument("Sensor rate must be greater than 0");
    }
    this->conn = conn;
    this->sensor_rate_hz = sensor_rate_hz;
    this->read_timeout = read_timeout;
    this->sensor_read_valid_time = (1.0 / sensor_rate_hz) * 2;
    this->set_proximity_sensor_status(ProximityStatus::STOPPED, "Proximity sensor not started yet");
}

ProximitySensor::~ProximitySensor()
{
    this->stop();
}

bool ProximitySensor::start()
{
    std::lock_guard<std::mutex> guard(start_stop_lock);
    // Check if the sensor is already running
    if (!should_exit.load()) return true;

    // If the sensor halted because of an error, ensure we join the old thread before starting a new one
    if (this->get_proximity_sensor_status().status != ProximityStatus::STOPPED && this->worker_thread.joinable())
    {
        this->worker_thread.join();
    }
    
    this->set_proximity_sensor_status(ProximityStatus::STARTING, "Initializing proximity sensor reader");
    
    // Try to connect to the sensor
    try {
        this->conn->connect();
    } catch (const SocketCreationError& e) {
        std::ostringstream oss;
        oss << "Failed to connect to sensor: " << e.what();
        this->set_proximity_sensor_status(ProximityStatus::START_FAILURE, oss.str());
        return false;
    }
    // Start the processing thread
    this->should_exit.store(false);
    this->set_last_sensor_message(nullptr);
    this->worker_thread = std::thread(&ProximitySensor::read_sensor_data, this);

    return true;
}

void ProximitySensor::stop()
{
    std::lock_guard<std::mutex> guard(start_stop_lock);
    this->should_exit.store(true);
    this->set_proximity_sensor_status(ProximityStatus::STOPPED, "Proximity sensor reader stopped");
    
    // Wait for the thread to finish
    if (this->worker_thread.joinable()) {
        this->worker_thread.join();
    }
    // Disconnect just in case the connection is still open (should not be - thread should have closed it)
    this->conn->disconnect();
}

ProximitySensorStatus ProximitySensor::get_proximity_sensor_status() const
{
    std::lock_guard<std::mutex> guard(status_lock);
    return this->proximity_sensor_status;
}

ProximityDistance ProximitySensor::get_distance(int channel, double measuring_range_micrometers) const
{
    OptionalChannelValue channel_value = this->get_sensor_channel_value(channel);
    return ProximityDistance::from_channel_value(channel_value, measuring_range_micrometers);
}

/*************************************
* ProximitySensor: private
**************************************/

void ProximitySensor::set_proximity_sensor_status(ProximityStatus status, const std::string& message)
{
    std::lock_guard<std::mutex> guard(status_lock);
    ProximitySensorStatus proximity_status{};
    proximity_status.status = status;
    proximity_status.message = message;
    this->proximity_sensor_status = proximity_status;
}

void ProximitySensor::set_last_sensor_message(std::unique_ptr<ProximitySensorMessage> sensor_message)
{
    std::lock_guard<std::mutex> guard(lock);
    if (sensor_message == nullptr)
    {
        this->last_sensor_message = nullptr;
        this->last_sensor_read_time = 0;
    }
    else
    {
        this->last_sensor_read_time = proximity_sensor::current_time_seconds();
        this->last_sensor_message = std::move(sensor_message);
    }
}

OptionalChannelValue ProximitySensor::get_sensor_channel_value(int channel) const
{
    std::lock_guard<std::mutex> guard(lock);

    OptionalChannelValue value;
    value.set = false;

    double read_time_delta = proximity_sensor::current_time_seconds() - this->last_sensor_read_time;

    if (this->last_sensor_message && read_time_delta < this->sensor_read_valid_time)
    {
        if (!this->last_sensor_message->channel_exists(channel))
        {
            return value;
        }
        // return the most recent value for the channel
        value.set = true;
        value.channel_value = this->last_sensor_message->get_channel_values(channel).back();
        return value;
    }
    return value;
}

double ProximitySensor::get_last_sensor_read_time()
{
    std::lock_guard<std::mutex> guard(lock);
    return this->last_sensor_read_time;
}

namespace internal {
std::unique_ptr<ProximitySensorMessage> process_messages(std::vector<char>& messages)
{
    std::unique_ptr<ProximitySensorMessageHeader> last_header;
    std::unique_ptr<ProximitySensorMessage> last_message;

    while (messages.size() >= sizeof(ProximitySensorMessageHeader))
    {
        last_header = ProximitySensorMessageHeader::decode(messages.data(), messages.size(), 0);

        // If the current position does not contain the expected preamble
        // remove the first byte from the vector and go to the next iteration
        if (last_header == nullptr)
        {
            messages.erase(messages.begin());
            continue;
        }

        size_t message_size = last_header->payload_size();

        // Not enough remaining data to decode the message, return last message
        if (message_size > messages.size())
        {
            return last_message;
        }

        // We have enough data to decode a message, assign it as the last message
        last_message = ProximitySensorMessage::decode(
            std::move(last_header),
            messages.data(),
            messages.size(),
            0
        );
        // Remove the processed message from the vector
        messages.erase(messages.begin(), messages.begin() + message_size);
    }

    return last_message;
}
 
ProximitySensorStatus handle_recv_errors(double last_sensor_read_time, double thread_start_time, double iter_start_time, double read_timeout)
{
    ProximitySensorStatus status;

    // There is no data to be read. Since we are using a non-blocking socket, 
    // this usually means that the sensor has not yet sent the next sensor reading
    if (errno == EAGAIN || errno == EWOULDBLOCK)
    {
        if (
            // We have yet to receive a sensor reading since startup and have timed out
            (last_sensor_read_time == 0 && iter_start_time - thread_start_time > read_timeout) ||
            // We have received a sensor reading before and have timed out
            (last_sensor_read_time > 0 && iter_start_time - last_sensor_read_time > read_timeout)
        )
        {
            std::ostringstream oss;
            oss << "Proximity sensor timed out. Failed to receive data for " << read_timeout << " seconds";
            status.status = ProximityStatus::TIMEOUT;
            status.message = oss.str();
            return status;
        }
        // We don't do anything, we are up to date with the socket buffer
        // Note that the get_sensor_channel_value getter returns boost::none if the value is no longer valid
        status.status = ProximityStatus::RUNNING;
        status.message = "Proximity sensor is running";
        return status;
    }
    // The connection terminated unexpectedly
    else if (errno == ECONNRESET || errno == ETIMEDOUT)
    {
        status.status = ProximityStatus::DISCONNECTED;
        status.message = "Connection to the proximity sensor closed by the server";
        return status;
    }
    // Other errors such as EPIPE, EINVAL, etc.
    else
    {
        std::ostringstream oss;
        oss << "Failed to receive data from the proximity sensor. Error code " << errno << ": " << strerror(errno);
        status.status = ProximityStatus::ERROR;
        status.message = oss.str();
        return status;
    }
}

} // namespace internal

void ProximitySensor::read_sensor_data()
{
    char buffer[1024];
    std::vector<char> accumulated_data;
    double thread_start_time = proximity_sensor::current_time_seconds();

    // set initial status to running
    this->set_proximity_sensor_status(ProximityStatus::RUNNING, "Proximity sensor is running");

    while (!should_exit.load())
    {
        double iter_start_time = proximity_sensor::current_time_seconds();
        int bytes_received = this->conn->recv(buffer, sizeof(buffer));

        if (bytes_received == 0)
        {
            this->set_proximity_sensor_status(
                ProximityStatus::DISCONNECTED, 
                "Connection to the proximity sensor closed by the server"
            );
            break;
        }

        if (bytes_received < 0)
        {
            ProximitySensorStatus proximity_status = proximity_sensor::internal::handle_recv_errors(
                this->get_last_sensor_read_time(),
                thread_start_time,
                iter_start_time,
                this->read_timeout
            );

            if (!proximity_status.running())
            {
                this->set_proximity_sensor_status(
                    proximity_status.status,
                    proximity_status.message
                );
                break;
            } 
            else {
                // We don't do anything, we are up to date with the socket buffer
                // skip to the next iteration
                continue;
            }
        }

        // Add received data (from start of buffer to buffer+bytes_received) to the end of the vector
        accumulated_data.insert(accumulated_data.end(), buffer, buffer + bytes_received);

        // Process the accumulated data and get the most recent message
        std::unique_ptr<ProximitySensorMessage> message = proximity_sensor::internal::process_messages(accumulated_data);

        // Set the sensor value if a valid message is found
        if (message != nullptr)
        {
            this->set_last_sensor_message(std::move(message));
            this->set_proximity_sensor_status(ProximityStatus::RUNNING, "Proximity sensor is running and receiving data");
        }
        // Check if we've timed out
        else {
            double last_read_time = this->get_last_sensor_read_time();
            if (
                // We have not yet received a valid message since startup and have reached the timeout
                (last_read_time == 0 && (current_time_seconds() - thread_start_time > this->read_timeout)) ||
                // We have received a valid message before and have reached the timeout
                (last_read_time > 0 && (current_time_seconds() - this->get_last_sensor_read_time() > this->read_timeout))
            ) {
                std::ostringstream oss;
                oss << "Failed to parse a valid proximity sensor message for " << this->read_timeout << " seconds";
                this->set_proximity_sensor_status(
                    ProximityStatus::PARSE_ERROR,
                    oss.str()
                );
                break;
            }
        }

        // Sleep for the remaining time of the sensor rate
        double iter_end_time = proximity_sensor::current_time_seconds();
        double remaining_time = (1.0 / this->sensor_rate_hz) - (iter_end_time - iter_start_time);

        if (remaining_time > 0)
        {
            std::this_thread::sleep_for(std::chrono::duration<double>(remaining_time));
        }
    }
    // Reset values before exiting
    this->should_exit.store(true);
    this->conn->disconnect();
}

} // namespace proximity_sensor