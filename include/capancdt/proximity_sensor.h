// ProximitySensor.h

#ifndef PROXIMITYSENSOR_H
#define PROXIMITYSENSOR_H

#include "capancdt/proximity_connection.h"
#include "capancdt/proximity_sensor_message.h"
#include <string>
#include <stdexcept>
#include <mutex>
#include <atomic>
#include <thread>
#include <memory>

namespace proximity_sensor{

enum ProximityStatus {
    // Sensor is starting
    STARTING = 0,
    // Sensor is running and receiving data
    RUNNING = 1,
    // Sensor is not running (stopped by stop()) or has not been started yet
    STOPPED = 2,
    // Sensor disconnected by the server
    DISCONNECTED = 3,
    // No data received for the given timeout period
    TIMEOUT = 4,
    // Unable to parse messages for the given timeout period but receiving some data
    PARSE_ERROR = 5,
    // Some other socketreceive error occurred
    ERROR = 6,
    // Failed during startup due to a socket error
    START_FAILURE = 7,
};

struct ProximitySensorStatus {
    // The status of the sensor
    ProximityStatus status;
    // Message with additional information about the status
    std::string message;

    /*
     * Check if the sensor is running
     *
     * @return true if the status is ProximityStatus::RUNNING or ProximityStatus::STARTING, false otherwise
     */
    bool running() const;
};

struct OptionalChannelValue {
    // The channel value
    ChannelValue channel_value;
    // True if set
    bool set = false;
};

enum ProximityDistanceType {
    // The sensor value is valid
    VALID = 0,
    // The sensor value is not available
    NOT_AVAILABLE = 1
};

struct ProximityDistance {
    // The distance to the object in micrometers
    double micrometers = -1.0;
    // The raw value from the sensor
    uint32_t raw_value = 0;
    // Sensor reading number
    uint32_t reading_number = 0;
    // The type of the value
    ProximityDistanceType type = ProximityDistanceType::NOT_AVAILABLE;
    // Error message if the value is not valid
    std::string error_message;

    /*
     * Check if the distance is valid
     *
     * @return true if the distance type is ProximityDistanceType::VALID, false otherwise
     */
    bool valid() const;

    /*
     * Convert an optional ChannelValue to a ProximityDistance
     * 
     * @param v The OptionalChannelValue to convert
     * @param measuring_range_micrometers The measuring range for the sensor type in micrometers
     * @return ProximityDistance with the distance in micrometers
     * @note Sets distance to -1 and an error type if the distance is not valid
     *   - VALID if the distance is valid
     *   - NOT_AVAILABLE if OptionalChannelValue is not set
     */
    static ProximityDistance from_channel_value(OptionalChannelValue v, double measuring_range_micrometers);

    /*
     * Print the distance to an output stream
     *
     * @param os The output stream to print to
     * @param  The distance to print
     * @return The output stream
     */
     friend std::ostream& operator<<(std::ostream& os, ProximityDistance const& c);
};

class ProximitySensor
{
    public:
        /*
         * Initialize a new ProximitySensor instance
         *
         * @param conn The connection implementation to use for the sensor
         * @param sensor_rate_hz The rate of the sensor in Hz (default is 104 Hz)
         * @param read_timeout The maximum time to wait for a valid read from the sensor before timing out (default is 1.0 seconds)
         */
        ProximitySensor(ProximityConnection* conn, double sensor_rate_hz = 104.0, double read_timeout = 1.0);
        ~ProximitySensor();

        /*
         * Get the status of the sensor reader
         * 
         * If the sensor is not in ProximityStatus::RUNNING or ProximityStatus::STARTING state, 
         *  the sensor is not reading data and can be restarted with start()
         * @note Thread-safe
         */
        ProximitySensorStatus get_proximity_sensor_status() const;

        /*
        * Connect to the sensor and begin reading data
        * 
        * @return true if the connection was successful, false otherwise
        * @note Check sensor status to see reason for failure
        * @note Thread-safe and blocks until the connection is established (or fails)
        */
        bool start();

        /*
        * Stop reading data from the sensor
        *
        * @note Thread-safe and blocks until worker thread finishes
        */
        void stop();

        /*
        * Get the distance from the sensor to an object in micrometers.
        * 
        * @param channel The channel number to get the value for
        * @param measuring_range_micrometers The measuring range for the sensor type in micrometers
        * @return ProximityDistance with the distance in micrometers
        * @note Thread-safe
        */
        ProximityDistance get_distance(int channel, double measuring_range_micrometers) const;

    private:
        // The connection to the sensor
        ProximityConnection* conn;
        // Read/write lock for the sensor value
        mutable std::mutex lock;
        // Read/write lock for the status
        mutable std::mutex status_lock;
        // Read/write lock for starting and stopping the sensor
        mutable std::mutex start_stop_lock;
        // The thread that reads data from the sensor
        std::thread worker_thread;
        // Signal to stop the processing thread
        std::atomic<bool> should_exit{true};
        // The rate of the sensor in Hz
        double sensor_rate_hz;
        // The time in seconds that the read sensor value is valid
        double sensor_read_valid_time;
        // Time since the last sensor reading in seconds - SHARED VARIABLE
        double last_sensor_read_time;
        // The maximum time to wait for a valid read from the sensor before timing out
        double read_timeout;
        // Last message received from the sensor - SHARED VARIABLE
        std::unique_ptr<ProximitySensorMessage> last_sensor_message;
        // Status of the proximity sensor
        ProximitySensorStatus proximity_sensor_status;

        /*
        * Get the last read time of the sensor.
        *
        * @return The last read time in seconds since the epoch.
        * @note This function is thread-safe and uses a mutex to protect the shared data.
        */
        double get_last_sensor_read_time();

        /*
         * Updates the latest read time and latest the sensor message.
         * If the sensor message is nullptr, the last read time is set to 0.
         *
         * @param sensor_message ProximitySensorMessage (or nullptr) to set
         * @note This function is thread-safe and uses a mutex to protect the shared data.
         */
        void set_last_sensor_message(std::unique_ptr<ProximitySensorMessage> sensor_value);

        /*
        * Sets the proximity status.
        *
        * @param status The new proximity status to set
        * @note This function is thread-safe and uses a mutex to protect the shared data.
        */
        void set_proximity_sensor_status(ProximityStatus status, const std::string& message);

        /*
        * Get the most recently read proximity sensor value for a specific channel.
        *
        * @param channel The channel number to get the value for
        * @return Most recent sensor value for the channel, or boost::none if the value is no longer valid
        * @note This function is thread-safe and uses a mutex to protect the shared data.
        */
        OptionalChannelValue get_sensor_channel_value(int channel) const;

        /*
         * Function for a processing thread to process data from the sensor
         * The processing thread can be terminated by setting the should_exit variable to true
         */
        void read_sensor_data();
};

namespace internal {
    /*
     * Helper method to handle errors when receiving data from the sensor
     * 
     * @param last_read_time The last read time of the sensor
     * @param thread_start_time The start time of the processing thread
     * @param iter_start_time The start time of the current iteration
     * @param read_timeout The timeout period for receiving data
     * @return ProximitySensorStatus with the status and message
     */
    ProximitySensorStatus handle_recv_errors(double last_read_time, double thread_start_time, double iter_start_time, double read_timeout);

    /*
     * Helper method to process messages from the sensor
     * 
     * @param messages A vector containing the accumulated sensor data from the socket
     * @return The most recent ProximitySensorMessage in the accumulated data.
     */
    std::unique_ptr<ProximitySensorMessage> process_messages(std::vector<char>& messages);
}

} // namespace proximity_sensor

#endif // PROXIMITYSENSOR_H
