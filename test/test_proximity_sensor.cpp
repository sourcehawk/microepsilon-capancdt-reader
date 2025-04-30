#include <gtest/gtest.h>
#include <cerrno>
#include <cstring>
#include <memory>
#include "capancdt/proximity_sensor.h"

using namespace proximity_sensor;


/*************************************
* ProximitySensorStatus
**************************************/

TEST(ProximitySensorStatusTest, RunningReturnsTrueWhenStatusRunning) {
    ProximitySensorStatus status;
    status.status = ProximityStatus::RUNNING;
    EXPECT_TRUE(status.running());
}

TEST(ProximitySensorStatusTest, RunningReturnsTrueWhenStatusStarting) {
    ProximitySensorStatus status;
    status.status = ProximityStatus::STARTING;
    EXPECT_TRUE(status.running());
}

TEST(ProximitySensorStatusTest, RunningReturnsFalseOtherwise) {
    ProximitySensorStatus status;

    status.status = ProximityStatus::STOPPED;
    EXPECT_FALSE(status.running());
    status.status = ProximityStatus::DISCONNECTED;
    EXPECT_FALSE(status.running());
    status.status = ProximityStatus::TIMEOUT;
    EXPECT_FALSE(status.running());
    status.status = ProximityStatus::PARSE_ERROR;
    EXPECT_FALSE(status.running());
    status.status = ProximityStatus::ERROR;
    EXPECT_FALSE(status.running());
    status.status = ProximityStatus::START_FAILURE;
    EXPECT_FALSE(status.running());
}

/*************************************
* ProximityDistance
**************************************/

TEST(ProximityDistanceTest, ValidReturnsTrueWhenTypeValid) {
    ProximityDistance distance;
    distance.type = ProximityDistanceType::VALID;
    EXPECT_TRUE(distance.valid());
}

TEST(ProximityDistanceTest, ValidReturnsFalseWhenTypeInvalid) {
    ProximityDistance distance;
    distance.type = ProximityDistanceType::NOT_AVAILABLE;
    EXPECT_FALSE(distance.valid());
}

TEST(ProximityDistanceTest, FromChannelValueReturnsValidDistance) {
    ChannelValue value(0, 0x00FF'FFFF);
    OptionalChannelValue optional_value;
    optional_value.set = true;
    optional_value.channel_value = value;
    ProximityDistance distance = ProximityDistance::from_channel_value(optional_value, 5000.0);

    EXPECT_EQ(distance.raw_value, 0x00FF'FFFF);
    EXPECT_EQ(distance.micrometers, 5000.0);
    EXPECT_EQ(distance.reading_number, 0);
    EXPECT_EQ(distance.type, ProximityDistanceType::VALID);
}

TEST(ProximityDistanceTest, FromChannelValueReturnsNotAvailableWhenValueIsNone) {
    OptionalChannelValue optional_value;
    ProximityDistance distance = ProximityDistance::from_channel_value(optional_value, 5000.0);

    EXPECT_EQ(distance.raw_value, 0);
    EXPECT_EQ(distance.micrometers, -1);
    EXPECT_EQ(distance.reading_number, 0);
    EXPECT_EQ(distance.type, ProximityDistanceType::NOT_AVAILABLE);
}

/*************************************
* ProximitySensor
**************************************/

struct ProximitySensorTest: ::testing::Test {
    void SetUp() override {
        // Create conn and mock conn
        mock_conn = new ProximityMockConnection();
        conn = mock_conn;

        // Create a mock message header
        ProximitySensorMessageHeader header{};
        memcpy(header.preamble, MESSAGE_HEADER_PREAMBLE_C, sizeof(header.preamble));
        header.order               = 69;
        header.serial              = 12345;
        header.channels            = 0x0000'0000'0000'0001u;
        header.status              = 0;
        header.frame_count         = 1;
        header.bytes_per_frame     = 4;
        header.start_sample_number = 0;

        // Create a mock message with the header
        message = ProximitySensorMessage(std::make_unique<ProximitySensorMessageHeader>(header));

        // Create proximity sensor instance
        proximity_sensor = std::make_unique<ProximitySensor>(conn, 10.0, 0.5);
    }

    void set_message_1(char* buffer, size_t buffer_size, size_t offset_bytes = 0) {
        message.channel_values.clear();
        message.header->channels = 0b00000000'00000000'00000000'00000000'00000000'00000000'00000000'00000101u;
        message.header->frame_count = 2;
        message.header->bytes_per_frame = 2*4; // 2 channels * 4 bytes each
        // frame 1
        message.set_channel_value(1, ChannelValue(message.header->start_sample_number, 0x00FF'FFFFu));
        message.set_channel_value(2, ChannelValue(message.header->start_sample_number+1, 0x007F'FFFFu));
        // frame 2
        message.set_channel_value(1, ChannelValue(message.header->start_sample_number+2, 0x006F'FFFFu));
        message.set_channel_value(2, ChannelValue(message.header->start_sample_number+3, 0x005F'FFFFu));

        // Encode the message to the buffer
        message.encode(buffer, buffer_size, offset_bytes);
    }

    void set_message_2(char* buffer, size_t buffer_size, size_t offset_bytes) {
        message.channel_values.clear();
        message.header->channels = 0b00000000'00000000'00000000'00000000'00000000'00000000'00000000'00000101u;
        message.header->frame_count = 1;
        message.header->start_sample_number = 4;
        message.set_channel_value(1, ChannelValue(message.header->start_sample_number, 0x004F'FFFFu));
        message.set_channel_value(2, ChannelValue(message.header->start_sample_number+1, 0x003F'FFFFu));

        // Encode the message to the buffer
        message.encode(buffer, buffer_size, offset_bytes);
    }

    ProximitySensorMessage message;
    ProximityMockConnection* mock_conn;
    ProximityConnection* conn;
    std::unique_ptr<ProximitySensor> proximity_sensor;
};

TEST_F(ProximitySensorTest, ProcessMessagesReturnsLatestMessage) {
    char buffer[1024] = {0};

    set_message_1(buffer, sizeof(buffer));
    size_t total_size = message.header->payload_size();
    set_message_2(buffer, sizeof(buffer), total_size);
    total_size += message.header->payload_size();

    // Convert buffer to a vector
    std::vector<char> messages(buffer, buffer + total_size);
    auto last_message = internal::process_messages(messages);

    EXPECT_EQ(*last_message, message);
}

TEST_F(ProximitySensorTest, ProcessMessagesFindsStartOfMessage) {
    char buffer[1024] = {0};

    // Leave 7 bytes of garbage at the start of the buffer
    set_message_1(buffer, sizeof(buffer), 7);
    
    // Convert buffer to a vector
    std::vector<char> messages(buffer, buffer + sizeof(buffer));

    auto last_message = internal::process_messages(messages);
    EXPECT_EQ(*last_message, message);
}

TEST_F(ProximitySensorTest, ProcessMessagesIgnoresInvalidBytesBetweenMessages) {
    char buffer[1024] = {0};

    // Adds some invalid bytes before first message
    set_message_1(buffer, sizeof(buffer), 20);

    // Add some invalid bytes between messages
    set_message_2(buffer, sizeof(buffer), message.header->payload_size()+31);

    // Convert buffer to a vector
    std::vector<char> messages(buffer, buffer + sizeof(buffer));

    auto last_message = internal::process_messages(messages);
    EXPECT_EQ(*last_message, message);
}

TEST_F(ProximitySensorTest, DefaultStatusIsStopped) {
    EXPECT_EQ(proximity_sensor->get_proximity_sensor_status().status, ProximityStatus::STOPPED);
}

TEST_F(ProximitySensorTest, StartProximitySensorFailsOnSocketCreationError) {
    mock_conn->connect_mock = []() {
        throw SocketCreationError("Socket creation failed");
    };
    bool started = proximity_sensor->start();
    EXPECT_FALSE(started);
    EXPECT_EQ(proximity_sensor->get_proximity_sensor_status().status, ProximityStatus::START_FAILURE);
}

TEST_F(ProximitySensorTest, StartSetsStartingStatus) {
    // Set up the mock connection that sleeps so that we can test the starting status
    mock_conn->connect_mock = []() {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    };

    // Start the sensor in a thread so that we can check the status while it is sleeping
    std::thread start_thread([this]() {
        proximity_sensor->start();
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(proximity_sensor->get_proximity_sensor_status().status, ProximityStatus::STARTING);

    // Wait for the thread to finish
    start_thread.join();

    EXPECT_NO_THROW(proximity_sensor->stop());
}

TEST_F(ProximitySensorTest, StatusBecomesRunningAfterStart) {
    // Add a valid message for the sensor to process
    char buffer[1024] = {0};
    set_message_1(buffer, sizeof(buffer));
    mock_conn->set_recv_data(buffer, message.header->payload_size());

    // Start the sensor and validate the status
    bool started = proximity_sensor->start();
    EXPECT_TRUE(started);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(proximity_sensor->get_proximity_sensor_status().status, ProximityStatus::RUNNING);

    EXPECT_NO_THROW(proximity_sensor->stop());
}

TEST_F(ProximitySensorTest, RepeatedStartCallsHaveNoEffect) {
    // Add a valid message for the sensor to process
    char buffer[1024] = {0};
    set_message_1(buffer, sizeof(buffer));
    mock_conn->set_recv_data(buffer, message.header->payload_size());

    // Start the sensor and validate the status
    bool started = proximity_sensor->start();
    EXPECT_TRUE(started);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(proximity_sensor->get_proximity_sensor_status().status, ProximityStatus::RUNNING);

    mock_conn->connect_mock = []() {
        throw std::runtime_error("Should not be called");
    };

    // Call start again and ensure it has no effect
    EXPECT_NO_THROW(proximity_sensor->start());
    EXPECT_EQ(proximity_sensor->get_proximity_sensor_status().status, ProximityStatus::RUNNING);

    EXPECT_NO_THROW(proximity_sensor->stop());
}

TEST_F(ProximitySensorTest, StopSetsStatusStopped) {
    char buffer[1024] = {0};
    set_message_1(buffer, sizeof(buffer));
    mock_conn->set_recv_data(buffer, message.header->payload_size());

    // Start the sensor and validate the status
    proximity_sensor->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(proximity_sensor->get_proximity_sensor_status().status, ProximityStatus::RUNNING);

    // Stop the sensor and validate the status
    EXPECT_NO_THROW(proximity_sensor->stop());
    EXPECT_EQ(proximity_sensor->get_proximity_sensor_status().status, ProximityStatus::STOPPED);
}

TEST_F(ProximitySensorTest, RepeatedStopCallsHaveNoEffect) {
    char buffer[1024] = {0};
    set_message_1(buffer, sizeof(buffer));
    mock_conn->set_recv_data(buffer, message.header->payload_size());

    // Start the sensor and validate the status
    proximity_sensor->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(proximity_sensor->get_proximity_sensor_status().status, ProximityStatus::RUNNING);

    // Stop the sensor and validate the status
    EXPECT_NO_THROW(proximity_sensor->stop());
    EXPECT_EQ(proximity_sensor->get_proximity_sensor_status().status, ProximityStatus::STOPPED);

    // Call stop again and ensure it has no effect
    EXPECT_NO_THROW(proximity_sensor->stop());
    EXPECT_EQ(proximity_sensor->get_proximity_sensor_status().status, ProximityStatus::STOPPED);
}

TEST_F(ProximitySensorTest, GetDistanceReturnsValidDistance) {
    char buffer[1024] = {0};
    set_message_1(buffer, sizeof(buffer));
    mock_conn->set_recv_data(buffer, message.header->payload_size());

    // Start the sensor and validate the status and distance values
    bool started = proximity_sensor->start();
    EXPECT_TRUE(started);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(proximity_sensor->get_proximity_sensor_status().status, ProximityStatus::RUNNING);

    ProximityDistance distance = proximity_sensor->get_distance(1, 5000.0);
    EXPECT_TRUE(distance.valid());
    EXPECT_EQ(distance.raw_value, 0x006F'FFFFu);

    distance = proximity_sensor->get_distance(2, 5000.0);
    EXPECT_TRUE(distance.valid());
    EXPECT_EQ(distance.raw_value, 0x005F'FFFFu); 

    // Stop the sensor
    proximity_sensor->stop();

    // Check that starting the sensor again does not affect the distance values
    started = proximity_sensor->start();
    EXPECT_TRUE(started);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(proximity_sensor->get_proximity_sensor_status().status, ProximityStatus::RUNNING);

    distance = proximity_sensor->get_distance(1, 5000.0);
    EXPECT_TRUE(distance.valid());
    EXPECT_EQ(distance.raw_value, 0x006F'FFFFu);

    // Add a new message to the buffer while the sensor is running
    char buffer2[1024] = {0};
    set_message_2(buffer2, sizeof(buffer2), 0);
    mock_conn->set_recv_data(buffer2, message.header->payload_size());

    // Ensure the new message is processed
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    EXPECT_EQ(proximity_sensor->get_proximity_sensor_status().status, ProximityStatus::RUNNING);
    distance = proximity_sensor->get_distance(1, 5000.0);
    EXPECT_TRUE(distance.valid());
    EXPECT_EQ(distance.raw_value, 0x004F'FFFFu);
}

TEST_F(ProximitySensorTest, GetDistanceReturnsValidDistanceWhenStopped) {
    char buffer[1024] = {0};
    set_message_1(buffer, sizeof(buffer));
    mock_conn->set_recv_data(buffer, message.header->payload_size());

    // Start the sensor and validate the status
    bool started = proximity_sensor->start();
    EXPECT_TRUE(started);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(proximity_sensor->get_proximity_sensor_status().status, ProximityStatus::RUNNING);

    // Stop the sensor and validate the status
    proximity_sensor->stop();

    // Check that the distance is still valid
    ProximityDistance distance = proximity_sensor->get_distance(1, 5000.0);
    EXPECT_TRUE(distance.valid());
}

TEST_F(ProximitySensorTest, GetDistanceReturnsNotAvailableWhenReadingNoLongerValid) {
    char buffer[1024] = {0};
    set_message_1(buffer, sizeof(buffer));
    mock_conn->set_recv_data(buffer, message.header->payload_size());

    // Start the sensor and validate the status
    bool started = proximity_sensor->start();
    EXPECT_TRUE(started);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(proximity_sensor->get_proximity_sensor_status().status, ProximityStatus::RUNNING);

    // Stop the sensor so that the reading is no longer updated
    proximity_sensor->stop();

    // Sleep past the valid time of a sensor reading and ensure the distance is not valid
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    ProximityDistance distance = proximity_sensor->get_distance(1, 5000.0);
    EXPECT_FALSE(distance.valid());
}

TEST_F(ProximitySensorTest, GetDistanceReturnsNotAvailableWhenNeverStarted) {
    ProximityDistance distance = proximity_sensor->get_distance(1, 5000.0);
    EXPECT_FALSE(distance.valid());
}

TEST_F(ProximitySensorTest, GetDistanceReturnsNotAvailableWhenNeverReceivedSensorReading) {
    // Give the mock connection some invalid data
    char buffer[1024] = {0};
    mock_conn->set_recv_data(buffer, sizeof(buffer));

    // Start the sensor and validate the status
    bool started = proximity_sensor->start();
    EXPECT_TRUE(started);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(proximity_sensor->get_proximity_sensor_status().status, ProximityStatus::RUNNING);

    // Ensure the distance is not valid
    ProximityDistance distance = proximity_sensor->get_distance(1, 5000.0);
    EXPECT_FALSE(distance.valid());

    // Stop the sensor
    proximity_sensor->stop();
}

TEST_F(ProximitySensorTest, GetDistanceResturnsNotAvailableWhenChannelDoesNotExist) {
    char buffer[1024] = {0};
    set_message_1(buffer, sizeof(buffer));
    mock_conn->set_recv_data(buffer, message.header->payload_size());

    // Start the sensor and validate the status
    bool started = proximity_sensor->start();
    EXPECT_TRUE(started);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(proximity_sensor->get_proximity_sensor_status().status, ProximityStatus::RUNNING);

    // Ensure the distance is not valid
    ProximityDistance distance = proximity_sensor->get_distance(3, 5000.0);
    EXPECT_FALSE(distance.valid());

    // Stop the sensor
    proximity_sensor->stop();
}

TEST_F(ProximitySensorTest, StatusBecomesDisconnectedWhenSocketClosedByServer) {
    mock_conn->set_recv_data(nullptr, 0);
    
    // Start the sensor and validate the status
    bool started = proximity_sensor->start();
    EXPECT_TRUE(started);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(proximity_sensor->get_proximity_sensor_status().status, ProximityStatus::DISCONNECTED);

    // Stop the sensor
    proximity_sensor->stop();
}

TEST_F(ProximitySensorTest, StatusBecomesTimeoutWhenNoDataReceivedForTimeoutPeriod) {
    mock_conn->set_recv_data(nullptr, -1, EAGAIN);

    bool started = proximity_sensor->start();
    EXPECT_TRUE(started);

    // sleep past the timeout period and ensure the status is timeout
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    EXPECT_EQ(proximity_sensor->get_proximity_sensor_status().status, ProximityStatus::TIMEOUT);

    // Set a message to check the same after a valid message has been received
    char buffer[1024] = {0};
    set_message_1(buffer, sizeof(buffer));
    mock_conn->set_recv_data(buffer, message.header->payload_size());

    // Sensor should be startable again after the timeout
    started = proximity_sensor->start();
    EXPECT_TRUE(started);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(proximity_sensor->get_proximity_sensor_status().status, ProximityStatus::RUNNING);

    // Set an error again
    mock_conn->set_recv_data(nullptr, -1, EAGAIN);

    // sleep past the timeout period and ensure the status is timeout
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    EXPECT_EQ(proximity_sensor->get_proximity_sensor_status().status, ProximityStatus::TIMEOUT);

    // Stop the sensor
    proximity_sensor->stop();
}

TEST_F(ProximitySensorTest, StatusBecomesParseErrorWhenMessageParsingFailsForTimeoutPeriod) {
    // set invalid data in the recv buffer
    char buffer[1024] = {0};
    mock_conn->set_recv_data(buffer, sizeof(buffer));

    // Start the sensor and validate the status
    bool started = proximity_sensor->start();
    EXPECT_TRUE(started);

    // sleep past the timeout period and ensure the status is parse error
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    EXPECT_EQ(proximity_sensor->get_proximity_sensor_status().status, ProximityStatus::PARSE_ERROR);

    // Set a message to check the same after a valid message has been received
    set_message_1(buffer, sizeof(buffer));
    mock_conn->set_recv_data(buffer, message.header->payload_size());

    // Start the sensor again
    started = proximity_sensor->start();
    EXPECT_TRUE(started);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(proximity_sensor->get_proximity_sensor_status().status, ProximityStatus::RUNNING);

    // Set invalid data again
    char buffer2[1024] = {0};
    mock_conn->set_recv_data(buffer2, sizeof(buffer2), 0);

    // sleep past the timeout period and ensure the status is a parse error
    std::this_thread::sleep_for(std::chrono::milliseconds(700));
    EXPECT_EQ(proximity_sensor->get_proximity_sensor_status().status, ProximityStatus::PARSE_ERROR);

    // Stop the sensor
    proximity_sensor->stop();
}

TEST_F(ProximitySensorTest, StatusBecomesErrorOnOtherSocketRecvErrors) {
    mock_conn->set_recv_data(nullptr, -1, EINVAL);

    // Start the sensor and validate the status
    bool started = proximity_sensor->start();
    EXPECT_TRUE(started);

    // ensure the status is error
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(proximity_sensor->get_proximity_sensor_status().status, ProximityStatus::ERROR);

    // Stop the sensor
    proximity_sensor->stop();
}

