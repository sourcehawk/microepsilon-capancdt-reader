#include <gtest/gtest.h>
#include <memory>
#include "capancdt/proximity_sensor_message.h"

using namespace proximity_sensor;

/*************************************
* ProximitySensorMessageHeader
**************************************/

struct ProximitySensorMessageHeaderTest: ::testing::Test
{
    void SetUp() override {
        memcpy(header.preamble, MESSAGE_HEADER_PREAMBLE_C, sizeof(header.preamble));
        header.order               = 69;
        header.serial              = 12345;
        header.channels            = 0x0000'0000'0000'0001u;
        header.status              = 0;
        header.frame_count         = 1;
        header.bytes_per_frame     = 4;
        header.start_sample_number = 0;
    }

    ProximitySensorMessageHeader header{};
};

TEST_F(ProximitySensorMessageHeaderTest, SizeOfHeaderStructIs32) {
    EXPECT_EQ(sizeof(ProximitySensorMessageHeader), 32);
}

TEST_F(ProximitySensorMessageHeaderTest, PayloadSize) {
    header.bytes_per_frame = 3*4;
    header.frame_count = 2;

    EXPECT_EQ(header.payload_size(), sizeof(header) + 3*4*2);
}

TEST_F(ProximitySensorMessageHeaderTest, PayloadSizeNoFrames) {
    header.frame_count = 0;
    EXPECT_EQ(header.payload_size(), sizeof(header));
}

TEST_F(ProximitySensorMessageHeaderTest, EnabledChannels) {
    // Note: the hex version would be 0x0000'0040'0000'0005u
    header.channels = 0b00000000'00000000'00000000'01000000'00000000'00000000'00000000'00000101u;
    std::vector<int> channels = header.enabled_channels();
    EXPECT_EQ(channels.size(), 3);
    EXPECT_EQ(channels[0], 1);
    EXPECT_EQ(channels[1], 2);
    EXPECT_EQ(channels[2], 20);
}

TEST_F(ProximitySensorMessageHeaderTest, EnabledChannelsEmpty) {
    header.channels = 0x0000'0000'0000'0000u;
    std::vector<int> channels = header.enabled_channels();
    EXPECT_EQ(channels.size(), 0);
}

TEST_F(ProximitySensorMessageHeaderTest, EncodeDecode) {
    // Create a buffer to hold the encoded message
    size_t header_size = sizeof(header);
    char buffer[header_size] = {};

    // Encode the header to the buffer
    header.encode(buffer, header_size);

    // Decode the message from the buffer
    std::unique_ptr<ProximitySensorMessageHeader> decoded_header = ProximitySensorMessageHeader::decode(buffer, header_size);

    // Check if the decoded header matches the original header
    EXPECT_NE(decoded_header, nullptr);
    EXPECT_EQ(*decoded_header, header);
}

TEST_F(ProximitySensorMessageHeaderTest, DecodeInvalidPreambleReturnsNullptr) {
    // Create a buffer with an invalid preamble
    char buffer[sizeof(ProximitySensorMessageHeader)] = {};
    memcpy(buffer, "MEA_INVALID", 7);

    // Decode the message from the buffer
    std::unique_ptr<ProximitySensorMessageHeader> decoded_header = ProximitySensorMessageHeader::decode(
        buffer, sizeof(ProximitySensorMessageHeader)
    );

    // Check if the decoded header is nullptr
    EXPECT_EQ(decoded_header, nullptr);
}

TEST_F(ProximitySensorMessageHeaderTest, DecodeRaisesWhenBufferTooSmall) {
    // Create a buffer that is too small
    char buffer[sizeof(ProximitySensorMessageHeader) - 1] = {};
    memcpy(buffer, MESSAGE_HEADER_PREAMBLE_C, sizeof(header.preamble));

    // Decode the message from the buffer
    EXPECT_THROW(
        ProximitySensorMessageHeader::decode(buffer, sizeof(buffer)),
        std::out_of_range
    );
}

TEST_F(ProximitySensorMessageHeaderTest, DecodeRaisesWhenOffsetTooLarge) {
    // Create a buffer that is too small
    char buffer[sizeof(ProximitySensorMessageHeader)] = {};
    memcpy(buffer, MESSAGE_HEADER_PREAMBLE_C, sizeof(header.preamble));

    // Decode the message from the buffer
    EXPECT_THROW(
        ProximitySensorMessageHeader::decode(buffer, sizeof(buffer), sizeof(buffer)),
        std::out_of_range
    );
}

TEST_F(ProximitySensorMessageHeaderTest, DecodeRaisesWhenBufferNull) {
    // Create a buffer that is too small
    char* buffer = nullptr;

    // Decode the message from the buffer
    EXPECT_THROW(
        ProximitySensorMessageHeader::decode(buffer, sizeof(header)),
        std::invalid_argument
    );
}

TEST_F(ProximitySensorMessageHeaderTest, EncodeRaisesWhenBufferTooSmall) {
    // Create a buffer that is too small
    char buffer[sizeof(ProximitySensorMessageHeader) - 1] = {};
    memcpy(buffer, MESSAGE_HEADER_PREAMBLE_C, sizeof(header.preamble));

    // Encode the message to the buffer
    EXPECT_THROW(
        header.encode(buffer, sizeof(buffer)),
        std::out_of_range
    );
}

TEST_F(ProximitySensorMessageHeaderTest, EncodeRaisesWhenOffsetTooLarge) {
    // Create a buffer that is too small
    char buffer[sizeof(ProximitySensorMessageHeader)] = {};
    memcpy(buffer, MESSAGE_HEADER_PREAMBLE_C, sizeof(header.preamble));

    // Encode the message to the buffer
    EXPECT_THROW(
        header.encode(buffer, sizeof(buffer), sizeof(buffer)),
        std::out_of_range
    );
}

TEST_F(ProximitySensorMessageHeaderTest, EncodeRaisesWhenBufferNull) {
    // Create a buffer that is too small
    char* buffer = nullptr;

    // Encode the message to the buffer
    EXPECT_THROW(
        header.encode(buffer, sizeof(header)),
        std::invalid_argument
    );
}

/*************************************
* ChannelValue
**************************************/

TEST(ChannelValueTest, Constructor) {
    ChannelValue value(0, 0x007F'FFFFu);
    EXPECT_EQ(value.sample_number, 0);
    EXPECT_EQ(value.raw_value, 0x007F'FFFFu);
}

TEST(ChannelValueTest, ConstructorRaisesWhenRawValueTooLarge) {
    EXPECT_THROW(
        ChannelValue(0, 0xFF00'0000u),
        std::out_of_range
    );
}

TEST(ChannelValueTest, ToMicrometers) {
    ChannelValue value(0, 0x007F'FFFFu);
    double micrometers = value.to_micrometers(5000.0);
    EXPECT_NEAR(micrometers, 5000.0/2.0, 0.01);
}

TEST(ChannelValueTest, ToMicrometersWithDifferentRange) {
    ChannelValue value(0, 0x00FF'FFFFu);
    double micrometers = value.to_micrometers(10000.0);
    EXPECT_DOUBLE_EQ(micrometers, 10000.0);
}

TEST(ChannelValueTest, FromMicrometers) {
    double micrometers = 5000.0;
    uint32_t raw_value = ChannelValue::from_micrometers(micrometers, 10000.0);
    EXPECT_EQ(raw_value, 0x007F'FFFFu);

    ChannelValue value(0, raw_value);
    EXPECT_NEAR(value.to_micrometers(10000.0), micrometers, 0.01);
}

TEST(ChannelValueTest, FromMicrometersWithDifferentRange) {
    double micrometers = 5000.0;
    uint32_t raw_value = ChannelValue::from_micrometers(micrometers, 5000.0);
    EXPECT_EQ(raw_value, 0x00FF'FFFFu);

    ChannelValue value(0, raw_value);
    EXPECT_NEAR(value.to_micrometers(5000.0), micrometers, 0.01);
}

/*************************************
* ProximitySensorMessage
**************************************/

struct ProximitySensorMessageTest: ::testing::Test
{
    void SetUp() override {
        memcpy(header.preamble, MESSAGE_HEADER_PREAMBLE_C, sizeof(header.preamble));
        header.order               = 69;
        header.serial              = 12345;
        header.channels            = 0x0000'0000'0000'0001u;
        header.status              = 0;
        header.frame_count         = 1;
        header.bytes_per_frame     = 4;
        header.start_sample_number = 0;

        message = ProximitySensorMessage(std::make_unique<ProximitySensorMessageHeader>(header));
    }

    ProximitySensorMessageHeader header {};
    ProximitySensorMessage message;
};

TEST_F(ProximitySensorMessageTest, Constructor) {
    EXPECT_NE(message.header, nullptr);
}

TEST_F(ProximitySensorMessageTest, ConstructorRaisesWhenHeaderNull) {
    EXPECT_THROW(
        ProximitySensorMessage(nullptr),
        std::invalid_argument
    );
}

TEST_F(ProximitySensorMessageTest, SetChannelValue) {
    ChannelValue value(message.header->start_sample_number, 0x007F'FFFFu);
    message.set_channel_value(1, value);

    EXPECT_EQ(message.channel_values[1].size(), 1);
    EXPECT_EQ(message.channel_values[1][0].sample_number, 0);
    EXPECT_EQ(message.channel_values[1][0].raw_value, 0x007F'FFFFu);
}

TEST_F(ProximitySensorMessageTest, SetChannelValueMultiple) {
    ChannelValue value1(message.header->start_sample_number, 0x007F'FFFFu);
    message.set_channel_value(1, value1);

    ChannelValue value2(message.header->start_sample_number+1, 0x00FF'FFFFu);
    message.set_channel_value(1, value2);

    EXPECT_EQ(message.channel_values[1].size(), 2);
    EXPECT_EQ(message.channel_values[1][0].sample_number, message.header->start_sample_number);
    EXPECT_EQ(message.channel_values[1][0].raw_value, 0x007F'FFFFu);
    EXPECT_EQ(message.channel_values[1][1].sample_number, message.header->start_sample_number+1);
    EXPECT_EQ(message.channel_values[1][1].raw_value, 0x00FF'FFFFu);
}

TEST_F(ProximitySensorMessageTest, ChannelExists) {
    ChannelValue value(message.header->start_sample_number, 0x007F'FFFFu);
    message.set_channel_value(1, value);

    EXPECT_TRUE(message.channel_exists(1));
    EXPECT_FALSE(message.channel_exists(2));
}

TEST_F(ProximitySensorMessageTest, GetChannelValues) {
    ChannelValue value1_1(message.header->start_sample_number, 0x007F'FFFFu);
    ChannelValue value1_2(message.header->start_sample_number+1, 0x00FF'FFFFu);
    ChannelValue value2(message.header->start_sample_number+2, 0x0050'FFFFu);
    message.set_channel_value(1, value1_1);
    message.set_channel_value(1, value1_2);
    message.set_channel_value(2, value2);

    const std::vector<ChannelValue>& values = message.get_channel_values(1);
    EXPECT_EQ(values.size(), 2);
    EXPECT_EQ(values[0].sample_number, message.header->start_sample_number);
    EXPECT_EQ(values[0].raw_value, 0x007F'FFFFu);

    EXPECT_EQ(values[1].sample_number, message.header->start_sample_number+1);
    EXPECT_EQ(values[1].raw_value, 0x00FF'FFFFu);

    const std::vector<ChannelValue>& values2 = message.get_channel_values(2);
    EXPECT_EQ(values2.size(), 1);
    EXPECT_EQ(values2[0].sample_number, message.header->start_sample_number+2);
    EXPECT_EQ(values2[0].raw_value, 0x0050'FFFFu);
}

TEST_F(ProximitySensorMessageTest, GetChannelValuesRaisesWhenChannelDoesNotExist) {
    EXPECT_THROW(
        message.get_channel_values(2),
        std::invalid_argument
    );
}

TEST_F(ProximitySensorMessageTest, GetChannelValue) {
    ChannelValue value(message.header->start_sample_number, 0x007F'FFFFu);
    message.set_channel_value(1, value);

    ChannelValue channel_value = message.get_channel_value(1, 0);
    EXPECT_EQ(channel_value.sample_number, message.header->start_sample_number);
    EXPECT_EQ(channel_value.raw_value, 0x007F'FFFFu);
}

TEST_F(ProximitySensorMessageTest, GetChannelValueRaisesWhenChannelDoesNotExist) {
    EXPECT_THROW(
        message.get_channel_value(2, 0),
        std::invalid_argument
    );
}

TEST_F(ProximitySensorMessageTest, GetChannelValueRaisesWhenIndexOutOfRange) {
    ChannelValue value(message.header->start_sample_number, 0x007F'FFFFu);
    message.set_channel_value(1, value);

    EXPECT_THROW(
        message.get_channel_value(1, 1),
        std::out_of_range
    );
}

TEST_F(ProximitySensorMessageTest, EncodeDecode) {
    message.header->channels = 0b00000000'00000000'00000000'01000000'00000000'00000000'00000000'00000101u;
    message.header->frame_count = 2;
    message.header->bytes_per_frame = 3*4; // 3 channels * 4 bytes each
    message.header->start_sample_number = 1;

    // frame 1
    message.set_channel_value(1, ChannelValue(message.header->start_sample_number, 0x007F'FFFFu));
    message.set_channel_value(2, ChannelValue(message.header->start_sample_number+1, 0x006F'FFFFu));
    message.set_channel_value(20, ChannelValue(message.header->start_sample_number+2, 0x005F'FFFFu));
    // frame 2
    message.set_channel_value(1, ChannelValue(message.header->start_sample_number+3, 0x004F'FFFFu));
    message.set_channel_value(2, ChannelValue(message.header->start_sample_number+4, 0x003F'FFFFu));
    message.set_channel_value(20, ChannelValue(message.header->start_sample_number+5, 0x002F'FFFFu));

    // Create a buffer to hold the encoded message
    char buffer[1024];
    
    // Encode the message to the buffer
    size_t payload_size = message.header->payload_size();
    message.encode(buffer, sizeof(buffer));

    // Add some extra data to the buffer
    buffer[payload_size] = 'x';
    buffer[payload_size+1] = 'y';
    buffer[payload_size+2] = 'z';

    auto header_ptr = std::make_unique<ProximitySensorMessageHeader>(*message.header); 
    // Decode the message from the buffer
    std::unique_ptr<ProximitySensorMessage> decoded_message = ProximitySensorMessage::decode(std::move(header_ptr), buffer, sizeof(buffer));

    // Check if the decoded message matches the original message
    EXPECT_NE(decoded_message, nullptr);
    EXPECT_EQ(*decoded_message, message);
}

TEST_F(ProximitySensorMessageTest, EncodeDecodeWorksWithBufferOffsetNonZero) {
    message.header->channels = 0b00000000'00000000'00000000'01000000'00000000'00000000'00000000'00000101u;
    message.header->frame_count = 2;
    message.header->bytes_per_frame = 3*4; // 3 channels * 4 bytes each
    message.header->start_sample_number = 1;

    // frame 1
    message.set_channel_value(1, ChannelValue(message.header->start_sample_number, 0x007F'FFFFu));
    message.set_channel_value(2, ChannelValue(message.header->start_sample_number+1, 0x006F'FFFFu));
    message.set_channel_value(20, ChannelValue(message.header->start_sample_number+2, 0x005F'FFFFu));
    // frame 2
    message.set_channel_value(1, ChannelValue(message.header->start_sample_number+3, 0x004F'FFFFu));
    message.set_channel_value(2, ChannelValue(message.header->start_sample_number+4, 0x003F'FFFFu));
    message.set_channel_value(20, ChannelValue(message.header->start_sample_number+5, 0x002F'FFFFu));

    // Create a buffer to hold the encoded message
    char buffer[1024];

    // Encode the message to the buffer with an offset of 3
    message.encode(buffer, sizeof(buffer), 3);

    // Add some extra data to the buffer
    buffer[0] = 'x';
    buffer[1] = 'y';
    buffer[2] = 'z';

    auto header_ptr = std::make_unique<ProximitySensorMessageHeader>(*message.header); 
    // Decode the message from the buffer with a start offset of 3
    std::unique_ptr<ProximitySensorMessage> decoded_message = ProximitySensorMessage::decode(std::move(header_ptr), buffer, sizeof(buffer), 3);

    // Check if the decoded message matches the original message
    EXPECT_NE(decoded_message, nullptr);
    EXPECT_EQ(*decoded_message, message); 
}

TEST_F(ProximitySensorMessageTest, DecodeNullptrHeaderRaisesInvalidArgument) {
    // Create a buffer with an invalid preamble
    char buffer[1024];

    EXPECT_THROW(
        ProximitySensorMessage::decode(nullptr, buffer, sizeof(buffer)),
        std::invalid_argument
    );
}

TEST_F(ProximitySensorMessageTest, DecodeRaisesWhenBufferTooSmall) {
    // Create a buffer that is too small
    char buffer[5];
    memcpy(buffer, MESSAGE_HEADER_PREAMBLE_C, sizeof(header.preamble));

    // Decode the message from the buffer
    EXPECT_THROW(
        ProximitySensorMessage::decode(std::make_unique<ProximitySensorMessageHeader>(header), buffer, sizeof(buffer)),
        std::out_of_range
    );
}

TEST_F(ProximitySensorMessageTest, DecodeRaisesWhenOffsetTooLarge) {
    // Create a buffer that is too small
    char buffer[5];
    memcpy(buffer, MESSAGE_HEADER_PREAMBLE_C, sizeof(header.preamble));

    // Decode the message from the buffer
    EXPECT_THROW(
        ProximitySensorMessage::decode(std::make_unique<ProximitySensorMessageHeader>(header), buffer, sizeof(buffer), sizeof(buffer)),
        std::out_of_range
    );
}

TEST_F(ProximitySensorMessageTest, DecodeRaisesWhenBufferNull) {
    // Create a buffer that is too small
    char* buffer = nullptr;

    // Decode the message from the buffer
    EXPECT_THROW(
        ProximitySensorMessage::decode(std::make_unique<ProximitySensorMessageHeader>(header), buffer, sizeof(header)),
        std::invalid_argument
    );
}

TEST_F(ProximitySensorMessageTest, EncodeRaisesWhenBufferTooSmall) {
    // Create a buffer that is too small
    char buffer[8];
    memcpy(buffer, MESSAGE_HEADER_PREAMBLE_C, sizeof(header.preamble));

    // Encode the message to the buffer
    EXPECT_THROW(
        message.encode(buffer, sizeof(buffer)),
        std::out_of_range
    );
}

TEST_F(ProximitySensorMessageTest, EncodeRaisesWhenOffsetTooLarge) {
    // Create a buffer that is too small
    char buffer[1024];
    memcpy(buffer, MESSAGE_HEADER_PREAMBLE_C, sizeof(header.preamble));

    // Encode the message to the buffer
    EXPECT_THROW(
        message.encode(buffer, sizeof(buffer), sizeof(buffer)),
        std::out_of_range
    );
}

TEST_F(ProximitySensorMessageTest, EncodeRaisesWhenBufferNull) {
    // Create a buffer that is too small
    char* buffer = nullptr;

    // Encode the message to the buffer
    EXPECT_THROW(
        message.encode(buffer, sizeof(header)),
        std::invalid_argument
    );
}

TEST(DecodeRealSensorDataTest, DecodeRealSensorData) {
    // Extracted from wireshark
    uint8_t network_data[] = {
        // HEADER
        0x53, 0x41, 0x45, 0x4d,             // preamble               = "MEAS" as a 32-bit LE int
        0x48, 0xa3, 0x3e, 0x00,             // order                  = 0x003EA348 = 4 105 032
        0x3e, 0x08, 0x00, 0x00,             // serial                 = 0x0000083E = 2 110
        0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // channels   = 1
        0x00, 0x00, 0x00, 0x00,             // status                 = 0
        0x04, 0x00,                         // bytes_per_frame        = 4
        0x01, 0x00,                         // frame_count            = 1
        0xee, 0x52, 0x13, 0x00,             // start_sample_number    = 0x001352EE = 1 266 414
        // FRAME 1
        0xe3, 0x8e, 0x67, 0x00              // sample[0]              = 0x00678EE3 = 6 786 787
    };

    char buffer[sizeof(network_data)];
    memcpy(buffer, network_data, sizeof(buffer));

    // Decode the header:
    auto header = ProximitySensorMessageHeader::decode(buffer, sizeof(buffer));
    ASSERT_NE(header, nullptr);

    // --- verify each field ---
    // The preamble really should read back as "MEAS" after you
    // do your little-endian→host conversion 
    EXPECT_EQ(std::string(header->preamble, 4), "MEAS");

    EXPECT_EQ(header->order,               0x00'3E'A3'48u);
    EXPECT_EQ(header->serial,              0x00'00'08'3Eu);
    EXPECT_EQ(header->channels,            0x1ull);
    EXPECT_EQ(header->status,              0x00'00'00'00u);
    EXPECT_EQ(header->frame_count,         0x00'01u);
    EXPECT_EQ(header->bytes_per_frame,     0x00'04u);
    EXPECT_EQ(header->start_sample_number, 0x00'13'52'EEu);

    auto message = ProximitySensorMessage::decode(std::move(header), buffer, sizeof(buffer));
    ASSERT_NE(message, nullptr);
    EXPECT_EQ(message->get_channel_values(1).size(), 1);
    EXPECT_EQ(message->get_channel_value(1, 0).raw_value, 0x00'67'8E'E3u);
}