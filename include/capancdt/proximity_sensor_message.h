#ifndef PROXIMITY_SENSOR_MESSAGE_H
#define PROXIMITY_SENSOR_MESSAGE_H

#include <string>        // For std::string
#include <vector>        // For std::vector
#include <unordered_map> // For std::unordered_map
#include <cstdint>       // For fixed-width integer types like uint32_t, uint64_t
#include <memory>        // For std::unique_ptr
#include <iostream>      // For std::ostream

#define MESSAGE_HEADER_PREAMBLE_C "MEAS"

namespace proximity_sensor{

// WARNING: Adding attributes causes the message encoding/decoding to fail (Must be updated)
#pragma pack(push, 1)
struct ProximitySensorMessageHeader {
    // 4 byte string indicating the start of the message
    char preamble[4] = {0};
    // Order number of proximity sensor
    uint32_t order = 0;
    // The serial number of the proximity sensor
    uint32_t serial = 0;
    // A bit field containing two bits for each channel (channel 1 on lowest order bits - rightmost)
    // 00: Channel not available
    // 01: Channel available
    uint64_t channels = 0;
    // Not used
    uint32_t status = 0;
    // The number of bytes each frame is (enabled channels * 4 bytes)
    uint16_t bytes_per_frame = 0;
    // How many frames we have (one frame contains one value for each enabled channel)
    uint16_t frame_count = 0;
    // The sample number of the first channel value
    // Each consecutive channel value has the sample number incremented by 1
    // E.g with two channels enabled and 2 frames:
    // 1st frame: channel 1 sample number = 0, channel 2 sample number = 1
    // 2nd frame: channel 1 sample number = 2, channel 2 sample number = 3
    uint32_t start_sample_number = 0;

    /*
     * Default constructor
     */
    ProximitySensorMessageHeader() = default;

    /*
     * Get the enabled channel numbers from the channels bit field.
     * The channel numbers start from 1 (not 0)
     */
    std::vector<int> enabled_channels() const;

    /*
     * Calculated total size of the complete message in bytes
     */
    size_t payload_size() const;

    /*
     * Decode a message from a char buffer to a ProximitySensorMessageHeader.
     *
     * @param buffer Pointer to the buffer containing the message
     * @param buffer_size Size of the buffer to ensure it is large enough
     * 
     * @return A unique pointer to the decoded ProximitySensorMessageHeader
     *   returns nullptr if the preamble does not match
     * @raises std::invalid_argument if the buffer is null
     * @raises std::out_of_range if the buffer size is too small or offset is larger than the buffer size
     * @note Ensure you have sizeof(ProximitySensorMessageHeader) bytes available in the buffer from the offset
     *   before calling this function
     */
    static std::unique_ptr<ProximitySensorMessageHeader> decode(const char* buffer, size_t buffer_size, size_t offset_bytes = 0);
    
    /*
     * Encode the message to a buffer (little-endian formatted header fields)
     *
     * @param buffer The buffer to store the encoded message
     * @param buffer_size The size of the buffer to ensure it is large enough
     * @param offset_bytes The byte offset to start encoding from
     * @raises std::invalid_argument if the buffer is null
     * @raises std::out_of_range if the buffer size is too small or offset is larger than the buffer size
     */
    void encode(char* buffer, size_t buffer_size, size_t offset_bytes = 0) const;

    /*
     * Check if the header is equal to another header
     *
     * @param other The other header to compare with
     * @return true if the headers are equal, false otherwise
     */
    bool operator==(ProximitySensorMessageHeader const &other) const;

    /*
     * Check if the header is not equal to another header
     *
     * @param other The other header to compare with
     * @return true if the headers are not equal, false otherwise
     */
    bool operator!=(ProximitySensorMessageHeader const &other) const;

    /*
     * Print the header to an output stream
     *
     * @param os The output stream to print to
     * @param header The header to print
     * @return The output stream
     */
    friend std::ostream& operator<<(std::ostream& os, ProximitySensorMessageHeader const& p);
};
#pragma pack(pop)
static_assert(sizeof(ProximitySensorMessageHeader) == 32, "Packed header struct must be 32 bytes");

struct ChannelValue {
    // The sample number of the channel value
    uint32_t sample_number;
    // The raw value for the channel
    int raw_value;

    /*
     * Default constructor
     */
    ChannelValue() = default;

    /*
     * Construct a new ChannelValue
     *
     * @param sample_number The sample number of the channel value
     * @param raw_value The raw value for the channel
     * @raises std::out_of_range if the raw value is out of range (> 0x00FFFFFF)
     */
    ChannelValue(int sample_number, int raw_value);

    /*
     * Convert the channel value to micrometers
     * 
     * @param measuring_range_micrometers The measuring range for the sensor type in micrometers
     * @return The sensor channel value in micrometers
     */
    double to_micrometers(double measuring_range_micrometers) const;

    /*
     * Get the raw value representation of a micrometer value
     */
    static uint32_t from_micrometers(double micrometers, double measuring_range_micrometers);

    /*
     * Check if the channel value is equal to another
     *
     * @param other The other channel value to compare with
     * @return true if the channel values are equal, false otherwise
     */
    bool operator==(ChannelValue const &other) const;

    /*
     * Print the channel value to an output stream
     *
     * @param os The output stream to print to
     * @param  The channel value to print
     * @return The output stream
     */
    friend std::ostream& operator<<(std::ostream& os, ChannelValue const& c);
};

struct ProximitySensorMessage {
    std::unique_ptr<ProximitySensorMessageHeader> header;
    std::unordered_map<int, std::vector<ChannelValue>> channel_values = {};

    /*
     * Default constructor
     */
    ProximitySensorMessage() = default;

    /*
     * Initialize a new ProximitySensorMessage with a header
     *
     * @param header The header for the message
     * @raises std::invalid_argument if the header is null
     */
     ProximitySensorMessage(std::unique_ptr<ProximitySensorMessageHeader> header);

    /*
     * Append a channel value to a specific channel in the message
     *
     * @param channel The channel number to set values for
     * @param value The value to set for the channel
     */
    void set_channel_value(int channel, ChannelValue value);

    /*
     * Check if a specific channel exists in the message
     *
     * @param channel The channel number to check
     * @return true if the channel exists, false otherwise
     */
    bool channel_exists(int channel) const;

    /*
     * Get the available values for a specific channel
     * The values are ordered from the oldest (0) to the most recent (N)
     * 
     * @param channel The channel number to get values for
     * @return A reference to the vector of channel values
     * @raises std::invalid_argument if the channel does not exist
     */
    const std::vector<ChannelValue>& get_channel_values(int channel) const;

    /*
     * Get channel value at a specific index
     *
     * @param channel The channel number to get values for
     * @param index The index of the value to get
     * @return The channel value at the specified index
     * @raises std::out_of_range if the index is out of range for the channel
     * @raises std::invalid_argument if the channel does not exist
     */
    ChannelValue get_channel_value(int channel, int index) const;

    /*
     * Decode a message from a char buffer to a ProximitySensorMessage
     *
     * @param header Pointer to the ProximitySensorMessageHeader for the message
     * @param buffer Pointer to the data buffer containing the message
     * @param buffer_size Size of the data buffer to ensure it is large enough
     * @param offset_bytes The byte offset to start decoding from (do NOT account for the header in the offset)
     * 
     * @return A unique pointer to the decoded ProximitySensorMessage
     * @raises std::invalid_argument if the buffer is null or header is null
     * @raises std::out_of_range if the buffer size is too small or offset is larger than the buffer size
     * @note Ensure you have enough bytes available in the buffer from the offset (use header->payload_size())
     *   before calling this function
     */
    static std::unique_ptr<ProximitySensorMessage> decode(std::unique_ptr<ProximitySensorMessageHeader> header, const char* buffer, size_t buffer_size, size_t offset_bytes = 0);

    /*
     * Encode the message to a char buffer (little-endian format)
     *
     * @param buffer The buffer to store the encoded message
     * @param offset_bytes The byte offset to start encoding from
     * @param buffer_size The size of the buffer to ensure it is large enough
     * @raises std::invalid_argument if the buffer is null
     * @raises std::out_of_range if the buffer size is too small or offset is larger than the buffer size
     */
    void encode(char* buffer, size_t buffer_size, size_t offset_bytes = 0) const;

    /*
     * Check if the message is equal to another message
     *
     * @param other The other message to compare with
     * @return true if the messages are equal, false otherwise
     */
    bool operator==(ProximitySensorMessage const &other) const;

    /*
     * Print the message to an output stream
     *
     * @param os The output stream to print to
     * @param  The message to print
     * @return The output stream
     */
    friend std::ostream& operator<<(std::ostream& os, ProximitySensorMessage const& m);
};

} // namespace proximity_sensor

#endif // PROXIMITY_SENSOR_MESSAGE_H
