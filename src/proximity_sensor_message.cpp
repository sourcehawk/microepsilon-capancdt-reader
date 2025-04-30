#include "capancdt/proximity_sensor_message.h"
#include <cstring>
#include <string>
#include <memory>
#include <stdexcept>
#include <endian.h>  // For endian conversion (le16toh, le32toh, le64toh)
#include <vector>
#include <sys/types.h>
#include <bitset>

namespace proximity_sensor
{
constexpr uint32_t CHANNEL_VALUE_MASK = 0x00FF'FFFFu;

/*************************************
* ProximitySensorMessageHeader
**************************************/

std::vector<int> ProximitySensorMessageHeader::enabled_channels() const {
     std::vector<int> acc_channels;

     // There are 32 pairs in 64 bits (2 bits per channel)
     for (int i = 0; i < 32; i++) {
          // Isolate the two bits corresponding to the current channel (& with '11')
          uint64_t bits = (channels >> (2 * i)) & 0x3ULL;
          // If the two bits equal '01', mark the channel as available.
          if (bits == 0x1ULL) {
               acc_channels.push_back(i+1);
          }
     }
     return acc_channels;
}

size_t ProximitySensorMessageHeader::payload_size() const {
     return sizeof(ProximitySensorMessageHeader) + (this->frame_count * this->bytes_per_frame);
}

std::unique_ptr<ProximitySensorMessageHeader> ProximitySensorMessageHeader::decode(const char* buffer, size_t buffer_size, size_t offset_bytes)
{
     if (!buffer) {
          throw std::invalid_argument("Buffer is null");
     }

     if (buffer_size < offset_bytes) {
          throw std::out_of_range("Buffer size is smaller than offset");
     }

     if (buffer_size - offset_bytes < sizeof(ProximitySensorMessageHeader)) {
          throw std::out_of_range("Buffer does not contain enough space for the message");
     }

     // We need to convert the preamble to host order to ensure our preamble comparison is not endian dependent
     // E.g on a host using little-endian, the preamble would read "MEAS", and on a big-endian host, it would read 
     // "SAEM" if we were to read it directly from the data buffer instead of converting it to host order first
     uint32_t le_preamble; // Little-endian preamble
     
     std::memcpy(&le_preamble, buffer + offset_bytes, sizeof(le_preamble));
     offset_bytes += sizeof(le_preamble);
     
     uint32_t h_preamble = le32toh(le_preamble); // Host order preamble
     char h_preamble_chars[4]; // Host order preamble char array to construct

     h_preamble_chars[0] = (h_preamble >> 24) & 0xFF; // highest‑order byte
     h_preamble_chars[1] = (h_preamble >> 16) & 0xFF;
     h_preamble_chars[2] = (h_preamble >> 8) & 0xFF;
     h_preamble_chars[3] = h_preamble & 0xFF;         // lowest‑order byte

     // Check if the preamble matches the expected value and return nullptr if not
     if (std::memcmp(h_preamble_chars, MESSAGE_HEADER_PREAMBLE_C, sizeof(h_preamble_chars)) != 0) {
          return nullptr;
     }

     // We are at the beginning of the header, so we can start decoding the rest of the message
     // The remainder of the header is also in little endian format
     uint32_t le_order;
     uint32_t le_serial;
     uint64_t le_channels;
     uint32_t le_status;
     uint16_t le_bytes_per_frame;
     uint16_t le_frame_count;
     uint32_t le_start_sample_number;

     // Copy the little-endian values from the data buffer to temporary variables
     std::memcpy(&le_order, buffer + offset_bytes, sizeof(le_order));
     offset_bytes += sizeof(le_order);
     std::memcpy(&le_serial, buffer + offset_bytes, sizeof(le_serial));
     offset_bytes += sizeof(le_serial);
     std::memcpy(&le_channels, buffer + offset_bytes, sizeof(le_channels));
     offset_bytes += sizeof(le_channels);
     std::memcpy(&le_status, buffer + offset_bytes, sizeof(le_status));
     offset_bytes += sizeof(le_status);
     std::memcpy(&le_bytes_per_frame, buffer + offset_bytes, sizeof(le_bytes_per_frame));
     offset_bytes += sizeof(le_bytes_per_frame);
     std::memcpy(&le_frame_count, buffer + offset_bytes, sizeof(le_frame_count));
     offset_bytes += sizeof(le_frame_count);
     std::memcpy(&le_start_sample_number, buffer + offset_bytes, sizeof(le_start_sample_number));

     // Convert the little-endian values to host order and assign them to the header struct
     ProximitySensorMessageHeader header{};
     memcpy(header.preamble, h_preamble_chars, sizeof(header.preamble));
     header.order               = le32toh(le_order);
     header.serial              = le32toh(le_serial);
     header.channels            = le64toh(le_channels);
     header.status              = le32toh(le_status);
     header.bytes_per_frame     = le16toh(le_bytes_per_frame);
     header.frame_count         = le16toh(le_frame_count);
     header.start_sample_number = le32toh(le_start_sample_number);

     return std::make_unique<ProximitySensorMessageHeader>(header);
}

void ProximitySensorMessageHeader::encode(char* buffer, size_t buffer_size, size_t offset_bytes) const
{
     if (!buffer) {
          throw std::invalid_argument("Buffer is null");
     }

     if (buffer_size < offset_bytes) {
          throw std::out_of_range("Buffer size is smaller than offset");
     }

     if (buffer_size - offset_bytes < sizeof(ProximitySensorMessageHeader)) {
          throw std::out_of_range("Buffer does not contain enough space for the message");
     }

     // Convert the preamble to int
     uint32_t h_preamble = 
          (uint32_t(uint8_t(this->preamble[0])) << 24) | 
          (uint32_t(uint8_t(this->preamble[1])) << 16) | 
          (uint32_t(uint8_t(this->preamble[2])) <<  8) | 
          (uint32_t(uint8_t(this->preamble[3]))      );

     // Convert to little-endian format
     uint32_t le_preamble = htole32(h_preamble);
     uint32_t le_order = htole32(this->order);
     uint32_t le_serial = htole32(this->serial);
     uint64_t le_channels = htole64(this->channels);
     uint32_t le_status = htole32(this->status);
     uint16_t le_bytes_per_frame = htole16(this->bytes_per_frame);
     uint16_t le_frame_count = htole16(this->frame_count);
     uint32_t le_start_sample_number = htole32(this->start_sample_number);

     // Copy the little-endian values to the buffer
     std::memcpy(buffer + offset_bytes, &le_preamble, sizeof(le_preamble));
     offset_bytes += sizeof(le_preamble);
     std::memcpy(buffer + offset_bytes, &le_order, sizeof(le_order));
     offset_bytes += sizeof(le_order);
     std::memcpy(buffer + offset_bytes, &le_serial, sizeof(le_serial));
     offset_bytes += sizeof(le_serial);
     std::memcpy(buffer + offset_bytes, &le_channels, sizeof(le_channels));
     offset_bytes += sizeof(le_channels);
     std::memcpy(buffer + offset_bytes, &le_status, sizeof(le_status));
     offset_bytes += sizeof(le_status);
     std::memcpy(buffer + offset_bytes, &le_bytes_per_frame, sizeof(le_bytes_per_frame));
     offset_bytes += sizeof(le_bytes_per_frame);
     std::memcpy(buffer + offset_bytes, &le_frame_count, sizeof(le_frame_count));
     offset_bytes += sizeof(le_frame_count);
     std::memcpy(buffer + offset_bytes, &le_start_sample_number, sizeof(le_start_sample_number));
}

bool ProximitySensorMessageHeader::operator==(ProximitySensorMessageHeader const &other) const
{
     return std::memcmp(this->preamble, other.preamble, sizeof(this->preamble)) == 0 &&
          this->order == other.order &&
          this->serial == other.serial &&
          this->channels == other.channels &&
          this->status == other.status &&
          this->frame_count == other.frame_count &&
          this->bytes_per_frame == other.bytes_per_frame &&
          this->start_sample_number == other.start_sample_number;
}

bool ProximitySensorMessageHeader::operator!=(ProximitySensorMessageHeader const &other) const
{
     return !(*this == other);
}

std::ostream& operator<<(std::ostream& os, ProximitySensorMessageHeader const& p)
{
     os << "\nPreamble: " << std::string(p.preamble, sizeof(p.preamble)) << "\n"
        << "Order: " << p.order << "\n"
        << "Serial: " << p.serial << "\n"
        << "Channels: " << std::bitset<64>(p.channels) << "\n" // Print channels as a binary string
        << "Status: " << p.status << "\n"
        << "Frame count: " << p.frame_count << "\n"
        << "Bytes per frame: " << p.bytes_per_frame << "\n"
        << "Start sample number: " << p.start_sample_number;
     return os;
}

/*************************************
* ChannelValue
**************************************/

ChannelValue::ChannelValue(int sample_number, int raw_value)
{
     if (static_cast<uint32_t>(raw_value) > CHANNEL_VALUE_MASK) {
          throw std::out_of_range("Raw value is out of range (> 0x00FFFFFF)");
     }
     this->sample_number = sample_number;
     this->raw_value = raw_value;
}

double ChannelValue::to_micrometers(double measuring_range_micrometers) const
{
     // scale into [0 … measuring_range_micrometers]
     return (static_cast<double>(this->raw_value) / CHANNEL_VALUE_MASK) * measuring_range_micrometers;
}

uint32_t ChannelValue::from_micrometers(double micrometers, double measuring_range_micrometers)
{
     // scale into [0 … CHANNEL_VALUE_MASK]
     return static_cast<uint32_t>((micrometers / measuring_range_micrometers) * CHANNEL_VALUE_MASK);
}

bool ChannelValue::operator==(ChannelValue const &other) const
{
     return this->sample_number == other.sample_number &&
          this->raw_value == other.raw_value;
}

std::ostream& operator<<(std::ostream& os, ChannelValue const& c)
{
     os << "ChannelValue(no: " << c.sample_number << ", val: " << c.raw_value << ")";
     return os;
}

/*************************************
* ProximitySensorMessage
**************************************/

ProximitySensorMessage::ProximitySensorMessage(std::unique_ptr<ProximitySensorMessageHeader> header)
{
     if (!header) {
          throw std::invalid_argument("Header is null");
     }
     this->header = std::move(header);
}

bool ProximitySensorMessage::channel_exists(int channel) const
{
     return this->channel_values.count(channel) > 0;
}


ChannelValue ProximitySensorMessage::get_channel_value(int channel, int index) const
{
     auto it = this->channel_values.find(channel);
     if (it == this->channel_values.end())
     {
          throw std::invalid_argument("Channel does not exist");
     }
     auto const & vec = it->second;

     if (index >= (int)vec.size())
     {
          throw std::out_of_range("Index out of range");
     }

     return vec[index];
}

const std::vector<ChannelValue>& ProximitySensorMessage::get_channel_values(int channel) const
{
     if (!this->channel_exists(channel)) {
          throw std::invalid_argument("Channel does not exist");
     }

     return channel_values.at(channel);
}

void ProximitySensorMessage::set_channel_value(int channel, ChannelValue value)
{
     if (this->channel_exists(channel)) {
          this->channel_values[channel].push_back(value);
     } else {
          this->channel_values[channel] = { value };
     }
}

std::unique_ptr<ProximitySensorMessage> ProximitySensorMessage::decode
(
     std::unique_ptr<ProximitySensorMessageHeader> header,
     const char* buffer,
     size_t buffer_size,
     size_t offset_bytes
)
{
     if (!header) {
          throw std::invalid_argument("Header is null");
     }

     if (!buffer) {
          throw std::invalid_argument("Buffer is null");
     }

     if (buffer_size < offset_bytes) {
          throw std::out_of_range("Buffer size is smaller than offset");
     }

     if (buffer_size - offset_bytes < header->payload_size()) {
          throw std::out_of_range("Buffer does not contain enough space for the message");
     }

     // Create a new ProximitySensorMessage
     std::unique_ptr<ProximitySensorMessage> message = std::make_unique<ProximitySensorMessage>(std::move(header));

     offset_bytes += sizeof(ProximitySensorMessageHeader);

     std::vector<int> enabled_channels = message->header->enabled_channels();
     uint32_t current_sample_number = message->header->start_sample_number;

     // Copy the channel values from the buffer to the message
     // Each frame contains one value for each enabled channel
     for (int i = 0; i < message->header->frame_count; i++) {
          for (int channel : enabled_channels) {
               // The channel values are in little-endian format
               uint32_t le_value;
               std::memcpy(&le_value, buffer + offset_bytes, sizeof(le_value));
               // Convert to host order
               int h_value = le32toh(le_value);
               // Mask the value to get the channel value (24 bits)
               h_value &= CHANNEL_VALUE_MASK;
               // Create a new ChannelValue object
               ChannelValue channel_value(current_sample_number, h_value);
               // Store the channel value in the message
               message->set_channel_value(channel, channel_value);

               offset_bytes += sizeof(le_value);
               current_sample_number ++;
          }
     }

     return message;
}

void ProximitySensorMessage::encode(char* buffer, size_t buffer_size, size_t offset_bytes) const {
     if (!buffer) {
          throw std::invalid_argument("Buffer is null");
     }

     if (buffer_size < offset_bytes) {
          throw std::out_of_range("Buffer size is smaller than offset");
     }

     if (buffer_size - offset_bytes < this->header->payload_size()) {
          throw std::out_of_range("Buffer does not contain enough space for the message");
     }

     // Encode the header
     this->header->encode(buffer, buffer_size, offset_bytes);
     offset_bytes += sizeof(ProximitySensorMessageHeader);

     std::vector<int> enabled_channels = this->header->enabled_channels();

     // Each frame contains one value for each enabled channel
     for (int i = 0; i < this->header->frame_count; i++) {
          for (int channel : enabled_channels) {
               // Convert the host order value to little-endian format
               uint32_t le_value = htole32(this->get_channel_value(channel, i).raw_value);
               // Copy the little-endian value to the buffer
               std::memcpy(buffer + offset_bytes, &le_value, sizeof(le_value));
               // Increase offset by the size of the value
               offset_bytes += sizeof(le_value);
          }
     }
}


bool ProximitySensorMessage::operator==(ProximitySensorMessage const &other) const
{
     if (*this->header != *other.header) {
          return false;
     }

     if (this->channel_values.size() != other.channel_values.size()) {
          return false;
     }

     for (const auto& pair : this->channel_values) {
          auto it = other.channel_values.find(pair.first);
          if (it == other.channel_values.end()) {
               return false;
          }
          if (pair.second != it->second) {
               return false;
          }
     }

     return true;
}


std::ostream& operator<<(std::ostream& os, ProximitySensorMessage const& m)
{
     os << *m.header << "\n";
     for (const auto& pair : m.channel_values) {
          os << "Channel " << pair.first << ": ";
          for (const ChannelValue& value : pair.second) {
               os << value << " ";
          }
          os << "\n";
     }
     return os;
}

} // namespace proximity_sensor