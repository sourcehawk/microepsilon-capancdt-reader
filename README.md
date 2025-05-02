[![codecov](https://codecov.io/gh/sourcehawk/microepsilon-capancdt-reader/graph/badge.svg?token=2BXAU5R0RQ)](https://codecov.io/gh/sourcehawk/microepsilon-capancdt-reader)

# Micro Epsilon capa NCDT reader

This is a simple c++ library created to read data from supported proximity sensors connected to [Micro Epsilon's capa NCDT](https://www.micro-epsilon.com/distance-sensors/capacitive-sensors/) control units. The control unit communication is done over a network connection using TCP/IP.

It was specifically developed to read data from the capa NCDT 6200 control units, but it should work with other capa NCDT control units if they adhere to the same messaging protocol.

To ensure your control unit is supported, please refer to the [Micro Epsilon manuals](https://www.micro-epsilon.com/distance-sensors/capacitive-sensors/capancdt-6200/) and compare the messaging protocol with the one that NCDT 6200 uses.

## Features

- Read data from supported proximity sensors
- Support for multiple channels, allowing simultaneous readings from multiple sensors
- Configurable read rate
- Simple and easy-to-use API

## Compatibility

The code is compatible with C++11 and later versions. It has been tested on Linux systems and may need modifications to work on other platforms such as Windows or macOS. More specifically, the socket connection code may need to be adapted for non-POSIX system, but that should be as simple as including the correct headers and using the correct socket functions.

The CI tests are running on Ubuntu 22.04 and 24.04, but the code has also been compiled and executed on ubuntu 18.04.

## Installation

1. Clone the repository:

   ```bash
   git clone git@github.com:sourcehawk/microepsilon-capancdt-reader.git
   ```

2. Navigate to the project directory:

   ```bash
   cd microepsilon_capa_ncdt_reader
   ```

3. Build the project:

   ```bash
   cmake -S . -B build
   cd build
   make
   ```

4. Install the library:

   ```bash
   sudo make install
   ```

## Usage

Before using the library, install it by following the [installation instructions](#installation) and then add the package to you CMake project:

```cmake
cmake_minimum_required(VERSION 3.10)
project(your_project)

# find the package
find_package(capancdt_proximity_sensor REQUIRED)

# your executable
add_executable(my_app src/main.cpp)

# link the library to your executable
target_link_libraries(my_app
  PRIVATE capancdt_proximity_sensor::capancdt_proximity_sensor_lib
)
```

Using the library is straightforward. The below example demonstrates how to read data from a proximity sensor connected to a capa NCDT control unit.

Ensure you provide the correct measuring range of the sensor you are using according to it's specifications. For instance the CS02 sensor has a measuring range of 0-2mm, hence the measuring range is 2000 in micrometers.

The channel number is the number of the channel your sensor is connected to on the control unit, starting from 1. The capaNCDT 6200 control unit for instance supports up to 4 channels, all of which can be used simultaneously.

For more information on the library interface, please refer to the [header files](include/capancdt/).

```cpp
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <capancdt/proximity_sensor.h>

using namespace proximity_sensor;

int main(int argc, char **argv)
 {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <ip> <port> <rate> <channel> <range>" << std::endl;
        return 1;
    }

    std::string ip = std::string(argv[1]);
    int port = std::stoi(argv[2]);
    double rate = std::stod(argv[3]);
    int channel = std::stoi(argv[4]);
    double range = std::stod(argv[5]);

    ProximityConnection* conn = new ProximitySocketConnection(ip, port);
    ProximitySensor* proximity_sensor = new ProximitySensor(conn, rate, 2.0);

    proximity_sensor->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    ProximitySensorStatus status = proximity_sensor->get_proximity_sensor_status();
    if (!status.running()) {
        std::cerr << "Failed to start the sensor: " << status.message << std::endl;
        return 1;
    }

    auto period = std::chrono::duration<double>(1.0 / rate);

    while (true)
    {
        std::cout << proximity_sensor->get_distance(channel, range) << std::endl;
        std::this_thread::sleep_for(period);
    }

    return 0;
}
```

## Mocking sensors in development

The library provides a mock implementation of the `ProximityConnection` class that can be used for testing and development purposes. This allows you to simulate the behavior of a real sensor without needing to connect to an actual device.

In this snippet, we create a mock connection, construct a mock message, and encode it to the mock data buffer to be returned by the mock connection on recv calls.

```cpp
ProximityConnection* conn;

if (mock) {
    ProximityMockConnection* mock_conn = new ProximityMockConnection();

    // Create a mock message header
    ProximitySensorMessageHeader mock_header {};
    memcpy(mock_header.preamble, MESSAGE_HEADER_PREAMBLE_C, sizeof(mock_header.preamble));
    mock_header.order               = 69;
    mock_header.serial              = 12345;
    mock_header.channels            = 0x0000'0000'0000'0001u << ((channel-1)*2);
    mock_header.status              = 0;
    mock_header.frame_count         = 1;
    mock_header.bytes_per_frame     = 4;
    mock_header.start_sample_number = 1;

    // Create a buffer to hold the encoded message
    size_t payload_size = mock_header.payload_size();
    char* messages = new char[payload_size];

    // Create a mock message with the header
    auto header_ptr = std::make_unique<ProximitySensorMessageHeader>(mock_header);
    ProximitySensorMessage mock_message = ProximitySensorMessage(std::move(header_ptr));
    mock_message.set_channel_value(1, ChannelValue(
        mock_header.start_sample_number,
        // This is the value that will be returned by the mock sensor
        ChannelValue::from_micrometers(2000.0, range)
    ));

    // Encode the mock message to the messages buffer
    mock_message.encode(messages, payload_size);
    // Set the mock data to return on recv calls
    mock_conn->set_recv_data(messages, payload_size);
    // Assign the mock connection to the abstract connection
    conn = mock_conn;
} else {
    conn = new ProximitySocketConnection(ip, port);
}

ProximitySensor* proximity_sensor = new ProximitySensor(conn, rate, 2.0);
```

## Provided verification binary

The library provides a standalone binary that can be used to read data from the capa NCDT control unit and output the measurements to the console. After installing the library, the binary can be found in the `bin` directory (f.x `/usr/local/bin/`). The binary is called `capancdt_read_sensor` and can be used as follows:

```bash
./capancdt_read_sensor <ip> <port> <rate> <channel> <range>
```

Where:

- `<ip>`: The IP address of the capa NCDT control unit.
- `<port>`: The port number of the capa NCDT control unit.
- `<rate>`: The read rate in Hz (e.g., 10.0 for 10 Hz).
- `<channel>`: The channel number of the sensor (1, 2, 3, 4...).
- `<range>`: The measuring range of the sensor in micrometers (e.g., 2000 for a 0-2mm sensor).

## Development

Install GTest and lcov for testing. On ubuntu you can do this with:

```bash
sudo apt-get install libgtest-dev lcov
```

You may need to build GTest:

```bash
cd /usr/src/googletest
sudo cmake .
sudo make
sudo make install
```

Build the project:

```bash
cmake -S . -B build \
  -DENABLE_COVERAGE=ON \
  -DBUILD_TESTING=ON \
  -DCMAKE_BUILD_TYPE=Debug

cmake --build build --parallel
```

Testing:

> cd build && make test
