# Micro Epsilon capa NCDT reader

This is a simple c++ library created to read data from supported proximity sensors connected to [Micro Epsilon's capa NCDT](https://www.micro-epsilon.com/distance-sensors/capacitive-sensors/) control units. The control unit communication is done over a network connection using TCP/IP.

It was specifically developed to read data from the capa NCDT 6200 control units, but it should work with other capa NCDT control units if they adhere to the same messaging protocol.

To ensure your control unit is supported, please refer to the [Micro Epsilon manuals](https://www.micro-epsilon.com/distance-sensors/capacitive-sensors/capancdt-6200/) for more information on the messaging protocol.

## Features

- Read data from supported proximity sensors
- Support for multiple channels, allowing simultaneous readings from multiple sensors
- Configurable read rate
- Simple and easy-to-use API

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
   mkdir build
   cd build
   # use flag -DBUILD_TESTING=ON to build the tests
   cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
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

Using the library is straightforward. This example demonstrates how to read data from a proximity sensor connected to a capa NCDT control unit.

Ensure you provide the correct measuring range of the sensor you are using according to it's specifications. For instance the CS02 sensor has a measuring range of 0-2mm, hence the measuring range is 2000 in micrometers.

The channel number is the number of the channel your sensor is connected to on the control unit, starting from 1. The capaNCDT 6200 control unit for instance supports up to 4 channels, all of which can be used simultaneously.

For more information on the library interface, please refer to the [header files](include/capancdt/proximity_sensor.h).

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

## Provided verification binary

The library provides a standalone binary that can be used to read data from the capa NCDT control unit and output the measurements to the console. After installing the library, the binary can be found in the `bin` directory. The binary is called `capancdt_read_sensor` amd can be used as follows:

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

Install GTest for testing. On ubuntu you can do this with:

```bash
sudo apt-get install libgtest-dev
```

Follow the installation instructions above to set up the project. The library uses CMake for building and testing. To run the tests, ensure you have the `BUILD_TESTING` option enabled in your CMake configuration.

Testing:

> make test
