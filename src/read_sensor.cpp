#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include "capancdt/proximity_sensor.h"

/*
 * A simple binary you can copy to a device and run independently to test the sensor
 *
 * First build the project, then copy the binary to the device:
 * catkin build dte_proximity_sensor
 * scp devel/.private/dte_proximity_sensor/lib/dte_proximity_sensor/read_sensor nordural_gr_p1:/~read_sensor
 * 
 * Then run the binary on the device:
 * ssh nordural_gr_p1
 * sudo su
 * cd /home/svc_ansible
 * chmod +x read_sensor
 * ./read_sensor 10.150.1.6 10001 104 1 5000
 */

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

    if (ip.empty()) {
        std::cerr << "Invalid IP address: " << ip << std::endl;
        return 1;
    }

    if (port <= 0 || port > 65535) {
        std::cerr << "Invalid port number: " << port << ". Port must be between 1 and 65535." << std::endl;
        return 1;
    }

    if (rate <= 0) {
        std::cerr << "Invalid rate: " << rate << ". Rate must be greater than 0." << std::endl;
        return 1;
    }

    if (channel <= 0) {
        std::cerr << "Invalid channel: " << channel << ". Channel must be greater than 0." << std::endl;
        return 1;
    }

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
