#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include "cannister.h"
#include <boost/asio.hpp>

using namespace boost::asio;

void sendCommandsToSerial(const std::string& port, const std::string text) {
    try {
        io_service io;
        serial_port serial(io, port);

        // Configure serial port
        serial.set_option(serial_port_base::baud_rate(115200));
        serial.set_option(serial_port_base::character_size(8));
        serial.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
        serial.set_option(serial_port_base::parity(serial_port_base::parity::none));
        serial.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));

        write(serial, buffer(text.c_str(), text.size()));
        serial.close();
            // std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    catch (std::exception& e) {
        std::cerr << "Error sending commands: " << e.what() << std::endl;
    }
}
