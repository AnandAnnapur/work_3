#pragma once
#include <boost/asio.hpp>
#include <string>

// Declare globals
extern boost::asio::io_context io;
extern boost::asio::serial_port serial;

// Functions
void sendCommandsToSerial(const std::string& port, const std::string text);
