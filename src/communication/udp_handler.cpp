#include "udp_handler.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <stdexcept>
#include "../messages/message_types.h"

UDPHandler::UDPHandler(int receive_port, const std::string& send_ip, int send_port)
    : m_receive_port(receive_port), m_send_ip(send_ip), m_send_port(send_port) {}

UDPHandler::~UDPHandler() {
    stop();
    if (m_socket_fd != -1) {
        close(m_socket_fd);
    }
}

bool UDPHandler::start(MessageCallback callback) {
    if (m_running) {
        std::cerr << "UDP handler already running." << std::endl;
        return false;
    }

    m_callback = callback;

    m_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_socket_fd < 0) {
        perror("socket creation failed");
        return false;
    }

    sockaddr_in receive_addr;
    memset(&receive_addr, 0, sizeof(receive_addr));
    receive_addr.sin_family = AF_INET;
    receive_addr.sin_addr.s_addr = INADDR_ANY;
    receive_addr.sin_port = htons(m_receive_port);

    if (bind(m_socket_fd, (const struct sockaddr *)&receive_addr, sizeof(receive_addr)) < 0) {
        perror("bind failed");
        close(m_socket_fd);
        m_socket_fd = -1;
        return false;
    }

    memset(&m_send_addr, 0, sizeof(m_send_addr));
    m_send_addr.sin_family = AF_INET;
    m_send_addr.sin_port = htons(m_send_port);
    if (inet_pton(AF_INET, m_send_ip.c_str(), &m_send_addr.sin_addr) <= 0) {
        perror("Invalid address/ Address not supported");
        close(m_socket_fd);
        m_socket_fd = -1;
        return false;
    }
    
    m_running = true;
    m_receive_thread = std::thread(&UDPHandler::receive_loop, this);
    
    std::cout << "UDP Handler started. Listening on port " << m_receive_port
              << ", Sending to " << m_send_ip << ":" << m_send_port << std::endl;
    return true;
}

void UDPHandler::stop() {
    m_running = false;
    if (m_receive_thread.joinable()) {
        m_receive_thread.join();
    }
}

bool UDPHandler::send(const std::string& message) {
    if (m_socket_fd < 0) {
        return false;
    }
    if (message.length() > MAX_PACKET_SIZE) {
        std::cerr << "Error: Message size (" << message.length() << " bytes) exceeds MAX_PACKET_SIZE (" << MAX_PACKET_SIZE << " bytes)." << std::endl;
        return false;
    }

    ssize_t bytes_sent = sendto(m_socket_fd, message.c_str(), message.length(), 0,
                                (const struct sockaddr *)&m_send_addr, sizeof(m_send_addr));

    return bytes_sent == (ssize_t)message.length();
}

void UDPHandler::receive_loop() {
    char buffer[MAX_PACKET_SIZE];
    sockaddr_in cliaddr;
    socklen_t len = sizeof(cliaddr);

    while (m_running) {
        ssize_t n = recvfrom(m_socket_fd, buffer, MAX_PACKET_SIZE, MSG_WAITALL,
                             (struct sockaddr *)&cliaddr, &len);
        
        if (n > 0) {
            std::string received_message(buffer, n);
            if (m_callback) {
                m_callback(received_message);
            }
        }
    }
}
