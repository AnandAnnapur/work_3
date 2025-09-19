#pragma once

#include <string>
#include <thread>
#include <functional>
#include <atomic>
#include <sys/socket.h>
#include <netinet/in.h>

class UDPHandler {
public:
    using MessageCallback = std::function<void(const std::string&)>;

    UDPHandler(int receive_port, const std::string& send_ip, int send_port);
    ~UDPHandler();

    bool start(MessageCallback callback);
    void stop();
    bool send(const std::string& message);

private:
    void receive_loop();

    int m_receive_port;
    std::string m_send_ip;
    int m_send_port;

    int m_socket_fd = -1;
    sockaddr_in m_send_addr;
    
    std::thread m_receive_thread;
    std::atomic<bool> m_running{false};
    MessageCallback m_callback;
};
