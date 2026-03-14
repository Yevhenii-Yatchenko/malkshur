#include "socket_client.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <iostream>

// UDP globals
static bool g_udp_verbose = false;
static int g_udp_sent_count = 0;
static int g_udp_fail_count = 0;

// TCP globals for persistent connection
static int g_tcp_sockfd = -1;
static std::string g_tcp_last_host = "";
static int g_tcp_last_port = -1;
static int g_tcp_sent_count = 0;
static int g_tcp_fail_count = 0;
static bool g_tcp_verbose = false;

bool sendJsonToServerUDP(const std::string& host, int port, const std::string& jsonStr) {
    try {
        int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0) {
            std::cerr << "[SOCKET-UDP ERROR] socket() failed: " << strerror(errno) << std::endl;
            g_udp_fail_count++;
            return false;
        }

        struct addrinfo hints{};
        hints.ai_family   = AF_UNSPEC;
        hints.ai_socktype = SOCK_DGRAM;

        struct addrinfo* result = nullptr;
        std::string portStr = std::to_string(port);
        int s = getaddrinfo(host.c_str(), portStr.c_str(), &hints, &result);
        if (s != 0) {
            std::cerr << "[SOCKET-UDP ERROR] getaddrinfo(" << host << ":" << port
                      << ") failed: " << gai_strerror(s) << std::endl;
            close(sockfd);
            g_udp_fail_count++;
            return false;
        }

        bool ok = false;
        std::string resolved_addr;
        for (struct addrinfo* rp = result; rp != nullptr; rp = rp->ai_next) {
            char ipstr[INET6_ADDRSTRLEN];
            void* addr_ptr = nullptr;
            if (rp->ai_family == AF_INET) {
                struct sockaddr_in* ipv4 = (struct sockaddr_in*)rp->ai_addr;
                addr_ptr = &(ipv4->sin_addr);
                inet_ntop(rp->ai_family, addr_ptr, ipstr, sizeof(ipstr));
                resolved_addr = std::string(ipstr);
            } else if (rp->ai_family == AF_INET6) {
                struct sockaddr_in6* ipv6 = (struct sockaddr_in6*)rp->ai_addr;
                addr_ptr = &(ipv6->sin6_addr);
                inet_ntop(rp->ai_family, addr_ptr, ipstr, sizeof(ipstr));
                resolved_addr = std::string("[") + ipstr + "]";
            }

            ssize_t sent = sendto(sockfd,
                                  jsonStr.data(),
                                  jsonStr.size(),
                                  0,
                                  rp->ai_addr,
                                  rp->ai_addrlen);

            if (sent == (ssize_t)jsonStr.size()) {
                ok = true;
                g_udp_sent_count++;
                if (g_udp_verbose || g_udp_sent_count == 1) {
                    std::cout << "[SOCKET-UDP OK] Sent " << sent << " bytes to "
                              << resolved_addr << ":" << port
                              << " (total sent: " << g_udp_sent_count << ")" << std::endl;
                }
                break;
            } else {
                std::cerr << "[SOCKET-UDP ERROR] sendto(" << resolved_addr << ":" << port
                          << ") sent " << sent << "/" << jsonStr.size()
                          << " bytes, errno: " << strerror(errno) << std::endl;
            }
        }

        freeaddrinfo(result);
        close(sockfd);

        if (!ok) {
            g_udp_fail_count++;
            std::cerr << "[SOCKET-UDP ERROR] All sendto() attempts failed. "
                      << "Failed count: " << g_udp_fail_count << std::endl;
        }
        return ok;
    } catch (const std::exception& e) {
        std::cerr << "[SOCKET-UDP Exception] " << e.what() << std::endl;
        g_udp_fail_count++;
        return false;
    }
}

bool connectToTCPServer(const std::string& host, int port) {
    if (g_tcp_sockfd >= 0 && g_tcp_last_host == host && g_tcp_last_port == port) {
        return true;
    }

    if (g_tcp_sockfd >= 0) {
        close(g_tcp_sockfd);
        g_tcp_sockfd = -1;
    }

    g_tcp_sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (g_tcp_sockfd < 0) {
        std::cerr << "[SOCKET-TCP ERROR] socket() failed: " << strerror(errno) << std::endl;
        return false;
    }

    struct timeval tv;
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    setsockopt(g_tcp_sockfd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    struct addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    struct addrinfo* result = nullptr;
    std::string portStr = std::to_string(port);
    int s = getaddrinfo(host.c_str(), portStr.c_str(), &hints, &result);
    if (s != 0) {
        std::cerr << "[SOCKET-TCP ERROR] getaddrinfo(" << host << ":" << port
                  << ") failed: " << gai_strerror(s) << std::endl;
        close(g_tcp_sockfd);
        g_tcp_sockfd = -1;
        return false;
    }

    bool connected = false;
    std::string resolved_addr;
    for (struct addrinfo* rp = result; rp != nullptr; rp = rp->ai_next) {
        char ipstr[INET6_ADDRSTRLEN];
        void* addr_ptr = nullptr;
        if (rp->ai_family == AF_INET) {
            struct sockaddr_in* ipv4 = (struct sockaddr_in*)rp->ai_addr;
            addr_ptr = &(ipv4->sin_addr);
            inet_ntop(rp->ai_family, addr_ptr, ipstr, sizeof(ipstr));
            resolved_addr = std::string(ipstr);
        } else if (rp->ai_family == AF_INET6) {
            struct sockaddr_in6* ipv6 = (struct sockaddr_in6*)rp->ai_addr;
            addr_ptr = &(ipv6->sin6_addr);
            inet_ntop(rp->ai_family, addr_ptr, ipstr, sizeof(ipstr));
            resolved_addr = std::string("[") + ipstr + "]";
        }

        if (connect(g_tcp_sockfd, rp->ai_addr, rp->ai_addrlen) == 0) {
            connected = true;
            std::cout << "[SOCKET-TCP] Connected to " << resolved_addr << ":" << port << std::endl;
            break;
        }
    }

    freeaddrinfo(result);

    if (!connected) {
        std::cerr << "[SOCKET-TCP ERROR] connect() failed: " << strerror(errno) << std::endl;
        close(g_tcp_sockfd);
        g_tcp_sockfd = -1;
        return false;
    }

    g_tcp_last_host = host;
    g_tcp_last_port = port;
    return true;
}

bool sendJsonToServerTCP(const std::string& host, int port, const std::string& jsonStr) {
    if (!connectToTCPServer(host, port)) {
        g_tcp_fail_count++;
        return false;
    }

    ssize_t sent = send(g_tcp_sockfd, jsonStr.data(), jsonStr.size(), MSG_NOSIGNAL);

    if (sent != (ssize_t)jsonStr.size()) {
        std::cerr << "[SOCKET-TCP ERROR] send() failed: " << strerror(errno)
                  << " (sent " << sent << "/" << jsonStr.size() << " bytes)" << std::endl;

        close(g_tcp_sockfd);
        g_tcp_sockfd = -1;
        g_tcp_fail_count++;
        return false;
    }

    g_tcp_sent_count++;
    if (g_tcp_verbose || g_tcp_sent_count == 1) {
        std::cout << "[SOCKET-TCP OK] Sent " << sent << " bytes (total: "
                  << g_tcp_sent_count << ")" << std::endl;
    }

    return true;
}

void closeTCPConnection() {
    if (g_tcp_sockfd >= 0) {
        close(g_tcp_sockfd);
        g_tcp_sockfd = -1;
    }
}

void getUDPStats(int& sent, int& failed) {
    sent = g_udp_sent_count;
    failed = g_udp_fail_count;
}

void getTCPStats(int& sent, int& failed) {
    sent = g_tcp_sent_count;
    failed = g_tcp_fail_count;
}

void setUDPVerbose(bool verbose) {
    g_udp_verbose = verbose;
}

void setTCPVerbose(bool verbose) {
    g_tcp_verbose = verbose;
}

