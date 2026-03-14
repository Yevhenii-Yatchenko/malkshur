#include "tcp_frame_client.h"

bool TCPFrameClient::connect() {
    if (connected_) {
        return true;
    }

    sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd_ < 0) {
        std::cerr << "[TCP-FRAME ERROR] socket() failed: " << strerror(errno) << std::endl;
        return false;
    }

    struct timeval tv;
    tv.tv_sec = 10;
    tv.tv_usec = 0;
    setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    struct addrinfo* result = nullptr;
    std::string portStr = std::to_string(port_);
    int s = getaddrinfo(host_.c_str(), portStr.c_str(), &hints, &result);
    if (s != 0) {
        std::cerr << "[TCP-FRAME ERROR] getaddrinfo failed: " << gai_strerror(s) << std::endl;
        close(sockfd_);
        sockfd_ = -1;
        return false;
    }

    bool ok = false;
    for (struct addrinfo* rp = result; rp != nullptr; rp = rp->ai_next) {
        if (::connect(sockfd_, rp->ai_addr, rp->ai_addrlen) == 0) {
            ok = true;

            char ipstr[INET6_ADDRSTRLEN];
            void* addr_ptr = nullptr;
            if (rp->ai_family == AF_INET) {
                struct sockaddr_in* ipv4 = (struct sockaddr_in*)rp->ai_addr;
                addr_ptr = &(ipv4->sin_addr);
                inet_ntop(rp->ai_family, addr_ptr, ipstr, sizeof(ipstr));
                std::cout << "[TCP-FRAME] Connected to Gazebo frame server at "
                          << ipstr << ":" << port_ << std::endl;
            }
            break;
        }
    }

    freeaddrinfo(result);

    if (!ok) {
        std::cerr << "[TCP-FRAME ERROR] connect() failed: " << strerror(errno) << std::endl;
        close(sockfd_);
        sockfd_ = -1;
        return false;
    }

    connected_ = true;
    return true;
}

bool TCPFrameClient::read(cv::Mat& frame) {
    if (!connected_) {
        return false;
    }

    try {
        // 1. Read size (4 bytes big-endian)
        uint32_t size_be;
        if (!recvExactly(&size_be, 4)) {
            connected_ = false;
            return false;
        }

        uint32_t size = ntohl(size_be);

        // Sanity check (JPEG cannot be larger than 10MB)
        if (size == 0 || size > 10 * 1024 * 1024) {
            std::cerr << "[TCP-FRAME ERROR] Invalid frame size: " << size << std::endl;
            std::cerr << "[TCP-FRAME ERROR] This likely means protocol mismatch!" << std::endl;
            std::cerr << "[TCP-FRAME ERROR] Expected: gazebo_frame_server.py protocol" << std::endl;
            std::cerr << "[TCP-FRAME ERROR] Make sure you're connecting to the right port (default: 5001)" << std::endl;
            connected_ = false;
            return false;
        }

        // 2. Read JPEG data
        std::vector<uint8_t> jpegData(size);
        if (!recvExactly(jpegData.data(), size)) {
            connected_ = false;
            return false;
        }

        // 3. Decode JPEG
        frame = cv::imdecode(jpegData, cv::IMREAD_COLOR);

        if (frame.empty()) {
            std::cerr << "[TCP-FRAME ERROR] Failed to decode JPEG" << std::endl;
            return false;
        }

        return true;

    } catch (const std::exception& e) {
        std::cerr << "[TCP-FRAME ERROR] Exception: " << e.what() << std::endl;
        connected_ = false;
        return false;
    }
}

void TCPFrameClient::release() {
    if (sockfd_ >= 0) {
        close(sockfd_);
        sockfd_ = -1;
    }
    connected_ = false;
}

