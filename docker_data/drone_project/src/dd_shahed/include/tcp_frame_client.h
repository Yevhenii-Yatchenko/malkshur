// TCP Frame Client - C++ version for receiving frames from gazebo_frame_server.py
// Compatible with OpenCV VideoCapture API

#ifndef TCP_FRAME_CLIENT_H
#define TCP_FRAME_CLIENT_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

class TCPFrameClient {
private:
    std::string host_;
    int port_;
    int sockfd_;
    bool connected_;

    /**
     * Receive exactly n bytes
     */
    bool recvExactly(void* buffer, size_t n) {
        size_t total = 0;
        char* buf = static_cast<char*>(buffer);

        while (total < n) {
            ssize_t received = recv(sockfd_, buf + total, n - total, 0);
            if (received <= 0) {
                if (received == 0) {
                    std::cerr << "[TCP-FRAME] Server closed connection" << std::endl;
                } else {
                    std::cerr << "[TCP-FRAME] recv error: " << strerror(errno) << std::endl;
                }
                return false;
            }
            total += received;
        }
        return true;
    }

public:
    TCPFrameClient(const std::string& host = "host.docker.internal", int port = 5001)
        : host_(host), port_(port), sockfd_(-1), connected_(false) {}

    ~TCPFrameClient() {
        release();
    }

    /**
     * Connect to TCP frame server
     */
    bool connect();

    /**
     * Read next frame from TCP stream
     * @param frame Output frame
     * @return true if frame read successfully
     */
    bool read(cv::Mat& frame);

    /**
     * Check if connection is open
     */
    bool isOpened() const {
        return connected_;
    }

    /**
     * Close connection
     */
    void release();

    /**
     * For compatibility with VideoCapture API
     */
    bool get(int propId) {
        return false;
    }
};

#endif // TCP_FRAME_CLIENT_H

