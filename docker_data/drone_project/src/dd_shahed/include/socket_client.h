#ifndef SOCKET_CLIENT_H
#define SOCKET_CLIENT_H

#include <string>

/**
 * Send JSON data via UDP
 * @param host Target hostname or IP
 * @param port Target port
 * @param jsonStr JSON string to send
 * @return true if sent successfully
 */
bool sendJsonToServerUDP(const std::string& host, int port, const std::string& jsonStr);

/**
 * Send JSON data via TCP (persistent connection)
 * @param host Target hostname or IP
 * @param port Target port
 * @param jsonStr JSON string to send
 * @return true if sent successfully
 */
bool sendJsonToServerTCP(const std::string& host, int port, const std::string& jsonStr);

/**
 * Close TCP connection (cleanup)
 */
void closeTCPConnection();

/**
 * Get UDP statistics
 */
void getUDPStats(int& sent, int& failed);

/**
 * Get TCP statistics
 */
void getTCPStats(int& sent, int& failed);

/**
 * Enable/disable verbose logging for UDP
 */
void setUDPVerbose(bool verbose);

/**
 * Enable/disable verbose logging for TCP
 */
void setTCPVerbose(bool verbose);

#endif // SOCKET_CLIENT_H

