#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

#include <string>
#include <vector>
#include <cstring>


enum SOCKET_STATUS 
{
    SOCKET_FAIL = -1,
    SOCKET_OK = 1
};

/**
 * @brief This class serves as an abstraction layer for the BSD Socket API.
 * It creates a TCP socket and connects with the ROS Robot Server.
 * Internally the socket is set to nonblocking mode.
 */
class Socket 
{          
    public:
        Socket(int port, std::string ip_addr);
        ~Socket();

        /**
         * @brief Send the specified amount of bytes to the server.
         * This method will block until it can write all data to the socket 
         * or another task is currently executing this method.
         * 
         * @param [in] tx_buffer 
         * @param [in] buffer_len
         * @return the amount of bytes sent if succesfull, SOCKET_FAIL if internal send() fails
         */
        int socket_send(uint8_t const* tx_buffer, int buffer_len);

        /**
         * @brief Receive data from the server.
         * This function will block until the specified amount of bytes are retrieved.
         * 
         * @param [out] rx_buffer
         * @param [in] recv_bytes
         * @return the amount of bytes received if succesfull, SOCKET_FAIL if internal recv() fails and recv would not block
         */
        int socket_receive(uint8_t* rx_buffer, int recv_bytes);

        /**
         * @brief Receive data from the server.
         * This method will not block.
         * 
         * @param [out] rx_buffer
         * @param [in] recv_bytes
         * @return the amount of bytes received if succesfull, SOCKET_FAIL if internal recv() fails and recv would not block
         */
        int socket_receive_nonblock(uint8_t* rx_buffer, int recv_bytes);

        /**
         * @brief Receive until '\0' character or the specified amount of bytes is received.
         * This function will block until the above condition is met.
         * 
         * @param [out] rx_string
         * @param [in] max_bytes
         * @return the amount of bytes received if succesfull, SOCKET_FAIL if internal recv() fails and recv would not block
         */
        int socket_receive_string(std::string& rx_string, int max_bytes);

        /**
         * @brief This method creates a TCP Socket and connects with the Server.
         * This function will only return when it succesfully connected with a server.
         */
        void connect_socket();  

        /**
         * @brief Close the socket connection to the server.
         */  
        void disconnect_socket();

    private:
        Socket(const Socket&) = delete;

        int _connection_fd;

        int const _socket_port;
        std::string _ip_addr;

        struct sockaddr_in _server_socket;

        //prevents multiple tasks from accessing send() simultaneously
        SemaphoreHandle_t _send_mutx;
};
