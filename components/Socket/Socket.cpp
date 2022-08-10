#include "Socket.h"

#include "esp_log.h"

#define TAG "Socket"

Socket::Socket(int port, std::string ip_addr) : _socket_port{port}, _ip_addr{ip_addr}
{
    _server_socket.sin_addr.s_addr = inet_addr(ip_addr.c_str());
    _server_socket.sin_family = AF_INET;
    _server_socket.sin_port = htons(port);

    _send_mutx = xSemaphoreCreateMutex();

    connect_socket();
}

Socket::~Socket()
{
    disconnect_socket();
    vSemaphoreDelete(_send_mutx);
}

void Socket::connect_socket()
{   
    int i = 0;

    while(1) 
    {
        _connection_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

        if(_connection_fd != -1)
        {
            ESP_LOGI(TAG, "Socket created, connecting to %s:%d", _ip_addr.c_str(), _socket_port);

            int err = connect(_connection_fd, (struct sockaddr*)&_server_socket, sizeof(struct sockaddr_in));
            if (!err) 
            {
                ESP_LOGI(TAG, "Successfully connected to %d", _socket_port);
                
                int flags = fcntl(_connection_fd, F_GETFL);
                fcntl(_connection_fd, F_SETFL, flags | O_NONBLOCK);

                break;
            }

            ESP_LOGE(TAG, "Socket (%d) unable to connect: errno %d", _socket_port, errno);
            disconnect_socket();
        } 
        else 
            ESP_LOGE(TAG, "Unable to create socket (%d): errno %d", _socket_port, errno);
        
        vTaskDelay( i*500 / portTICK_PERIOD_MS);
        i = (i < 8) ? i+1 : 10;   
    }   
}

void Socket::disconnect_socket() 
{
    if (_connection_fd != -1) {
        ESP_LOGE(TAG, "Shutting down socket (port: %d) and restarting...", _socket_port);
        close(_connection_fd);
    }
}

int Socket::socket_receive(uint8_t* rx_buffer, int recv_bytes)
{
    int bytes_read = 0;
    int len = 0;

    do
    {   
        len = recv(_connection_fd, rx_buffer + bytes_read, recv_bytes - bytes_read, 0);

        if(len == SOCKET_FAIL && errno == EWOULDBLOCK)
            len = 0;

        bytes_read += len;

        if(bytes_read < recv_bytes)
            vTaskDelay(1 / portTICK_PERIOD_MS);
    } 
    while (bytes_read < recv_bytes && len != SOCKET_FAIL);

    return bytes_read;
}


int Socket::socket_receive_string(std::string& rx_string, int max_bytes)
{
    int bytes_read = 0;
    int len = 0;
    char* rx_buffer = new char[max_bytes];

    while(bytes_read < max_bytes)
    {   
        len = recv(_connection_fd, rx_buffer + bytes_read, 1, 0);

        if(len == SOCKET_FAIL && errno == EWOULDBLOCK)
        {
            len = 0;
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }

        if(len == SOCKET_FAIL || rx_buffer[bytes_read] == '\0')
            break;
        else
            bytes_read += len;
    }

    if(len != SOCKET_FAIL)
        rx_string.assign(rx_buffer);
    else
        bytes_read = SOCKET_FAIL;
    
    return bytes_read;
}


int Socket::socket_receive_nonblock(uint8_t* rx_buffer, int recv_bytes)
{
    int len = recv(_connection_fd, rx_buffer, recv_bytes, 0);

    if(len == SOCKET_FAIL && errno == EWOULDBLOCK)
        len = 0;

    return len;
}

int Socket::socket_send(uint8_t const* tx_buffer, int buffer_len)
{   
    int len = 0;
    int bytes_sent = 0;

    if(xSemaphoreTake(_send_mutx, portMAX_DELAY) == pdPASS)
    {
        while(bytes_sent < buffer_len)
        {
            len = send(_connection_fd, tx_buffer + bytes_sent, buffer_len - bytes_sent, 0);

            if(len == SOCKET_FAIL && errno == EWOULDBLOCK)
                len = 0;
            else if(len == SOCKET_FAIL)
            {
                bytes_sent = SOCKET_FAIL;
                break;
            } 

            bytes_sent += len;

            if(bytes_sent < buffer_len)
                vTaskDelay(1 / portMAX_DELAY);
        }

        xSemaphoreGive(_send_mutx);
    }

    return bytes_sent;
}