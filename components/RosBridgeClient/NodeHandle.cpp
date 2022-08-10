#include "NodeHandle.h"

#include "esp_log.h"

namespace ros
{   
    #define TAG "NodeHandle"

    NodeHandle* NodeHandle::_node_handle_obj = nullptr;


    NodeHandle::NodeHandle(std::string const& ros_namespace, Socket& sock) : _sock{sock}, _ros_namespace{ros_namespace}
    {   
        _keep_alive_time_us = esp_timer_get_time();
        _last_send_keep_alive_us = esp_timer_get_time();
        
        _send_init();

        xTaskCreate(_communication_handler, "_communication_handler", 8192, this, 5, &_communication_handler_thread);
    }

    NodeHandle::~NodeHandle()
    {   
        vTaskDelete(_communication_handler_thread);


        for(auto i : _subscriber)
            delete i;

        for(auto i : _publisher)
            delete i;
    }

    NodeHandle& NodeHandle::init(std::string const& ros_namespace, Socket& sock)
    {
        if(_node_handle_obj == nullptr)
            _node_handle_obj = new NodeHandle(ros_namespace, sock);

        return *_node_handle_obj;
    }

    void NodeHandle::registerConnectionLostCallback(std::function<void()> connection_lost_callback)
    {
        _connection_lost_callback.push_back(connection_lost_callback);
    }

    void NodeHandle::_restart_protocol()
    {   
        ESP_LOGI(TAG, "Restarting Protocol!");

        for(ros::PublisherInterface* i : _publisher)
            i->block_publishing();

        for(auto i : _connection_lost_callback)
            i();

        _sock.disconnect_socket();
        vTaskDelay((2000 + MAX_KEEP_ALIVE_TIMOUT_MS)  / portTICK_PERIOD_MS);
        _sock.connect_socket();

        _send_init();
        _keep_alive_time_us = esp_timer_get_time();

        for(ros::Subscriber* i : _subscriber)
            i->subscribe();

        for(ros::PublisherInterface* i : _publisher)
        {
            i->advertise();
            i->unblock_publishing();
        }
    }

    int NodeHandle::_send_init()
    {
        uint8_t pkt_buffer[_ros_namespace.size() + 2];
        int pkt_len = 0;

        pkt_buffer[0] = INIT_ID;
        pkt_len++;

        memcpy(pkt_buffer + pkt_len, _ros_namespace.c_str(), _ros_namespace.size());
        pkt_len += _ros_namespace.size();

        pkt_buffer[pkt_len] = '\0';
        pkt_len++;

        ESP_LOGI(TAG, "Sending initialize message!");

        return _sock.socket_send(pkt_buffer, pkt_len);
    }

    void NodeHandle::_send_keep_alive()
    {   
        uint64_t time_now_us = esp_timer_get_time();

        if((time_now_us - _last_send_keep_alive_us) / 1000 > KEEP_ALIVE_SEND_PERIOD_MS)
        {
            uint8_t pkt_buffer[1 + sizeof(uint64_t)];
            int pkt_len = 0;

            pkt_buffer[0] = KEEP_ALIVE_ID;
            pkt_len++;

            *(uint64_t*)(pkt_buffer + pkt_len) = (uint64_t)time_now_us;
            pkt_len += sizeof(uint64_t);

            _last_send_keep_alive_us = time_now_us;

            _sock.socket_send(pkt_buffer, pkt_len);
        }
    }

    void NodeHandle::_communication_handler(void* arg)
    {   
        NodeHandle* node_handle = (NodeHandle*)arg;

        while(1)
        {      
            node_handle->_send_keep_alive();   

            if(node_handle->_interpret_receive() == SOCKET_FAIL)
            {
                ESP_LOGE(TAG,"Interpret Receive failed!");
                node_handle->_restart_protocol();
            }

            if((esp_timer_get_time() - node_handle->_keep_alive_time_us) / 1000 > MAX_KEEP_ALIVE_TIMOUT_MS)
            {
                ESP_LOGE(TAG, "Check Keep Alive Timeout!");
                ESP_LOGE(TAG, "Current Time: %lld, Last Keep_Alive: %lld", esp_timer_get_time(), node_handle->_keep_alive_time_us);
                node_handle->_restart_protocol();
            }
            
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
    
        vTaskDelete(NULL);
    }


    int NodeHandle::_interpret_receive()
    {   
        int status_error = 0;

        while(1)
        {

            uint8_t msg_id;

            status_error = _sock.socket_receive_nonblock(&msg_id, 1);

            if(status_error == SOCKET_FAIL)
            {
                ESP_LOGE(TAG, "Error while receiving MSG ID (errno: %d)", errno);
                return status_error;
            } 
            else if (status_error == 0)
            {
                return status_error;
            }

            switch(msg_id)
            {   
                case KEEP_ALIVE_ID:
                {   
                    uint64_t server_time;
                    status_error = _sock.socket_receive((uint8_t*)&server_time, sizeof(server_time));

                    if(status_error == SOCKET_FAIL)
                        break;

                    _keep_alive_time_us = esp_timer_get_time();

                    _server_time_difference_us = server_time - _keep_alive_time_us;

                    ESP_LOGI(TAG, "Keep Alive! Server Time: %lld", server_time);

                    break;
                }
                case PUBLISH_ID:
                {   
                    std::string topic;
                    status_error = _sock.socket_receive_string(topic, MAX_TOPIC_LENGTH);

                    if(status_error == SOCKET_FAIL)
                        break;

                    //ESP_LOGI(TAG, "Received topic: %s", topic.c_str());
                    
                    Subscriber* sub = _getSubscriber(topic);

                    if(sub != nullptr)
                    {
                        if(sub->recvMessage() == false)
                            status_error = SOCKET_FAIL;
                    }
                    else
                        status_error = SOCKET_FAIL;

                    break;
                }
                default:
                    ESP_LOGE(TAG, "ID not found: %d", msg_id);
                    status_error = SOCKET_FAIL;
                    break;
            }

            if(status_error == SOCKET_FAIL)
                break;

        }

        return status_error;
    }

    Subscriber* NodeHandle::_getSubscriber(std::string const& topic)
    {
        for(auto i : _subscriber)
        {
            if(i->compareTopic(topic))
                return i;
        }

        return nullptr;
    }
}