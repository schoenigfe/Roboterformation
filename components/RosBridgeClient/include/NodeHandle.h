#pragma once

#include "Socket.h"
#include "RosMsgs.h"
#include "Publisher.h"
#include "Subscriber.h"
#include "msg_id.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

#include <string>
#include <vector>
#include <functional>
#include <memory>

namespace ros
{  
    /**
     * @brief This class is the counterpart to the Ros Bridge Server in the Roboterformation ROS repository.
     * It communicates the ROS topics with the server over a custom application layer protocol and a TCP socket.
     * 
     * 1. The communication gets initialized with a init message from the robot.
     * 2. The robot tells the server which topics it wants to advertise and subscribe.
     * 3. Messages can be puslished over the Publisher objects. Received messages trigger a call to the Subscriber callback function.
     * 4. The robot sends a Keep Alive message to the server every 500ms.
     * 5. If the robot does not receive a Keep Alive message from the server for a specified time period, the communication is restarted.
     */
    class NodeHandle 
    {
        public:
            /**
             * @brief Initialize the NodeHandle instance
             * 
             * @note It is safe to call this function multiple times. It will only create one NodeHandle instance.
             * 
             * @param [in] name of the robot/namespace
             * @param [in] reference to a socket object
             * @return Reference to the NodeHandle instance
             */ 
            static NodeHandle& init(std::string const& ros_namespace, Socket& sock);

            /** 
             * @brief Register functions, which get called when the robot loses connection.
             * 
             * @param [in] connection_lost_callback 
             */
            void registerConnectionLostCallback(std::function<void()> connection_lost_callback);

            /** 
             * @brief This function creates a new Publisher of the provided RosMsg Type, 
             * a pointer to the Publisher is stored in the NodeHandler and 
             * an advertise message is sent to the ROS Bridge Server
             * 
             * @param [in] topic topic name
             * @return reference to the Publisher object
             */
            template <typename T> Publisher<T>& advertise(std::string const& topic);

            /**
             * @brief This function creates a Subscriber of the provided RosMsg type,
             * a pointer to the Subscriber is stored in the NodeHandler and 
             * a subscribe message is sent to the ROS Bridge Server
             * 
             * @param [in] topic topic name
             * @param [in] callback_function gets called when a message on the topic is received
             */
            template <typename T> void subscribe(std::string const& topic, std::function<void(std::shared_ptr<T> ros_msg)> callback_function);

        private:
        	NodeHandle(std::string const& ros_namespace, Socket& sock);
            NodeHandle(const NodeHandle&) = delete;
            ~NodeHandle();

            /**
             * @brief Send the intialize message to the ROS Bridge Server.
             */
            int _send_init();

            /**
             * @brief Send a Keep Alive message to the ROS Bridge Server.
             */
            void _send_keep_alive();

            /** @brief Restart the protocol. 
             * 1. Send initialize message 
             * 2. Send advertise and subscribe messages
             */
            void _restart_protocol();

            /**
             * @brief Poll the TCP socket for new data.
             * Interpret new data according to the received msg_id (first byte in message).
             */
            int _interpret_receive();

            /** 
             * @brief FreeRTOS task for handling the communication.
             * 
             * @param [in] arg pointer to NodeHandle object
             */
            static void _communication_handler(void* arg);

            /** 
             * @brief Returns a pointer to the subscriber object of the provided topic
             * 
             * @param [in] topic 
             * @return Pointer to Subscriber object, nullptr if topic does not exist
             */
            Subscriber* _getSubscriber(std::string const& topic);

            static NodeHandle* _node_handle_obj;

            Socket& _sock;
            std::string _ros_namespace;
            std::vector<Subscriber*> _subscriber;
            std::vector<PublisherInterface*> _publisher;

            std::vector<std::function<void()>> _connection_lost_callback;

            TaskHandle_t _communication_handler_thread;

            uint64_t _server_time_difference_us;
            uint64_t _keep_alive_time_us;
            uint64_t _last_send_keep_alive_us;
    };

    template <typename T> Publisher<T>& NodeHandle::advertise(std::string const& topic)
    {
        Publisher<T>* publisher = new Publisher<T>(topic, _sock);

        publisher->advertise();

        _publisher.push_back(publisher);

        return *publisher;
    }

    template <typename T> void NodeHandle::subscribe(std::string const& topic, std::function<void(std::shared_ptr<T> ros_msg)>callback_function)
    {   
        Subscriber* subscriber = new SubscriberImpl<T>(topic, _sock, callback_function);

        subscriber->subscribe();

        _subscriber.push_back(subscriber);
    }

}

