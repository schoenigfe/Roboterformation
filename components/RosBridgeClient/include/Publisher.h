#pragma once

#include "Socket.h"
#include "msg_id.h"
#include "RosMsgs.h"

#include <vector>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace ros
{  
    /**
     * @brief Interface for storing Publishers of different message types in std::vector
     */
    class PublisherInterface
    {
        public:
            virtual ~PublisherInterface() {}
            virtual void advertise() = 0;
            virtual void block_publishing() = 0;
            virtual void unblock_publishing() = 0;
    };

    /**
     * @brief The Publisher enables publishing messages to ROS.
     */
    template <typename T> class Publisher : public PublisherInterface
    {
        public:
            /**
             * @brief The publish() method serializes the RosMsg, adds the topic name as a header
             * and sends it as a Publish message to the ROS Robot Server.
             * 
             * @param [in] msg Message of the Publisher type
             */
            void publish(T const& msg);

        private: 
            friend class NodeHandle;

            explicit Publisher(std::string const& topic, Socket& sock);
            explicit Publisher(Publisher const&) = delete;
            ~Publisher();

            /**
             * @brief The advertise() method sends/advertises the topic name with the message type to 
             * the ROS Robot Server as a Advertise message.
             */
            void advertise() override;

            /**
             * @brief This method is used by the NodeHandle object to block publishing during protocol restart.
             */
            void block_publishing() override;

            /**
             * @brief This method is used to unblock publishing after the protocol restart in NodeHandle.
             */
            void unblock_publishing() override;

            const std::string _topic;
            Socket& _sock;
            SemaphoreHandle_t _block_publishing_semphr;
    };

    template <typename T> Publisher<T>::Publisher(std::string const& topic, Socket& sock) : _topic{topic}, _sock{sock}
    {
        _block_publishing_semphr = xSemaphoreCreateBinary();
        xSemaphoreGive(_block_publishing_semphr);
    }

    template <typename T> Publisher<T>::~Publisher()
    {
        vSemaphoreDelete(_block_publishing_semphr);
    }
    
    template <typename T> void Publisher<T>::publish(T const& msg)
    {   
        if(xSemaphoreTake(_block_publishing_semphr, 0) == pdPASS && msg.getSize() != 0)
        { 
            uint8_t* pkt_buffer = new uint8_t[_topic.size() + msg.getSize() +  2];

            int pkt_len = 0;

            pkt_buffer[0] = PUBLISH_ID;
            pkt_len++;

            memcpy(pkt_buffer + pkt_len, _topic.c_str(), _topic.size());
            pkt_len += _topic.size();

            pkt_buffer[pkt_len] = '\0';
            pkt_len++;

            msg.serialize(pkt_buffer + pkt_len);
            pkt_len += msg.getSize();

            _sock.socket_send(pkt_buffer, pkt_len);

            delete[] pkt_buffer;

            xSemaphoreGive(_block_publishing_semphr);
        }
    }

    template <typename T> void Publisher<T>::advertise()
    {
        uint8_t* pkt_buffer = new uint8_t[1 + _topic.size() + 1 + T::getMsgType().size() + 1];

        int pkt_len = 0;

        pkt_buffer[0] = ADVERTISE_ID;
        pkt_len++;

        memcpy(pkt_buffer + pkt_len, _topic.c_str(), _topic.size());
        pkt_len += _topic.size();

        pkt_buffer[pkt_len] = '\0';
        pkt_len++;

        memcpy(pkt_buffer + pkt_len, T::getMsgType().c_str(), T::getMsgType().size());
        pkt_len += T::getMsgType().size();

        pkt_buffer[pkt_len] = '\0';
        pkt_len++;

        _sock.socket_send(pkt_buffer, pkt_len);

        delete[] pkt_buffer;
    }

    template <typename T> void Publisher<T>::block_publishing()
    {
        xSemaphoreTake(_block_publishing_semphr, portMAX_DELAY);
    }

    template <typename T> void Publisher<T>::unblock_publishing()
    {
        xSemaphoreGive(_block_publishing_semphr);
    }
}