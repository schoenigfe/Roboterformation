#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
//#include "RosMsgsLw.h"


/**
 * @brief The different Sensors are abstracted with the SensorValue interface to simplify interchanging.
 */
template <typename T>
class SensorValue {
    public:
        /**
         * @brief This method reads a new Value from the internal value queue without removing it.
         * 
         * @param [out] current_value
         * @return true if reading was succesfull
         */
        bool peekAtValue(T& current_value) 
		{
			if(xQueuePeek(_peek_at_value_queue, &current_value, 0) == pdPASS)	
				return true;
			return false;
		}
        /**
         * @brief This method reads a new Value from the internal value queue and removes it
         * 
         * @param [out] current_value
         * @return true if reading was succesfull, false if no value element is in the queue
         */
        bool getValue(T& current_value)
        {
            if(xQueueReceive(_current_value_queue, &current_value, 0) == pdPASS)	
                return true;
            return false;
        }
		/**
         * @brief This method overwrites the Value in the internal value queue
         * 
         * @param [in] new_value
         */
        void overwriteValue(T& new_value)
        {
            xQueueOverwrite(_peek_at_value_queue, &new_value);
            xQueueOverwrite(_current_value_queue, &new_value);
        }        
        SensorValue()
        {
            _peek_at_value_queue = xQueueCreate(1, sizeof(T));
            _current_value_queue = xQueueCreate(1, sizeof(T));
        }
        ~SensorValue()
        {
            vQueueDelete(_peek_at_value_queue);
            vQueueDelete(_current_value_queue);	
        }        
        SensorValue(SensorValue const&) = delete;
	private: 
        QueueHandle_t _peek_at_value_queue;
		QueueHandle_t _current_value_queue;              
};