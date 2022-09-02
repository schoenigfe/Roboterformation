#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "RosMsgs.h"
#include "RosMsgsLw.h"


/**
 * @brief The different Sensors are abstracted with the Sensor interface to simplify interchanging.
 */
class Sensor {
    public:
        /**
         * @brief This method reads a new Value from the internal queue without removing it.
         * 
         * @param [out] current_value
         * @return true if reading was succesfull
         */
        virtual bool peekAtValue(current_value) const = 0;

        /**
         * @brief This method reads a new Pose from the internal pose queue and removes it
         * 
         * @param [out] current_pose
         * @return true if reading was succesfull, false if no pose element is in the queue
         */
        virtual bool getValue(current_value) const = 0;

    protected:
        Sensor() {}
        ~Sensor() {}
        Sensor(Sensor const&) = delete;
        
};

