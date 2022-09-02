#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "RosMsgs.h"
#include "RosMsgsLw.h"
#include "SensorValue.h"

class ImuSensor{
    public:
		SensorValue* accel;
		SensorValue* gyro;
		
    protected:
        ImuSensor() 
        {
			accel = new SensorValue<ros_msgs::accel>;
			gyro = new SensorValue;
		}
        ~ImuSensor() 
        {
			delete accel;
			delete gyro;
		}
        ImuSensor(ImuSensor const&) = delete;
        
};

