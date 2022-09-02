#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "RosMsgs.h"
#include "RosMsgsLw.h"
#include "SensorValue.h"

class UsSensor{
    public:
		SensorValue* accel;
		SensorValue* gyro;
		
    protected:
        UsSensor() 
        {
			accel = new SensorValue;
			gyro = new SensorValue;
		}
        ~UsSensor() 
        {
			delete accel;
			delete gyro;
		}
        UsSensor(ImuSensor const&) = delete;
        
};

