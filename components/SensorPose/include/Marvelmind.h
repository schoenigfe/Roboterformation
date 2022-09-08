#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "driver/uart.h"

#include "RosMsgsLw.h"
#include "SensorValue.h"

/**
 * The Marvelmind class drives the UART communication with the Marvelmind RX Beacon.
 * It can be used as a standalone SensorPose or as a source for the KalmanFilter.
 */
class Marvelmind
{
    public:
		static SensorValue<ros_msgs_lw::Pose2D>* pose;
		static SensorValue<ros_msgs_lw::PoseQual>* poseQual;
		static SensorValue<ros_msgs_lw::Imu>* imu;
		
        Marvelmind();
        ~Marvelmind(); 
        Marvelmind(Marvelmind const&) = delete;
	private: 
	    static void _uart_read_data_task(void* pvParameters);  
        static const uart_port_t _uart_port; 
        static const uart_config_t _uart_conf;
        TaskHandle_t _uart_read_data_task_handle;   
};