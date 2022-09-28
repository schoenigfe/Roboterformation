#pragma once
#include "RosMsgsLw.h"
#include "SensorValue.h"
#include "Marvelmind.h"
//#include "freertos/FreeRTOS.h"
//#include "freertos/queue.h"
//#include "freertos/semphr.h"

class Kalman
{
    public:
		SensorValue<ros_msgs_lw::Pose2D> kalmanPose;
	private:
		Kalman();
		Kalman(Kalman const&) = delete;
		~Kalman();		
		//xTimerHandle _kalman_filter_loop_handle;
	    //void _kalman_task(void* pvParameters);
		//static void _kalman_timer(TimerHandle_t timer);
	    //void _kalman_filter_loop();
		//TaskHandle_t _kalman_task_handle;	
		void loopTask();
		void calculate_imu();
		void calculate_gps();
		static ros_msgs_lw::Pose2D pose;
		static ros_msgs_lw::PoseQual poseQual;
		static ros_msgs_lw::Imu imu;
		bool new_measurement_imu;
		bool new_measurement_gps;
		uint timestamp_imu = 0;
		uint timestamp_gps = 0;	
		uint last_timestamp_imu = 0;
		uint last_timestamp_gps = 0;
		float dt_imu;
		float dt_gps;	
	    float signal_quality;
	    float ax;
	    float ay;
	    float atheta;
	    float xy_deviation;
	    float theta_deviation;
		float rxy;
		float rt;
		float acc_x;
		float acc_y;
		float gyro_z;
		float x;
		float y;
		float theta;
	    float dt;		
	    dspm::Mat A = dspm::Mat(6,6);
	    dspm::Mat B = dspm::Mat(6,6);
	    dspm::Mat H = dspm::Mat(3,6);
	    dspm::Mat P = dspm::Mat(6,6);
	    dspm::Mat Q = dspm::Mat(6,6);
	    dspm::Mat R	= dspm::Mat(3,3);
		dspm::Mat U = dspm::Mat(6,1);
		dspm::Mat Z = dspm::Mat(3,1);
		dspm::Mat X = dspm::Mat(6,1);
		dspm::Mat X_prio = dspm::Mat(6,1);
		dspm::Mat Y = dspm::Mat(3,1);
		dspm::Mat K = dspm::Mat(6,3);
};
