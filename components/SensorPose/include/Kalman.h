#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "RosMsgsLw.h"
#include "Marvelmind.h"

class Kalman : public SensorPose
{
    public:
		SensorValue<ros_msgs_lw::Pose2D> kalmanPose;
		bool calculate();
		
		Kalman();
		~Kalman();
		Kalman(Kalman const&) = delete;
		
	private:
	


