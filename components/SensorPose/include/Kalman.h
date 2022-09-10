#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "RosMsgsLw.h"
#include "SensorValue.h"
#include "Marvelmind.h"

class Kalman
{
    public:
		SensorValue<ros_msgs_lw::Pose2D> kalmanPose;
		
		/**In: Pointer to pose_current_value_queue, Pointer to pose_qual_current_value_queue, Pointer to imu_current_value_queue
		 * Out: writes kalmanPose (two Queues for calculated Poses)
		**/
		bool calculate(SensorValue<ros_msgs_lw::Pose2D>*, SensorValue<ros_msgs_lw::PoseQual>*, SensorValue<ros_msgs_lw::Imu>);
		
		Kalman();
		~Kalman();
		Kalman(Kalman const&) = delete;
		
	private:
	
};

