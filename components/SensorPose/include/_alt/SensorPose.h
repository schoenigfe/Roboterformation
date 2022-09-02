#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "RosMsgs.h"
#include "RosMsgsLw.h"


/**
 * @brief The different Pose Sensor are abstracted with the SensorPose interface to simplify interchanging.
 */
class SensorPose {
    public:
        /**
         * @brief This method reads a new Pose from the internal pose queue without removing it.
         * 
         * @param [out] current_pose
         * @return true if reading was succesfull
         */
        virtual bool peekAtPose(ros_msgs_lw::Pose2D& current_pose) const = 0;

        /**
         * @brief This method reads a new Pose from the internal pose queue and removes it
         * 
         * @param [out] current_pose
         * @return true if reading was succesfull, false if no pose element is in the queue
         */
        virtual bool getPose(ros_msgs_lw::Pose2D& current_pose) const = 0;

         /**
         * @brief This method is only implemented in the KalmanFilter to reinitialize the internal state vector 
         * and its covariance matrix.
         */
        virtual void reInit() = 0;

    protected:
        SensorPose() {}
        ~SensorPose() {}
        SensorPose(SensorPose const&) = delete;
        
};

