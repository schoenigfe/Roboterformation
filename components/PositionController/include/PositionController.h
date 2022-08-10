#pragma once

#include "RosMsgsLw.h"

/**
 * @brief Interface for the PositionController classes 
 */
class PositionController
{
    public:
        virtual ~PositionController() {}

        /**
         * @brief Updates the underlying PositionController object
         * 
         * @note This function must be called periodically
         * 
         * @param [in] actual_pose current pose of the robot
         * @return output velocity vector
         */
        virtual ros_msgs_lw::Twist2D update(ros_msgs_lw::Pose2D const& actual_pose) = 0;

        /**
         * @brief This function checks if the goal of the PositionController has been reached.
         * 
         * @return true if goal has been reached, false if not
         */
        virtual bool destination_reached() = 0;
    
    private:

};