#pragma once

#include "NodeHandle.h"
#include "Publisher.h"
#include "OutputVelocity.h"
#include "RosMsgs.h"
#include "RosMsgsLw.h"

/**
 * @brief This class publishes the robot velocity vector to ROS.
 * This can be useful for simulating the robot with e.g Turtlesim
 */
class OutputVelocitySim : public OutputVelocity
{
    public:
        /**
         * @brief Initialize the OutputVelocitySim instance
         * 
         * @note It is safe to call this function multiple times. It will only create one instance.
         * 
         * @param [in] reference to the RosBridgeClient Publisher
         * @return Reference to the OutputVelocity instance
         */  
        static OutputVelocity& init(ros::Publisher<ros_msgs::Twist2D>& publisher);
        
        /**
         * @brief This function publishes the velocity vector to ROS. 
         * 
         * @param [in] velocity velocity vector
         */ 
        void setVelocity(ros_msgs_lw::Twist2D const& velocity) override;

    private:
        OutputVelocitySim(ros::Publisher<ros_msgs::Twist2D>& publisher);
        OutputVelocitySim(OutputVelocitySim const&) = delete;
        ~OutputVelocitySim() {}

        ros::Publisher<ros_msgs::Twist2D>& _publisher;
};