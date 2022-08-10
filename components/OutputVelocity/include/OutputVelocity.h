#pragma once

#include "RosMsgsLw.h"

/**
 * @brief Interface class which enables switching between real world motor control and simulator mode 
 */ 
class OutputVelocity
{
    public:
        virtual ~OutputVelocity() {}
        
        virtual void setVelocity(ros_msgs_lw::Twist2D const& velocity) = 0;
        ros_msgs_lw::Twist2D getVelocity() const;

    protected:
        static OutputVelocity* _output_velocity_obj;
        ros_msgs_lw::Twist2D _current_velocity;

};
