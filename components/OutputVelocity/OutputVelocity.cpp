#include "OutputVelocity.h"


#define TAG "OutputVelocity"

OutputVelocity* OutputVelocity::_output_velocity_obj = nullptr;

ros_msgs_lw::Twist2D OutputVelocity::getVelocity() const
{
    return _current_velocity;
}