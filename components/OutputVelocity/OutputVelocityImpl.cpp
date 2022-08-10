#include "OutputVelocityImpl.h"

#define WHEEL_BASE_DISTANCE 0.13
#define WHEEL_RADIUS 0.07
#define MAX_MOTOR_RPS (220. / 60.)

OutputVelocityImpl::OutputVelocityImpl(MotorController& motor_controller) : _motor_controller{motor_controller} {}

OutputVelocity& OutputVelocityImpl::init(MotorController& motor_controller)
{
    if(_output_velocity_obj == nullptr)
        _output_velocity_obj = new OutputVelocityImpl(motor_controller);

    return *_output_velocity_obj;
}

void OutputVelocityImpl::setVelocity(ros_msgs_lw::Twist2D const& velocity)
{    
    double motor_a_rps = (velocity.v - WHEEL_BASE_DISTANCE / 2 * velocity.w) / (2 * M_PI * WHEEL_RADIUS);
    double motor_b_rps = -(velocity.v + WHEEL_BASE_DISTANCE / 2 * velocity.w) / (2 * M_PI * WHEEL_RADIUS);

    //normalize motor speed
    if(abs(motor_a_rps) > MAX_MOTOR_RPS || abs(motor_b_rps > MAX_MOTOR_RPS))
    {
        float rescale_factor = 1;

        if(abs(motor_a_rps) > abs(motor_b_rps))
            rescale_factor = abs(MAX_MOTOR_RPS / motor_a_rps);
        else
            rescale_factor = abs(MAX_MOTOR_RPS / motor_b_rps);

        motor_a_rps *= rescale_factor;
        motor_b_rps *= rescale_factor;
    }

    _current_velocity.v = (motor_a_rps - motor_b_rps) * M_PI * WHEEL_RADIUS;
    _current_velocity.w = -(motor_a_rps + motor_b_rps) / WHEEL_BASE_DISTANCE * 2 * M_PI * WHEEL_RADIUS;

    _motor_controller.setVelocity(motor_a_rps, motor_b_rps);
}