#include "OutputVelocitySim.h"

#define WHEEL_BASE_DISTANCE 0.13
#define WHEEL_RADIUS 0.07
#define MAX_MOTOR_RPS (220. / 60.)
#define MAX_LINEAR_VELOCITY (MAX_MOTOR_RPS * 2. * M_PI * WHEEL_RADIUS) 
#define MAX_ANGULAR_VELOCITY (2. * MAX_LINEAR_VELOCITY / WHEEL_BASE_DISTANCE)

OutputVelocitySim::OutputVelocitySim(ros::Publisher<ros_msgs::Twist2D>& publisher) : _publisher{publisher} {}

OutputVelocity& OutputVelocitySim::init(ros::Publisher<ros_msgs::Twist2D>& publisher)
{
    if(_output_velocity_obj == nullptr)
        _output_velocity_obj = new OutputVelocitySim(publisher);

    return *_output_velocity_obj;
}

void OutputVelocitySim::setVelocity(ros_msgs_lw::Twist2D const& velocity)
{   
    _current_velocity = velocity;

    if(velocity.v > MAX_LINEAR_VELOCITY)
        _current_velocity.v = MAX_LINEAR_VELOCITY;
    else if(velocity.v < -MAX_LINEAR_VELOCITY)
        _current_velocity = -MAX_LINEAR_VELOCITY;
        
    if(velocity.w > MAX_ANGULAR_VELOCITY)
        _current_velocity.w = MAX_ANGULAR_VELOCITY;
    else if(velocity.w < -MAX_ANGULAR_VELOCITY)
        _current_velocity.w = -MAX_ANGULAR_VELOCITY;

    ros_msgs::Twist2D vel_msg(_current_velocity);

    _publisher.publish(vel_msg);
}