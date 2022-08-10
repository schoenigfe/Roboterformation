#include "dynInOutLinController.h"

#include "esp_log.h"

#define DATA_LOGGING
#include "DataLogger.h"

#define TAG "dynInOutLinController"

dynInOutLinController::dynInOutLinController(std::shared_ptr<ros_msgs::Trajectory> trajectory) : 
    _prev_time_us{(uint64_t)esp_timer_get_time()},  _trajectory{trajectory} 
{
    _prev_velocity = sqrt(pow((*_trajectory)[0].dx, 2) + pow((*_trajectory)[0].dy, 2));
    _state_vector_time_difference_us = (*_trajectory)[0].timestamp /1000 - esp_timer_get_time();
}

ros_msgs_lw::Twist2D dynInOutLinController::update(ros_msgs_lw::Pose2D const& actual_pose)
{   
    ros_msgs_lw::Twist2D output_vel;

    uint64_t current_time_us = esp_timer_get_time();

    while(_trajectory_cntr + 1 < _trajectory->getTrajectorySize() &&
        (*_trajectory)[_trajectory_cntr + 1].timestamp / 1000 - _state_vector_time_difference_us < current_time_us)
    {
        _trajectory_cntr++;
    }

    if(_trajectory_cntr < _trajectory->getTrajectorySize())
    {
        /*calculate time since last excecution*/
        uint64_t delta_time_us = current_time_us - _prev_time_us;
        _prev_time_us = current_time_us;

        ros_msgs::TrajectoryStateVector const&  setpoint_vector = (*_trajectory)[_trajectory_cntr];
        _trajectory_cntr++;

        LOG_DATA("%.2f, %.2f, %.2f, %.2f\n", setpoint_vector.x, setpoint_vector.y, actual_pose.x, actual_pose.y);


        float actual_dx = _prev_velocity * cos(actual_pose.theta);
        float actual_dy = _prev_velocity * sin(actual_pose.theta);

        
        float u1 = setpoint_vector.ddx + _kp_1 * (setpoint_vector.x - actual_pose.x) + _kd_1 * (setpoint_vector.dx - actual_dx);
        float u2 = setpoint_vector.ddy + _kp_2 * (setpoint_vector.y - actual_pose.y) + _kd_2 * (setpoint_vector.dy - actual_dy);

        float output_acceleration = u1 * cos(actual_pose.theta) + u2 * sin(actual_pose.theta);
        
        output_vel.w = - u1 * sin(actual_pose.theta) / _prev_velocity + u2 * cos(actual_pose.theta) / _prev_velocity;

        _prev_velocity = delta_time_us / 1000000.0 * (_prev_acceleration + output_acceleration) / 2. + _prev_velocity;
        _prev_acceleration = output_acceleration;

        if(_prev_velocity == 0)
            _prev_velocity = 0.01;
        
        output_vel.v = _prev_velocity;

    }
    else
    {
        output_vel = 0;
        _destination_reached = true;
    }

    return output_vel;
}

bool dynInOutLinController::destination_reached()
{
    return _destination_reached;
}