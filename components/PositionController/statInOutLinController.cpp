#include "statInOutLinController.h"

//#define DATA_LOGGING
#include "DataLogger.h"

statInOutLinController::statInOutLinController(std::shared_ptr<ros_msgs::Trajectory> trajectory) : _trajectory{trajectory} {}

ros_msgs_lw::Twist2D statInOutLinController::update(ros_msgs_lw::Pose2D const& actual_pose) 
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
        ros_msgs::TrajectoryStateVector const&  setpoint_vector = (*_trajectory)[_trajectory_cntr];
        _trajectory_cntr++;

        LOG_DATA("%.2f, %.2f, %.2f, %.2f\n", setpoint_vector.x, setpoint_vector.y, actual_pose.x, actual_pose.y);

        //calculate current position of point b
        ros_msgs::Pose2D actual_b_pose;
        actual_b_pose.x = actual_pose.x + _b_offset * cos(actual_pose.theta);
        actual_b_pose.y = actual_pose.y + _b_offset * sin(actual_pose.theta);

        float u1 = setpoint_vector.dx + _kp_1 * (setpoint_vector.x - actual_b_pose.x);
        float u2 = setpoint_vector.dy + _kp_2 * (setpoint_vector.y - actual_b_pose.y);

        output_vel.v = u1 * cos(actual_pose.theta) + u2 * sin(actual_pose.theta);
        output_vel.w = - u1 * sin(actual_pose.theta) / _b_offset + u2 * cos(actual_pose.theta) / _b_offset;
    }
    else
    {
        output_vel = 0;
        _destination_reached = true;
    }

    return output_vel;
}

bool statInOutLinController::destination_reached()
{
    return _destination_reached;
}