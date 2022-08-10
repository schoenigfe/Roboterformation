#include "p2pController.h"

#include "esp_log.h"

//#define DATA_LOGGING
#include "DataLogger.h"

#define TAG "p2pController"

p2pController::p2pController(ros_msgs_lw::Point2D const& goal_point) : _goal_point{goal_point} {}

ros_msgs_lw::Twist2D p2pController::update(ros_msgs_lw::Pose2D const& actual_pose)
{   
    ros_msgs_lw::Twist2D output_vel;

    LOG_DATA("%.2f, %.2f\n", actual_pose.x, actual_pose.y);

    //calculate new orientation setpoint
    float setpoint_theta =  atan2(_goal_point.y - actual_pose.y, _goal_point.x - actual_pose.x); 

    //Calculate orientation error and keep it between -pi - pi
    float error_theta = setpoint_theta - actual_pose.theta;
    error_theta = atan2(sin(error_theta), cos(error_theta)); 
    //ESP_LOGI(TAG, "error: %f, actual_pose: %f, %f, %f", error_theta, actual_pose.x, actual_pose.y, actual_pose.theta);
    
    //calculate distance error
    float error_dist = sqrt(pow(_goal_point.x - actual_pose.x, 2) + pow(_goal_point.y - actual_pose.y, 2));  

    //Calculate output
    output_vel.v = _kp_v * error_dist;
    output_vel.w = _kp_w * error_theta;

    //check if destination has been reached
    if(error_dist < 0.05)
    {
        _destination_reached = true;
        output_vel = 0;
    }

    return output_vel;
}

bool p2pController::destination_reached()
{   
    return _destination_reached;
}