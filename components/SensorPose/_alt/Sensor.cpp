#include "SensorPoseSim.h"

SensorPoseSim* SensorPoseSim::_sensor_pose_sim = nullptr;

SensorPoseSim::SensorPoseSim(ros::NodeHandle& node_handle) : _measurement_noise_cov(3, 3)  //constructor of dspm::Mat sets noise covariance to 0
{
    _current_pose_queue = xQueueCreate(1, sizeof(ros_msgs_lw::Pose2D));
    _peek_at_pose_queue = xQueueCreate(1, sizeof(ros_msgs_lw::Pose2D));


    node_handle.subscribe<ros_msgs::Pose2DSim>("pose", std::bind(&SensorPoseSim::_setPose, this, std::placeholders::_1));
}

SensorPoseSim::~SensorPoseSim()
{
    vQueueDelete(_current_pose_queue);
    vQueueDelete(_peek_at_pose_queue);
}

SensorPoseSim& SensorPoseSim::init(ros::NodeHandle& node_handle)
{
    if(_sensor_pose_sim == nullptr)
        _sensor_pose_sim = new SensorPoseSim(node_handle);

    return *_sensor_pose_sim;
}

bool SensorPoseSim::getPose(ros_msgs_lw::Pose2D& current_pose) const
{
    if(xQueueReceive(_current_pose_queue, &current_pose, 0) == pdPASS)
        return true;

    return false;
}

bool SensorPoseSim::peekAtPose(ros_msgs_lw::Pose2D& current_pose) const
{
    if(xQueuePeek(_peek_at_pose_queue, &current_pose, 0) == pdPASS)
        return true;

    return false;
}

bool SensorPoseSim::calculateKalman(ros_msgs_lw::Pose2D const& a_priori_estimate, dspm::Mat const& a_priori_cov, ros_msgs_lw::Pose2D& a_posterior_estimate, dspm::Mat& a_posterior_cov) const
{
    ros_msgs_lw::Pose2D current_pose;

    if(xQueueReceive(_current_pose_queue, &current_pose, 0) == pdPASS)
    {
        float kalman_data[9];
        dspm::Mat kalman_gain(kalman_data, 3, 3);

        kalman_gain = a_priori_cov * (a_priori_cov + _measurement_noise_cov).inverse();
        
        a_posterior_estimate = a_priori_estimate + kalman_gain * (current_pose - a_priori_estimate);

        float identity_data[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        dspm::Mat identity(identity_data, 3, 3);

        a_posterior_cov = (identity - kalman_gain) * a_priori_cov;

        return true;
    }

    return false;
}

bool SensorPoseSim::getAbsolutePose(ros_msgs_lw::Pose2D& initial_pose) const
{
    if(xQueueReceive(_current_pose_queue, &initial_pose, 0) == pdPASS)
        return true;

    return false;
}

void SensorPoseSim::getMeasurementNoiseCov(dspm::Mat& measurement_cov) const
{
    measurement_cov = _measurement_noise_cov;
}

void SensorPoseSim::_setPose(std::shared_ptr<ros_msgs::Pose2DSim> pose)
{   
    ros_msgs_lw::Pose2D current_pose(*pose);

    xQueueOverwrite(_current_pose_queue, &current_pose);
}

