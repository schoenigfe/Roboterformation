#include "RosMsgs.h"
#include "RosMsgsLw.h"

namespace ros_msgs
{
    //size_t const Pose2DSim::_msg_size = 12;
    size_t const Pose2D::_msg_size = 24;
    size_t const Twist2D::_msg_size = 16;
    size_t const Point2D::_msg_size = 16;
    size_t const TrajectoryStateVector::_msg_size = 36;
    size_t const Imu::_msg_size = 84;
    size_t const PoseQual::_msg_size = 1;



    Pose2D::Pose2D(ros_msgs_lw::Pose2D const& pose) : x{static_cast<double>(pose.x)}, y{static_cast<double>(pose.y)}, theta{static_cast<double>(pose.theta)} {}
    Twist2D::Twist2D(ros_msgs_lw::Twist2D const& velocity) : v{static_cast<double>(velocity.v)}, w{static_cast<double>(velocity.w)} {}
    Point2D::Point2D(ros_msgs_lw::Point2D const& point) : x{static_cast<double>(point.x)}, y{static_cast<double>(point.y)} {}
    Imu::Imu(ros_msgs_lw::Imu const& imu) : timestamp{static_cast<uint32_t>(imu.timestamp)}, quaternion_orientation_x{static_cast<double>(imu.quaternion_orientation_x)}, quaternion_orientation_y{static_cast<double>(imu.quaternion_orientation_y)}, quaternion_orientation_z{static_cast<double>(imu.quaternion_orientation_z)}, quaternion_orientation_w{static_cast<double>(imu.quaternion_orientation_w)}, angular_velocity_x{static_cast<double>(imu.angular_velocity_x)}, angular_velocity_y{static_cast<double>(imu.angular_velocity_y)}, angular_velocity_z{static_cast<double>(imu.angular_velocity_z)}, linear_acceleration_x{static_cast<double>(imu.linear_acceleration_x)}, linear_acceleration_y{static_cast<double>(imu.linear_acceleration_y)}, linear_acceleration_z{static_cast<double>(imu.linear_acceleration_z)} {}
	PoseQual::PoseQual(ros_msgs_lw::PoseQual const& qual) : q{static_cast<uint8_t>(qual.q)} {}
}
