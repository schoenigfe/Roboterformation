#include "RosMsgs.h"
#include "RosMsgsLw.h"

namespace ros_msgs
{
    //size_t const Pose2DSim::_msg_size = 12;
    size_t const Pose2D::_msg_size = 24;
    size_t const Twist2D::_msg_size = 16;
    size_t const Point2D::_msg_size = 16;
    size_t const TrajectoryStateVector::_msg_size = 36;
    size_t const Imu::_msg_size = 24;
    size_t const PoseQual::_msg_size = 1;



    Pose2D::Pose2D(ros_msgs_lw::Pose2D const& pose) : x{static_cast<double>(pose.x)}, y{static_cast<double>(pose.y)}, theta{static_cast<double>(pose.theta)} {}
    Twist2D::Twist2D(ros_msgs_lw::Twist2D const& velocity) : v{static_cast<double>(velocity.v)}, w{static_cast<double>(velocity.w)} {}
    Point2D::Point2D(ros_msgs_lw::Point2D const& point) : x{static_cast<double>(point.x)}, y{static_cast<double>(point.y)} {}
    Imu::Imu(ros_msgs_lw::Imu const& imu) : a_x{static_cast<double>(imu.a_x)}, a_y{static_cast<double>(imu.a_y)}, v_theta{static_cast<double>(imu.v_theta)} {}    
	PoseQual::PoseQual(ros_msgs_lw::PoseQual const& qual) : q{static_cast<uint8_t>(qual.q)} {}
}
