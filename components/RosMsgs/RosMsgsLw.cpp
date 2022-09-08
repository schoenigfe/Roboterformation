#include "RosMsgsLw.h"
#include "RosMsgs.h"

namespace ros_msgs_lw
{
    Pose2D::Pose2D(float x, float y, float theta) : x{x}, y{y}, theta{theta} 
    {
        //Keep theta between -pi and pi
        theta = atan2(sin(theta), cos(theta));
    }

    Pose2D::Pose2D(ros_msgs::Pose2D const& pose) : x{static_cast<float>(pose.x)}, y{static_cast<float>(pose.y)}, theta{static_cast<float>(pose.theta)} {}

    Pose2D::Pose2D(ros_msgs::Pose2DSim const& pose) : x{pose.x}, y{pose.y}, theta{pose.theta} {}

    Pose2D::Pose2D(dspm::Mat const& pose) 
    {
        if(pose.rows != 3 || pose.cols != 1)
            ESP_ERROR_CHECK(ESP_FAIL);

        x = pose(0, 0);
        y = pose(1, 0);
        theta = atan2(sin(pose(2, 0)), cos(pose(2, 0)));
    }

    void Pose2D::operator=(Pose2D const& pose)
    {
        x = pose.x;
        y = pose.y;
        theta = pose.theta;

        //Keep theta between -pi and pi
        theta = atan2(sin(theta), cos(theta));
    }

    void Pose2D::operator=(dspm::Mat const& pose)
    {   
        if(pose.rows != 3 || pose.cols != 1)
            ESP_ERROR_CHECK(ESP_FAIL);

        x = pose(0, 0);
        y = pose(1, 0);
        theta = atan2(sin(pose(2, 0)), cos(pose(2, 0)));
    }

    Pose2D operator+(Pose2D const& pose_1, Pose2D const& pose_2)
    {
        Pose2D result;
        result.x = pose_1.x + pose_2.x;
        result.y = pose_1.y + pose_2.y;
        result.theta = pose_1.theta + pose_2.theta;
        result.theta = atan2(sin(result.theta), cos(result.theta));

        return result;
    }

    Pose2D operator-(Pose2D const& pose_1, Pose2D const& pose_2)
    {
        Pose2D result;

        result.x = pose_1.x - pose_2.x;
        result.y = pose_1.y - pose_2.y;
        result.theta = pose_1.theta - pose_2.theta;
        result.theta = atan2(sin(result.theta), cos(result.theta));

        return result;
    }

    Pose2D operator+(Pose2D const& pose, dspm::Mat const& mat)
    {
        if(mat.rows != 3 || mat.cols != 1)
            ESP_ERROR_CHECK(ESP_FAIL);
        
        Pose2D result;
        result.x = pose.x + mat(0, 0);
        result.y = pose.y + mat(1, 0);
        result.theta = pose.theta + mat(2, 0);
        result.theta = atan2(sin(result.theta), cos(result.theta));

        return result;
    }

    Pose2D operator*(float scalar, Pose2D const& pose)
    {
        Pose2D result;

        result.x = scalar * pose.x;
        result.y = scalar * pose.y;
        result.theta = scalar * pose.theta;

        result.theta = atan2(sin(result.theta), cos(result.theta));

        return result;
    }

    dspm::Mat operator*(dspm::Mat const& mat, Pose2D const& pose)
    {
        if(mat.cols != 3)
            ESP_ERROR_CHECK(ESP_FAIL);

        dspm::Mat mat_pose(3, 1);
        mat_pose(0, 0) = pose.x;
        mat_pose(1, 0) = pose.y;
        mat_pose(2, 0) = pose.theta;

        dspm::Mat result = mat * mat_pose;

        return result;
    }


    Twist2D::Twist2D(ros_msgs::Twist2D const& velocity): v{static_cast<float>(velocity.v)}, w{static_cast<float>(velocity.w)} {}
    
    void Twist2D::operator=(Twist2D const& velocity)
    {  
        v = velocity.v;
        w = velocity.w;
    }

    void Twist2D::operator=(ros_msgs::Twist2D const& velocity)
    {
        v = static_cast<float>(velocity.v);
        w = static_cast<float>(velocity.w);
    }

    void Twist2D::operator=(float i)
    {  
        v = i;
        w = i;
    }


    Point2D::Point2D(ros_msgs::Point2D const& point) : x{static_cast<float>(point.x)}, y{static_cast<float>(point.y)} {}
    
    Imu::Imu(ros_msgs::Imu const& imu) : timestamp{static_cast<uint32_t>(imu.timestamp)}, quaternion_orientation_x{static_cast<float>(imu.quaternion_orientation_x)}, quaternion_orientation_y{static_cast<float>(imu.quaternion_orientation_y)}, quaternion_orientation_z{static_cast<float>(imu.quaternion_orientation_z)}, quaternion_orientation_w{static_cast<float>(imu.quaternion_orientation_w)}, angular_velocity_x{static_cast<float>(imu.angular_velocity_x)}, angular_velocity_y{static_cast<float>(imu.angular_velocity_y)}, angular_velocity_z{static_cast<float>(imu.angular_velocity_z)}, linear_acceleration_x{static_cast<float>(imu.linear_acceleration_x)}, linear_acceleration_y{static_cast<float>(imu.linear_acceleration_y)}, linear_acceleration_z{static_cast<float>(imu.linear_acceleration_z)} {}
  
	void Imu::operator=(Imu const& imu)
            {
                timestamp = imu.timestamp;
                quaternion_orientation_x = imu.quaternion_orientation_x;
                quaternion_orientation_y = imu.quaternion_orientation_y;
                quaternion_orientation_z = imu.quaternion_orientation_z;
                quaternion_orientation_w = imu.quaternion_orientation_w;
                angular_velocity_x = imu.angular_velocity_x;
                angular_velocity_y = imu.angular_velocity_y;
                angular_velocity_z = imu.angular_velocity_z;
                linear_acceleration_x = imu.linear_acceleration_x;
                linear_acceleration_y = imu.linear_acceleration_y;
                linear_acceleration_z = imu.linear_acceleration_z;
            }

	PoseQual::PoseQual(uint q) : q{q} {}
	PoseQual::PoseQual(ros_msgs::PoseQual const& qual) : q{static_cast<uint>(qual.q)} {}
}
