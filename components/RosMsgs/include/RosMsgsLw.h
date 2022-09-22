#pragma once

#include "esp_err.h"
#include "math.h"

#include "mat.h"

namespace ros_msgs
{
    struct Pose2D;
    //struct Pose2DSim;
    struct Twist2D;
    struct Point2D;
    struct Imu;
    struct PoseQual;
}

/**
 * @brief Since double precision arithmetic is only emulated on the ESP32, 
 * the ros_msgs_lw classes can be used to maximize execution speed.
 * These classes should provide a type casting constructor for ros_msgs.
 */
namespace ros_msgs_lw
{
    struct Pose2D
    {   
        public:
        
            explicit Pose2D(float x, float y, float theta);
            explicit Pose2D(ros_msgs::Pose2D const& pose);
            //explicit Pose2D(ros_msgs::Pose2DSim const& pose);
            explicit Pose2D(dspm::Mat const& pose);
        
            Pose2D() : x{0}, y{0}, theta{0} {}

            void operator=(Pose2D const& pose);

            void operator=(dspm::Mat const& pose);

            float x;
            float y;
            float theta;
               };
	   
	Pose2D operator*(float scalar, Pose2D const& pose);

	Pose2D operator+(Pose2D const& pose_1, Pose2D const& pose_2);

	Pose2D operator-(Pose2D const& pose_1, Pose2D const& pose_2);

	Pose2D operator+(Pose2D const& pose, dspm::Mat const& mat);

	dspm::Mat operator*(dspm::Mat const& mat, Pose2D const& pose);


    struct Twist2D
    {
        public:
            explicit Twist2D(float x, float w) : v{x}, w{w} {}
            explicit Twist2D(ros_msgs::Twist2D const& velocity);
            Twist2D() : v{0}, w{0} {}      

            void operator=(Twist2D const& velocity);

            void operator=(ros_msgs::Twist2D const& velocity);

            void operator=(float i);

            float v;
            float w;
    };

    struct Point2D
    {   
        public:
            explicit Point2D(float x, float y) : x{x}, y{y} {}
            explicit Point2D(ros_msgs::Point2D const& point);
            Point2D() : x{0}, y{0} {}

            float x;
            float y;

    };
    
    struct Imu
    {   
        public:
        
            explicit Imu(uint32_t timestamp, float quaternion_orientation_x, float quaternion_orientation_y, float quaternion_orientation_z, float quaternion_orientation_w, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z, float linear_acceleration_x, float linear_acceleration_y, float linear_acceleration_z);
            explicit Imu(ros_msgs::Imu const& pose);
            explicit Imu(dspm::Mat const& imu);

            Imu() : timestamp{0}, quaternion_orientation_x{0}, quaternion_orientation_y{0}, quaternion_orientation_z{0}, quaternion_orientation_w{0}, angular_velocity_x{0}, angular_velocity_y{0}, angular_velocity_z{0}, linear_acceleration_x{0}, linear_acceleration_y{0}, linear_acceleration_z{0} {}

            void operator=(Imu const& imu);

            //void operator=(dspm::Mat const& imu);
        
            uint32_t timestamp;
            float quaternion_orientation_x;
            float quaternion_orientation_y;
            float quaternion_orientation_z;
            float quaternion_orientation_w;
            float angular_velocity_x;
            float angular_velocity_y;
            float angular_velocity_z;
            float linear_acceleration_x;
            float linear_acceleration_y;
            float linear_acceleration_z;
            
    };
    /*
    IMU operator*(float scalar, IMU const& imu);
    
    IMU operator+(IMU const& imu_1, IMU const& imu_2);
    
    IMU operator-(IMU const& imu_1, IMU const& imu_2);
    
    IMU operator+(IMU const& imu, dspm::Mat const& mat);
    
    dspm::Mat operator*(dspm::Mat const& mat, IMU const& imu);
    */
    
    struct PoseQual
    {
		public: 
			explicit PoseQual(uint8_t q);
			explicit PoseQual(ros_msgs::PoseQual const& qual);			
			PoseQual() : q{0} {}			
            size_t getSize() const 
            { 
                return _msg_size; 
            }            
            uint8_t q;
        private:
            static size_t const _msg_size;
	};
}
