#pragma once

#include <string>
#include <vector>
#include <cstring>
#include <array>

#include "math.h"

#include "msg_id.h"

#include "esp_log.h"

namespace ros_msgs_lw
{
    struct Pose2D;
    struct Twist2D;
    struct Point2D;
    struct Imu;
    struct PoseQual;
}

/**
 * @brief For every ROS Message Type which is communicated over the ROS Robot Bridge there has to be a ros_msgs::Type.
 * These classes must provide the methods getSize(), allocateMemory(), getMsgType(), serialize() and deserialize().
 * There are two types of RosMsgs Standard and Array type (e.g. String, Trajectory).
 * For the Standard type message the allocateMemory() method can be left empty and
 * the getSize() method will always return the accumulated size of its members.
 * The Array type message uses the allocateMemory() method to allocate the required memory for the deserialization and
 * the getSize() method returns zero if the array is empty or array length in bytes + 4 bytes (for the serialization of array length).
 * 
 * @note Publishing empty Array messages is not supported and will break the protocol!!!
 */
namespace ros_msgs
{
    struct String
    {   
        public:
            explicit String() {}
            explicit String(std::string data) : data{data} {}
            size_t getSize() const
            { 
                if(data.empty() == true)
                    return 0; 
                
                return sizeof(int32_t) + data.size();
            }
            void allocateMemory(int32_t msg_len) 
            {
                data.reserve(msg_len);

                _reserved_bytes = msg_len;
            }
            static std::string getMsgType()
            {
                return "std_msgs/String";
            }           
            void serialize(uint8_t* buffer) const
            { 
                if(data.empty() == false)
                {
                    ((int32_t*)buffer)[0] = data.size();
                    memcpy(buffer + sizeof(int32_t), data.c_str(), data.size());
                    //buffer[sizeof(int32_t) + data.size()] = '\0';
                }
            }           
            void deserialize(uint8_t* buffer)
            {
                data.assign((char*)buffer, _reserved_bytes);
            }
            bool operator==(std::string string_2) const
            {
                return data == string_2;
            }
            explicit operator std::string() const
            {
                return data;
            }
            std::string data;
        private:
            int _reserved_bytes = 0;
    };

    /*struct Pose2DSim
    {   
        public:
            explicit Pose2DSim(float x, float y, float theta) : x{x}, y{y}, theta{theta} 
            {
                //Keep theta between -pi and pi
                theta = atan2(sin(theta), cos(theta));
            }
            Pose2DSim() : x{0}, y{0}, theta{0} {}

            size_t getSize() const
            { 
                return _msg_size; 
            }
            
            void allocateMemory(int32_t msg_len) {}

            static std::string getMsgType()
            {
                return "turtlesim/Pose";
            }

            void serialize(uint8_t* buffer) const 
            { 
                float* buff = (float*)buffer;
                buff[0] = x;
                buff[1] = y;
                buff[2] = theta;
            }

            void deserialize(uint8_t* buffer)
            {
                float* buff = (float*)buffer;

                x = buff[0];
                y = buff[1];
                theta = buff[2];

                //Keep theta between -pi and pi
                theta = atan2(sin(theta), cos(theta));
            }

            float x;
            float y;
            float theta;

        private:
            static size_t const _msg_size;
    };*/  
    struct Imu
    {   
        public:            
            explicit Imu(double a_x, double a_y, double v_theta) : 
            a_x{a_x}, a_y{a_y}, v_theta{v_theta} {}
            explicit Imu(ros_msgs_lw::Imu const& imu);           
            Imu() : a_x{0}, a_y{0}, v_theta{0} {}            
            size_t getSize() const 
            { 
                return _msg_size; 
            }           
            void allocateMemory(int32_t msg_len) {}
            static std::string getMsgType()
            {
                return "sensor_msgs/Imu";
            }
            void serialize(uint8_t* buffer) const
            { 
                double* buff = (double*)buffer;
                buff[0] = a_x;
                buff[1] = a_y;
                buff[2] = v_theta;
            }
            void deserialize(uint8_t* buffer)
            {
                double* buff = (double*)buffer;
                a_x = buff[0];
                a_y = buff[1];
                v_theta = buff[2];
            }          
            double a_x;
            double a_y;
            double v_theta;
        private:
            static size_t const _msg_size;
    };
	struct PoseQual
    {
		public: 
			explicit PoseQual(uint8_t poseQual) : q{q} {}
			explicit PoseQual(ros_msgs_lw::PoseQual const& qual);			
			PoseQual() : q{0} {}
            			
            size_t getSize() const 
            { 
                return _msg_size; 
            }            
            void allocateMemory(int32_t msg_len) {}
            static std::string getMsgType()
            {
                return "std_msgs/UInt8";
            }            
            void serialize(uint8_t* buffer) const
            { 				
                buffer[0] = q;
            }
            void deserialize(uint8_t* buffer)
            {
                uint8_t* buff = (uint8_t*)buffer;
                q = buff[0];
            }
            uint8_t q;
        private:
            static size_t const _msg_size;
	};    
    struct Pose2D
    {   
        public:            
            explicit Pose2D(double x, double y, double theta) : x{x}, y{y}, theta{theta} 
            {
                //Keep theta between -pi and pi
                theta = atan2(sin(theta), cos(theta));
            }            
            //explicit Pose2D(ros_msgs::Pose2DSim const& pose) : x{pose.x}, y{pose.y}, theta{pose.theta} {}
            explicit Pose2D(ros_msgs_lw::Pose2D const& pose);           
            Pose2D() : x{0}, y{0}, theta{0} {}            
            size_t getSize() const 
            { 
                return _msg_size; 
            }           
            void allocateMemory(int32_t msg_len) {}
            static std::string getMsgType()
            {
                return "geometry_msgs/Pose2D";
            }
            void serialize(uint8_t* buffer) const
            { 
                // fill buffer with data from x, y, theta
                double* buff = (double*)buffer;
                buff[0] = x;
                buff[1] = y;
                buff[2] = theta;
            }
            void deserialize(uint8_t* buffer)
            {
                double* buff = (double*)buffer;
                x = buff[0];
                y = buff[1];
                theta = buff[2];
                //Keep theta between -pi and pi
                theta = atan2(sin(theta), cos(theta));
            }           
            void operator=(Pose2D const& pose)
            {
                x = pose.x;
                y = pose.y;
                theta = pose.theta;
                //Keep theta between -pi and pi
                theta = atan2(sin(theta), cos(theta));
            }          
            /*void operator=(Pose2DSim const& pose)
            {
                x = pose.x;
                y = pose.y;
                theta = pose.theta;

                //Keep theta between -pi and pi
                theta = atan2(sin(theta), cos(theta));
            }*/
            double x;
            double y;
            double theta;
        private:
            static size_t const _msg_size;
    };
    struct Twist2D
    {
        public:
            explicit Twist2D(double x, double w) : v{x}, w{w} {}
            explicit Twist2D(ros_msgs_lw::Twist2D const& velocity);
            Twist2D() : v{0}, w{0} {}

            size_t getSize() const
            { 
                return _msg_size; 
            }
            
            void allocateMemory(int32_t msg_len) {}

            static std::string getMsgType()
            {
                return "geometry_msgs/Twist";
            }

            void serialize(uint8_t* buffer) const
            { 
                double* buff = (double*)buffer;
                buff[0] = v;
                buff[1] = w;
            }

            void deserialize(uint8_t* buffer)
            {
                double* buff = (double*)buffer;

                v = buff[0];
                w = buff[1];
            }       

            void operator=(Twist2D const& velocity)
            {  
                v = velocity.v;
                w = velocity.w;
            }

            void operator=(double i)
            {  
                v = i;
                w = i;
            }

            double v;
            double w;

        private:
            static size_t const _msg_size;
    };

    struct Point2D
    {   
        public:
            explicit Point2D(double x, double y) : x{x}, y{y} {}
            explicit Point2D(ros_msgs_lw::Point2D const& point);
            Point2D() : x{0}, y{0} {}

            size_t getSize() const 
            { 
                return _msg_size; 
            }            
            void allocateMemory(int32_t msg_len) {}
            static std::string getMsgType()
            {
                return "geometry_msgs/Point";
            }
            void serialize(uint8_t* buffer) const
            { 
                double* buff = (double*)buffer;
                buff[0] = x;
                buff[1] = y;
            }
            void deserialize(uint8_t* buffer)
            {
                double* buff = (double*)buffer;
                x = buff[0];
                y = buff[1];
            }
            double x;
            double y;
        private:
            static size_t const _msg_size;
    };
	struct TrajectoryStateVector
    {
        public:
            explicit TrajectoryStateVector(float x, float y, float dx, float dy, float ddx, float ddy, float theta, uint64_t timestamp) : x{x}, y{y}, dx{dx}, 
                dy{dy}, ddx{ddx}, ddy{ddy}, timestamp{timestamp} {}
            TrajectoryStateVector() : x{0}, y{0}, dx{0}, dy{0}, ddx{0}, ddy{0}, timestamp{0} {} 

            static size_t getSize()
            { 
                return _msg_size; 
            }

            void allocateMemory(int32_t msg_len) {}

            static std::string getMsgType()
            {
                return "trajecgenerator/c_trajec";
            }

            void serialize(uint8_t* buffer) const
            { 
                float* buff = (float*)buffer;
                buff[0] = x;
                buff[1] = y;
                buff[2] = dx;
                buff[3] = dy;
                buff[4] = ddx;
                buff[5] = ddy;

                *reinterpret_cast<uint64_t*>(&buff[6]) = timestamp;
            }

            void deserialize(uint8_t* buffer)
            {   
                float* buff = (float*)buffer;

                x = buff[0];
                y = buff[1];
                dx = buff[2];
                dy = buff[3];
                ddx = buff[4];
                ddy = buff[5];

                timestamp = *reinterpret_cast<uint64_t*>(&buff[6]);
            }

            float x;
            float y;
            float dx;
            float dy;
            float ddx;
            float ddy;
            uint64_t timestamp;

        private:
            static size_t const _msg_size;
    };

    struct Trajectory
    {
        public:
            Trajectory() {}
            ~Trajectory()
            {
                delete[] trajectory;
            }

            size_t getSize() const 
            {
                if(_trajectory_points == 0)
                    return 0;

                return _trajectory_points * TrajectoryStateVector::getSize() + sizeof(int32_t);
            }
            
            void allocateMemory(int32_t msg_len) 
            {
                _trajectory_points = msg_len / TrajectoryStateVector::getSize();

                if(trajectory != nullptr)
                    delete trajectory;
                
                trajectory = new TrajectoryStateVector[_trajectory_points];
            }

            static std::string getMsgType()
            {
                return "trajecgenerator/c_trajec_vector";
            }

            size_t getTrajectorySize() const
            {
                return _trajectory_points; 
            }

            void serialize(uint8_t* buffer) const
            {   
                if(_trajectory_points > 0)   
                {
                    *reinterpret_cast<int32_t*>(buffer) = _trajectory_points * TrajectoryStateVector::getSize();
                    buffer += sizeof(int32_t);

                    for(int i = 0; i < _trajectory_points; i++)
                    {
                        trajectory[i].serialize(buffer);

                        buffer += TrajectoryStateVector::getSize();
                    }
                }
            }

            void deserialize(uint8_t* buffer)
            {
               for(int i = 0; i < _trajectory_points; i++)
               {
                   trajectory[i].deserialize(buffer);

                   buffer += TrajectoryStateVector::getSize();
               }
            }

            ros_msgs::TrajectoryStateVector& operator[](int i) const
            {
                return trajectory[i];
            }

        private:
            uint32_t _trajectory_points = 0;
            ros_msgs::TrajectoryStateVector* trajectory = nullptr;
    };
}


