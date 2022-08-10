#pragma once

#include "PositionController.h"
#include "RosMsgsLw.h"
#include "RosMsgs.h"

#include "esp_timer.h"

#include <memory>


/**
 * @brief PositionController implementation -> check Projektarbeit Regelung und Simulation von Schwarmrobotern
 */
class dynInOutLinController : public PositionController
{
    public:
        explicit dynInOutLinController(std::shared_ptr<ros_msgs::Trajectory> trajectory);
        ros_msgs_lw::Twist2D update(ros_msgs_lw::Pose2D const& actual_pose) override;
        bool destination_reached() override;

    private:
        float _kp_1 = 0.3;
        float _kp_2 = 0.3;
        float _kd_1 = 0.6;
        float _kd_2 = 0.6;

        uint64_t _prev_time_us;
        float _prev_velocity = 0.1;
        float _prev_acceleration = 0;

        std::shared_ptr<ros_msgs::Trajectory> _trajectory;
        uint32_t _trajectory_cntr = 0;

        int64_t _state_vector_time_difference_us;

        bool _destination_reached = false;
};