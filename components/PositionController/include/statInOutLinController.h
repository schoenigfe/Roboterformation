#pragma once

#include "PositionController.h"
#include "RosMsgsLw.h"
#include "RosMsgs.h"
#include "esp_timer.h"

#include <memory>

/**
 * @brief PositionController implementation -> check Projektarbeit Regelung und Simulation von Schwarmrobotern
 */
class statInOutLinController : public PositionController
{
    public:
        explicit statInOutLinController(std::shared_ptr<ros_msgs::Trajectory> trajectory);
        ros_msgs_lw::Twist2D update(ros_msgs_lw::Pose2D const& actual_pose) override;
        bool destination_reached() override;

    private:
        float _kp_1 = 0.3;
        float _kp_2 = 0.3;
        float _b_offset = 0.2;

        std::shared_ptr<ros_msgs::Trajectory> _trajectory;
        uint32_t _trajectory_cntr = 0;

        int64_t _state_vector_time_difference_us;

        bool _destination_reached = false;
};