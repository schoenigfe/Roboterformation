#pragma once

#include "PositionController.h"
#include "RosMsgsLw.h"
#include "RosMsgs.h"
#include "esp_timer.h"

#include "math.h"

#include <memory>

/**
 * @brief PositionController implementation -> check Projektarbeit Regelung und Simulation von Schwarmrobotern
 */
class approxLinController : public PositionController
{
    public:
        explicit approxLinController(std::shared_ptr<ros_msgs::Trajectory> trajectory);

        ros_msgs_lw::Twist2D update(ros_msgs_lw::Pose2D const& actual_pose) override;
        bool destination_reached() override;

    private:
        float _damping_coefficient = 0.5;
        float _natural_frequency = 1.;

        std::shared_ptr<ros_msgs::Trajectory> _trajectory;
        uint32_t _trajectory_cntr = 0;

        int64_t _state_vector_time_difference_us;

        bool _destination_reached = false;
};