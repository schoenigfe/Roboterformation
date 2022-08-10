#pragma once

#include "math.h"

#include "PositionController.h"
#include "RosMsgsLw.h"



/**
 * @brief PositionController implementation -> check Projektarbeit Regelung und Simulation von Schwarmrobotern
 */
class p2pController : public PositionController
{
    public:
        explicit p2pController(ros_msgs_lw::Point2D const& goal_point);

        ros_msgs_lw::Twist2D update(ros_msgs_lw::Pose2D const& actual_pose) override;
        bool destination_reached() override;

    private:
        float _kp_v = 0.2;
        float _kp_w = 1.0;

        ros_msgs_lw::Point2D const _goal_point;
        
        bool _destination_reached = false;
};