#pragma once

#include "OutputVelocity.h"
#include "MotorController.h"

#include "math.h"

/**
 * @brief OutputVelocityImpl transforms the velocity vector (m/s, RAD/s) to motor speeds (RAD/s) for the MotorController object.
 */
class OutputVelocityImpl : public OutputVelocity
{
    public:
        /**
         * @brief Initialize the OutputVelocityImpl instance
         * 
         * @note It is safe to call this function multiple times. It will only create one instance.
         * 
         * @param [in] motor_controller reference to the motor controller object
         * @return Reference to the OutputVelocity instance
         */ 
        static OutputVelocity& init(MotorController& motor_controller);

        /**
         * @brief This function does the differential drive forward kinematic 
         * and passes the resulting setpoint motor velocities to the MotorController object
         * 
         * @param [in] velocity velocity vector
         */  
        void setVelocity(ros_msgs_lw::Twist2D const& velocity) override;

    private:
        OutputVelocityImpl(MotorController& motor_controller);
        OutputVelocityImpl(OutputVelocityImpl const&) = delete;
        ~OutputVelocityImpl() {}

        MotorController& _motor_controller;
};