#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "driver/mcpwm.h"
#include "soc/rtc.h"

/**
 * @brief This class controls Motor_A and Motor_B. 
 * It provides a callback function for the MCPWM capture component for measuring acutal motor speed, 
 * it sets the motor duty cycle and it provides a PI controller for controlling the motor speed. 
 */
class Motor
{
    public:
        static Motor& init_a();
        static Motor& init_b();

        /**
         * @brief Set the target angular velocity for the motor PI controller in RAD/s
         * 
         * @param [in] setpoint_velocity Target angular velocity RAD/s
         */
        void setSetpointVelocity(float setpoint_velocity);
        
        /**
         * @brief Get the target angular velocity of the motor in RAD/s
         * 
         * @return Target angular velocity 
         */
        float getSetpointVelocity() const;

        /**
         * @brief Get the actual velocity of the motor in RAD/s
         * 
         * @return Actual motor velocity 
         */
        float getActualVelocity();

        /**
         * @brief Calculate PI controller output for the next time step 
         * 
         * @note This function must be called periodically.
         * 
         * @param [in] actual_velocity current velocity of the motor
         * 
         * @return ouput duty cycle
         */
        float updatePIControl(float actual_velocity);

        /**
         * @brief Set output duty cycle for the motor
         * 
         * @param [in] duty_cycle Output duty cycle (Accepted range: -100 - +100)  
         */
        void setDuty(float duty_cycle);

    private:
        Motor(mcpwm_unit_t mcpwm_unit, mcpwm_pin_config_t motor_pins, bool motor_dir);
        ~Motor() {}
        Motor(Motor const&) = delete;
        
        /**
         * @brief Callback function for the Espressife IDF MCPWM Capture Module 
         * Calculates the time difference (_encoder_pulse_period) between two positive edges on CAP_0. The sign is obtained by checking if the last edge on CAP_1 was positive or negative. 
         * 
         * @note Check ESP-IDF Programming Guide for a description of the parameters
         */
        static bool IRAM_ATTR _encoder_callback(mcpwm_unit_t mcpwm_unit, mcpwm_capture_channel_id_t cap_channel, const cap_event_data_t *edata, void *user_data);

        static Motor* _motor_a;
        static Motor* _motor_b;

        float _kp = 20.;
        float _ki = 500.0;  

        /// Invert mapping between actual motor direction and sign of duty cycle
        bool _motor_dir;

        float _setpoint_velocity = 0;

        int64_t _prev_time_us = 0;
        float _error_integral = 0;

        uint32_t _encoder_timestamp;
        int32_t _encoder_pulse_period;
        int32_t _prev_pulse_period;

        float _current_duty_cycle;

        mcpwm_unit_t _mcpwm_unit;
        mcpwm_timer_t _mcpwm_timer;
        mcpwm_pin_config_t _motor_pins;
        mcpwm_config_t _mcpwm_config;

        mcpwm_capture_config_t _mcpwm_capture_config_0;
        mcpwm_capture_config_t _mcpwm_capture_config_1;
};