#pragma once

#include <array>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/uart.h"

#include "RosMsgsLw.h"
#include "SensorPose.h"
#include "KalmanSensor.h"
#include "mat.h"


/**
 * The Marvelmind class drives the UART communication with the Marvelmind RX Beacon.
 * It can be used as a standalone SensorPose or as a source for the KalmanFilter.
 */
class Marvelmind : public KalmanSensor, public SensorPose
{
    public:
        /**
         * @brief Initialize the Marvelmind instance
         * 
         * @note It is safe to call this function multiple times. It will only create one instance.
         * 
         * @return Reference to the Marvelmind instance
         */ 
        static Marvelmind& init();

        
        bool peekAtPose(ros_msgs_lw::Pose2D& current_pose) const override; 
        bool getPose(ros_msgs_lw::Pose2D& current_pose) const override;
        bool getIMU(ros_msgs_lw::Imu& current_imu) const; //override;         
        void reInit() override {}

        void calculateMeasurementNoiseCov() const override;
        bool calculateKalman(ros_msgs_lw::Pose2D const& a_priori_estimate, dspm::Mat const& a_priori_cov, ros_msgs_lw::Pose2D& a_posterior_estimate, dspm::Mat& a_posterior_cov) const override;
        bool getAbsolutePose(ros_msgs_lw::Pose2D& initial_pose) const override;
        void getMeasurementNoiseCov(dspm::Mat& measurement_cov) const override;

    private:
        Marvelmind();
        Marvelmind(Marvelmind const&) = delete;
        ~Marvelmind();

        static void _uart_read_data_task(void* pvParameters);

        static Marvelmind* _marvelmind_sensor;

        QueueHandle_t _current_pose_queue;
        QueueHandle_t _peek_at_pose_queue;
        QueueHandle_t _current_imu_queue;

        dspm::Mat _measurement_noise_cov;

        static const uart_port_t _uart_port; 
        static const uart_config_t _uart_conf;

        TaskHandle_t _uart_read_data_task_handle;
};
