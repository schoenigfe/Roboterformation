#pragma once

#include <initializer_list>
#include <vector>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "math.h"

#include "SensorPose.h"
#include "KalmanSensor.h"
#include "OutputVelocity.h"
#include "RosMsgsLw.h"
#include "mat.h"

class KalmanFilter : public SensorPose
{
    public:
         /**
         * @brief Initialize the KalmanFilter instance
         * 
         * @note It is safe to call this function multiple times. It will only create one instance.
         * 
         * @param [in] sensor_list list of all the KalmanSensors which are fused by the KalmanFilter. The first sensor the list is used to initialize the KalmanFilter
         * @param [in] output_velocity reference to the OutputVelocity object to obtain the velocity vector for the state estimation step
         * @return Reference to the KalmanFilter instance
         */
        static SensorPose& init(std::initializer_list<KalmanSensor const*> const& sensor_list, OutputVelocity const& output_velocity);

        bool peekAtPose(ros_msgs_lw::Pose2D& current_pose) const override;
        bool getPose(ros_msgs_lw::Pose2D& current_pose) const override;
        void reInit() override;

    private:
        KalmanFilter(std::initializer_list<KalmanSensor const*> const& sensor_list, OutputVelocity const& output_velocity);
        KalmanFilter(KalmanFilter const&) = delete;
        ~KalmanFilter();
        
        static void _kalman_filter_loop_timer(TimerHandle_t timer);
        static void _kalman_filter_loop_task(void* pvParameters);

        static KalmanFilter* _kalman_filter;

        //The first sensor in the vector must be of absolute type
        std::vector<KalmanSensor const*> _sensor_list;

        OutputVelocity const& _output_velocity;

        ros_msgs_lw::Pose2D _a_posterior_estimate;
        dspm::Mat _a_posterior_cov;

        float const _process_noise_variance = 0.0000001;
        dspm::Mat _process_noise_cov;

        uint64_t _timestamp_us;

        QueueHandle_t _current_pose_queue;
        QueueHandle_t _peek_at_pose_queue;

        SemaphoreHandle_t _reinitialize_sensor_semphr;

        TimerHandle_t _kalman_filter_loop_timer_handle;
        TaskHandle_t _kalman_filter_loop_task_handle;
};