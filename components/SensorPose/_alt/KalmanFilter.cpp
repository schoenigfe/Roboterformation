#include "KalmanFilter.h"

#include "esp_log.h"

#define TAG "KalmanFilter"


KalmanFilter* KalmanFilter::_kalman_filter = nullptr;

KalmanFilter::KalmanFilter(std::initializer_list<KalmanSensor const*> const& sensor_list, OutputVelocity const& output_velocity) : 
    _sensor_list(sensor_list), _output_velocity{output_velocity}, _a_posterior_cov(3, 3), _process_noise_cov(_process_noise_variance * dspm::Mat::eye(3))
{   
    _reinitialize_sensor_semphr = xSemaphoreCreateBinary();
    xSemaphoreGive(_reinitialize_sensor_semphr);

    _current_pose_queue = xQueueCreate(1, sizeof(ros_msgs_lw::Pose2D));
    _peek_at_pose_queue = xQueueCreate(1, sizeof(ros_msgs_lw::Pose2D));

    xTaskCreate(_kalman_filter_loop_task, "_kalman_filter_loop_task", 8192, this, 9, &_kalman_filter_loop_task_handle);

    _kalman_filter_loop_timer_handle = xTimerCreate("_kalman_filter_loop", pdMS_TO_TICKS(10), pdTRUE, NULL, _kalman_filter_loop_timer);
    xTimerStart(_kalman_filter_loop_timer_handle, portMAX_DELAY);
}

KalmanFilter::~KalmanFilter()
{
    xTimerDelete(_kalman_filter_loop_timer_handle, portMAX_DELAY);
    vTaskDelete(_kalman_filter_loop_task_handle);

    vQueueDelete(_current_pose_queue);
    vQueueDelete(_peek_at_pose_queue);
}

SensorPose& KalmanFilter::init(std::initializer_list<KalmanSensor const*> const& sensor_list, OutputVelocity const& output_velocity)
{
    if(_kalman_filter == nullptr)
        _kalman_filter = new KalmanFilter(sensor_list, output_velocity);

    return *_kalman_filter;
}

bool KalmanFilter::peekAtPose(ros_msgs_lw::Pose2D& current_pose) const
{
    if(xQueuePeek(_peek_at_pose_queue, &current_pose, 0) == pdPASS)
        return true;

    return false;
}

bool KalmanFilter::getPose(ros_msgs_lw::Pose2D& current_pose) const
{
    if(xQueueReceive(_current_pose_queue, &current_pose, 0) == pdPASS)
        return true;

    return false;
}

void KalmanFilter::reInit()
{   
    xSemaphoreGive(_reinitialize_sensor_semphr);
}

void KalmanFilter::_kalman_filter_loop_timer(TimerHandle_t timer)
{
    xTaskNotifyGive(_kalman_filter->_kalman_filter_loop_task_handle);
}

void KalmanFilter::_kalman_filter_loop_task(void* pvParameters)
{
    KalmanFilter& kalman_filter = *(reinterpret_cast<KalmanFilter*>(pvParameters));

    kalman_filter._timestamp_us = esp_timer_get_time();

    dspm::Mat lin_state_trans_fnc_transp(3, 3);
    dspm::Mat lin_state_trans_fnc(3, 3);

    dspm::Mat a_priori_cov(3, 3);

    while(1)
    {
        if(xSemaphoreTake(kalman_filter._reinitialize_sensor_semphr, 0) == pdPASS)
        {
            while(kalman_filter._sensor_list[0]->getAbsolutePose(kalman_filter._a_posterior_estimate) == false)
                vTaskDelay(500 / portTICK_PERIOD_MS);

            kalman_filter._sensor_list[0]->getMeasurementNoiseCov(kalman_filter._a_posterior_cov);
        }

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint64_t current_timestamp_us = esp_timer_get_time();
        float delta_time = static_cast<float>(current_timestamp_us - kalman_filter._timestamp_us) / 1000000.;
        kalman_filter._timestamp_us = current_timestamp_us;

        ros_msgs_lw::Twist2D cmd_vel = kalman_filter._output_velocity.getVelocity();


        //linearize transition function
        lin_state_trans_fnc(0, 0) = 1;
        lin_state_trans_fnc(0, 1) = 0;
        lin_state_trans_fnc(0, 2) = -cmd_vel.v * sin(kalman_filter._a_posterior_estimate.theta) * delta_time; 
        lin_state_trans_fnc(1, 0) = 0;
        lin_state_trans_fnc(1, 1) = 1;
        lin_state_trans_fnc(1, 2) = cmd_vel.v * cos(kalman_filter._a_posterior_estimate.theta) * delta_time; 
        lin_state_trans_fnc(2, 0) = 0;
        lin_state_trans_fnc(2, 1) = 0;
        lin_state_trans_fnc(2, 2) = 1;

        lin_state_trans_fnc_transp = lin_state_trans_fnc.t();


        //Predict
        ros_msgs_lw::Pose2D delta_pose;
        delta_pose.x = cmd_vel.v * cos(kalman_filter._a_posterior_estimate.theta) * delta_time;
        delta_pose.y = cmd_vel.v * sin(kalman_filter._a_posterior_estimate.theta) * delta_time;
        delta_pose.theta = cmd_vel.w * delta_time;

        ros_msgs_lw::Pose2D a_priori_estimate;
        a_priori_estimate = kalman_filter._a_posterior_estimate + delta_pose;

        a_priori_cov = lin_state_trans_fnc * kalman_filter._a_posterior_cov * lin_state_trans_fnc_transp + kalman_filter._process_noise_cov;

        //ESP_LOGI(TAG, "a priori: %f, %f, %f", a_priori_estimate.x, a_priori_estimate.y, a_priori_estimate.theta);

        
        //Update
        bool update = false;
        
        for(KalmanSensor const* i : kalman_filter._sensor_list)
        {
            if(i->calculateKalman(a_priori_estimate, a_priori_cov, kalman_filter._a_posterior_estimate, kalman_filter._a_posterior_cov) == true)
            {
                //ESP_LOGI(TAG, "a posterior: %f, %f, %f", kalman_filter._a_posterior_estimate.x, kalman_filter._a_posterior_estimate.y, kalman_filter._a_posterior_estimate.theta);
                update = true;

                a_priori_estimate = kalman_filter._a_posterior_estimate;
                a_priori_cov = kalman_filter._a_posterior_cov;
            }
        }
        
        if(update == false)
        {
            kalman_filter._a_posterior_estimate = a_priori_estimate;
            kalman_filter._a_posterior_cov = a_priori_cov;
        }

        xQueueOverwrite(kalman_filter._current_pose_queue, &kalman_filter._a_posterior_estimate);
        xQueueOverwrite(kalman_filter._peek_at_pose_queue, &kalman_filter._a_posterior_estimate);
    }
}