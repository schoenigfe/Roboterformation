#pragma once 

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "SensorPose.h"
#include "RosMsgsLw.h"
#include "mat.h"

#define KALMAN_SENSOR_COVARIANCE_CALCULATION_SAMPLES 256

class KalmanSensor
{
    public:
        KalmanSensor() {}
        ~KalmanSensor() {}

        /**
         * @brief This method can be used to calculate the noise covariance of the sensor.
         */
        virtual void calculateMeasurementNoiseCov() const = 0;

        /**
         * @brief This method is used to update the pose and covariance estimate of the KalmanFilter
         * with the measurements of underlying KalmanSensor
         * 
         * @param [in] a_priori_estimate
         * @param [in] a_priori_cov
         * @param [out] a_posterior_estimate
         * @param [out] a_posterior_cov
         * @return true if new measurement was available and the pose and covariance was updated
         */
        virtual bool calculateKalman(ros_msgs_lw::Pose2D const& a_priori_estimate, dspm::Mat const& a_priori_cov, ros_msgs_lw::Pose2D& a_posterior_estimate, dspm::Mat& a_posterior_cov) const = 0;

        /**
         * @brief This function is used for the reinitialization of the KalmanFilter.
         * 
         * @param [out] initial_pose
         * @return true if reading new pose was succesfull
         */
        virtual bool getAbsolutePose(ros_msgs_lw::Pose2D& initial_pose) const = 0;

        /**
         * @brief This function is used for the reinitialization of the KalmanFilter
         * 
         * @param [out] measurement_cov
         */
        virtual void getMeasurementNoiseCov(dspm::Mat& measurement_cov) const = 0;

};