#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_log.h"

#include "Wifi.h"
#include "SensorValue.h"
#include "Marvelmind.h"
#include "NodeHandle.h"
#include "Publisher.h"
#include "RosMsgsLw.h"
#include "RosMsgs.h"
#include "StateMachine.h"
#include "OutputVelocity.h"
#include "OutputVelocitySim.h"
#include "OutputVelocityImpl.h"
//#include "Kalman.h"
#include "Motor.h"
#include "MotorController.h"
#include "ControllerMaster.h"
#include "LedStrip.h"

#define TAG "Main"

#define DATA_LOGGING
#include "DataLogger.h"

//#define USE_SIM
#define KALMAN
//#define STEP_RESPONSE

#define ROS_SOCKET_PORT CONFIG_ROS_SOCKET_PORT
#define SERVER_IP_ADDR CONFIG_SERVER_IP_ADDR


extern "C" void app_main(void)
{   
	//Initialize NVS
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
	
	Wifi& wifi = Wifi::init();
	wifi.begin();
	Socket& ros_socket = *new Socket(ROS_SOCKET_PORT, SERVER_IP_ADDR);
	ros::NodeHandle& node_handle = ros::NodeHandle::init("robot_1", ros_socket);
	
	Motor& motor_a = Motor::init_a();
	Motor& motor_b = Motor::init_b();
	MotorController& motor_controller = MotorController::init(motor_a, motor_b);
	
	/*#ifdef KALMAN
	SensorPoseSim& sim_sensor = SensorPoseSim::init(node_handle);
	SensorPose& pose_sensor = KalmanFilter::init({&sim_sensor}, output_velocity);
	#else
	SensorPose& pose_sensor = SensorPoseSim::init(node_handle);
	#endif*/
	
	
	#ifdef STEP_RESPONSE
		motor_controller.disablePIcontrol();
	#endif
	
	OutputVelocity& output_velocity = OutputVelocityImpl::init(motor_controller);
	
	Marvelmind marvelmind_sensor;
	SensorValue<ros_msgs_lw::Pose2D> pose;
	#ifdef KALMAN
		//Kalman kalman_filter;
		//pose = kalman_filter.pose;
	#endif
	
	#ifdef DATA_LOGGING
		ros::Publisher<ros_msgs::String>& data_log_publisher = node_handle.advertise<ros_msgs::String>("data_log");
		DataLogger& data_logger = DataLogger::init(data_log_publisher);
		node_handle.subscribe<ros_msgs::String>("start_log", std::bind(&DataLogger::startLogging, &data_logger, std::placeholders::_1));
	#endif
	
	ControllerMaster& controller_master = ControllerMaster::init(output_velocity, pose);
	
	ros::Publisher<ros_msgs::Pose2D>& pose_feedback = node_handle.advertise<ros_msgs::Pose2D>("pose2D");
	ros::Publisher<ros_msgs::PoseQual>& qual_feedback = node_handle.advertise<ros_msgs::PoseQual>("qual");
	ros::Publisher<ros_msgs::Imu>& imu_feedback = node_handle.advertise<ros_msgs::Imu>("imu");
	
	StateMachine& state_machine = StateMachine::init(controller_master, output_velocity);
	node_handle.subscribe<ros_msgs::Point2D>("goal_point", std::bind(&StateMachine::set_goal_point, &state_machine, std::placeholders::_1));
	node_handle.subscribe<ros_msgs::Twist2D>("vel", std::bind(&StateMachine::set_velocity, &state_machine, std::placeholders::_1));
	node_handle.subscribe<ros_msgs::Trajectory>("trajectory", std::bind(&StateMachine::set_trajectory, &state_machine, std::placeholders::_1));
	node_handle.registerConnectionLostCallback(std::bind(&StateMachine::stop, &state_machine));
	
	LedStrip& led_strip = LedStrip::init();
	node_handle.subscribe<ros_msgs::String>("led", std::bind(&LedStrip::animation_callback, &led_strip, std::placeholders::_1));
	
	while(1) 
	{ 
		ros_msgs_lw::Pose2D pose;
		ros_msgs_lw::PoseQual poseQual;
		ros_msgs_lw::Imu imu;
		
		if(marvelmind_sensor.pose->peekAtValue(pose))
		{
			ESP_LOGI(TAG, "X: %f, Y: %f, Theta: %f", pose.x, pose.y,  pose.theta);		
			ros_msgs::Pose2D pose_msg(pose);
			pose_feedback.publish(pose_msg);
		}
		if(marvelmind_sensor.poseQual->peekAtValue(poseQual))	
		{
			ros_msgs::PoseQual qual_msg(poseQual);
			qual_feedback.publish(qual_msg);
		}		
		if(marvelmind_sensor.imu->peekAtValue(imu))
		{   
			ros_msgs::Imu imu_msg(imu);
			imu_feedback.publish(imu_msg);
		}		
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}
