#include "Kalman.h"
#include "esp_log.h"

#define TAG "Kalman"


Kalman::Kalman(Marvelmind*)
{ 
    //xTaskCreate(_kalman_task, "_kalman_task", 8192, this, 9, NULL);
	
	//DeltaTime
	dt_gps=0.1; //insert actual time difference here
	
	/**Matrix A (Transition Matrix)   A = [1 0 0 dt 0 0;
										 0 1 0 0 dt 0;
										 0 0 1 0 0 0;
										 0 0 0 1 0 0;
										 0 0 0 0 1 0;
										 0 0 0 0 0 0]; */
	A = dspm::Mat::eye(6);
    A(0, 3) = dt_gps;
    A(1, 4) = dt_gps;
    A(5, 5) = 0;

	
	/**Matrix H (Observation Matrix)    H = [1 0 0 0 0 0;
										 0 1 0 0 0 0;
										 0 0 1 0 0 0;
										 0 0 0 1 0 0;
										 0 0 0 0 1 0;
										 0 0 0 0 0 1]; */										 				
	H = dspm::Mat::eye(6);
	
	/**Matrix P (Covariance Matrix)		P = [1 0 0 0 0 0;
											 0 1 0 0 0 0;
											 0 0 1 0 0 0;
											 0 0 0 1 0 0;
											 0 0 0 0 1 0;
											 0 0 0 0 0 1];*/
	P = dspm::Mat::eye(6);
	
	/**Matrix Q (Uncertainty of Kinematic Model) Q = [0.25*ax 0       0           0.5*ax 0      0;
													 0       0.25*ay 0           0      0.5*ay 0;
													 0       0       0.25*atheta 0      0      0.5*atheta;
													 0.5*ax  0       0           ax     0      0;
													 0       0.5*ay  0           0      ay     0;
													 0       0       0.5*atheta  0      0      atheta];*/
    //define uncertainty of x,y,theta dimensions
	ax = 1e-2;
	ay = 1e-2;
	atheta = 1e-2;
	//set 
	Q = dspm::Mat::eye(6);
	Q(0,0) = 0.25*ax;
	Q(0,3) = 0.5*ax;
	Q(1,1) = 0.25*ay;
	Q(1,4) = 0.5*ay;
	Q(2,2) = 0.25*atheta;
	Q(2,5) = 0.5*atheta;
	Q(3,0) = 0.5*ax;
	Q(3,3) = 1.0*ax;
	Q(4,1) = 0.5*ay;
	Q(4,4) = 1.0*ay;
	Q(5,2) = 0.5*atheta;
	Q(5,5) = 1.0*atheta;
	
	X(0,0) = 0; //take pose2D instead as initial value?
	X(1,0) = 0; //take pose2D instead as initial value?
	X(2,0) = 0; //take pose2D instead as initial value?
	X(3,0) = 0;	//start with zero velocity
	X(4,0) = 0;
	X(5,0) = 0;
}
Kalman::~Kalman()
{   	
    //xTimerDelete(_kalman_timer_handle, portMax_Delay);
    //vTaskDelete(_kalman_task_handle);
	//vTaskDelete(_kalman_task_handle);
}
void Kalman::calculate_imu()
{
	//get pose2D aposteriori
	x = X(0, 0);
	y = X(1, 0);
	theta = X(2, 0);
	
	//get DeltaTime
	dt_gps=0.1;
    dt_imu=0.01;///get real clock difference here!!!!

	//get State_Vector
	Z(0,0) = 0;///////get GPS Data here!!!!
	Z(0,1) = 0;///////get GPS Data here!!!!
	Z(0,2) = 0;///////get GPS Data here!!!!
	Z(0,3) = 0;///////get global Velocity here!!!!
	Z(0,4) = 0;///////get global Velocity here!!!!
	Z(0,5) = 0;///////get global Velocity here!!!!
	
	//get IMU Data
	U(0,0)=0; //get IMU dat from IMU!!!
	U(0,1)=0; //get IMU dat from IMU!!!
	U(0,2)=0; 
	U(0,3)=0;
	U(0,4)=0;
	U(0,5)=0;
	U(0,6)=0; //get IMU dat from IMU!!!

	//get Signal Quality
	signal_quality=100; /// get real signal quality !!!
	
	
	if(signal_quality>50){
		rxy=0.01;
		rt=0.01;
		rxy_dot=0.01;
		rt_dot=0.01;
	}
	else{
		rxy=1000;
		rt=1000;
		rxy_dot=1000;
		rt_dot=1000;
	}
	 
	/*Rauschmatrix  					R = [rxy 0   0     0       0        0;
											 0   rxy 0     0       0        0;
											 0   0  rtheta 0       0        0;
											 0   0   0   rxy_dot   0        0;
											 0   0   0     0    rxy_dot     0;
											 0   0   0     0       0    rtheta_dot];*/
	R = dspm::Mat::eye(6);
	R(0,0) = rxy;
	R(1,1) = rxy;
	R(2,2) = rt;
	R(3,3) = rxy_dot;
	R(4,4) = rxy_dot;
	R(5,5) = rt_dot;
	
	
	//get gyro_z from Imu Data
	gyro_z =U(0,6);
	
///calulate_gps()   
    //update DeltaTime of Matrix A with Clockdifference
    A(0, 3) = dt_gps;
    A(1, 4) = dt_gps;
    A(2, 4) = dt_gps;
    
    /**Matrix B			B = [0.5*dt*dt*cos(x(3,k)+imu(k,6)*dt) -0.5*dt*dt*sin(x(3,k)+imu(k,6)*dt) 0 0 0 0;    %x from ax,ay and x(k,3)
							 0.5*dt*dt*sin(x(3,k)+imu(k,6)*dt)  0.5*dt*dt*cos(x(3,k)+imu(k,6)*dt) 0 0 0 0;    %y from ax,ay and x(k,3) 
							 0                            						    0             0 0 0 dt;   %theta from wz                 
							 dt*cos(x(3,k)+imu(k,6)*dt)      -dt*sin(x(3,k)+imu(k,6)*dt)          0 0 0 0;    %x° from ax,ay and x(k,3)
							 dt*sin(x(3,k)+imu(k,6)*dt)       dt*cos(x(3,k)+imu(k,6)*dt)          0 0 0 0;    %y° from ax,ay and x(k,3)
							 0                         								0             0 0 0 1];   %theta°=wz*/
			 
	for(int i=0; i<6;i++)
	{
		for(int j=0; j<6;j++)
		{
			B(i,j)=0;
		}
	}
	B(0,0) = 0.5*dt*dt*cos(theta+gyro_z*dt);
	B(0,1) = -0.5*dt*dt*sin(theta+gyro_z*dt);
	B(1,0) = 0.5*dt*dt*sin(theta+gyro_z*dt);
	B(1,1) = 0.5*dt*dt*cos(theta+gyro_z*dt);
	B(2,5) = dt;
	B(3,0) = dt*cos(theta+gyro_z*dt);
	B(3,1) = -dt*sin(theta+gyro_z*dt);
	B(4,0) = dt*sin(theta+gyro_z*dt);
	B(4,1) = dt*cos(theta+gyro_z*dt);
	B(5,5) = dt;
	
	//calculate x_prio
	X_prio = A*X+B*U;
	
	//Calculate uncertainty of estimation
	P = A*P*A.t()+Q;
	
	//Observe State Vector with gps scope
	Y = H*X_prio;
	
	//Calculate  Kalman_Gain
	K = P*H.t()*(H*P*H.t()+R);
	
	//Update State_Vector
	X = X_prio+K*(Z-Y);
	
	//Update Covariance Matrix
	P = (dspm::Mat::eye(6)-K*H)*P;  
}
void Kalman::calculate_gps()
{
		ros_msgs_lw::Pose2D new_pose;
		kalmanPose.overwriteValue(new_pose);
}
void Kalman::_kalman_task(void* pvParameters) 
{
	timestamp_imu = Marvelmind::imu->getTimestamp();
	new_measurement_imu = (last_timestamp_imu != timestamp_imu);
	new_measurement_gps = (last_timestamp_gps != timestamp_gps);
	if(new_measurement_imu){
		dt_imu = (int)(last_timestamp_imu-timestamp_imu);
		Marvelmind::imu->getValue(imu);
		//printf("-----imu_test: ", imu.angular_velocity_z);
		/*ax = imu.angular_velocity_x;
		ay = imu.angular_velocity_y;
		az = imu.angular_velocity_z;*/		
		calculate_imu();
	}
	if(new_measurement_gps){
		dt_gps = (int)(last_timestamp_gps-timestamp_gps);
		//Marvelmind::pose->getValue(pose);
		//Marvelmind::pose->getValue(poseQual);
		calculate_gps();
	}	
	new_measurement_imu = false;
	new_measurement_gps = false;
}

