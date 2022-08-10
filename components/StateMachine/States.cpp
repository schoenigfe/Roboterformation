#include "States.h"
#include "StateMachine.h"

#include "approxLinController.h"
#include "dynInOutLinController.h"
#include "p2pController.h"
#include "statInOutLinController.h"

#define TAG "States"

std::string const Idle::_state = "Idle";
std::string const DriveWithVelocity::_state = "DriveWithVelocity";
std::string const DriveToPoint::_state = "DriveToPoint";
std::string const FollowTrajectory::_state = "FollowTrajectory";


void Idle::set_velocity(StateMachine& state_machine, std::shared_ptr<ros_msgs::Twist2D> vel_vector_msg)
{
    ros_msgs_lw::Twist2D vel_vector(*vel_vector_msg);
    
    state_machine.output_velocity.setVelocity(vel_vector);

    if (vel_vector.v != 0 || vel_vector.w != 0)
        state_machine.setState(new DriveWithVelocity);
    
}

void Idle::set_goal_point(StateMachine& state_machine, std::shared_ptr<ros_msgs::Point2D> goal_point)
{
    state_machine.controller_master.start_controller(new p2pController(ros_msgs_lw::Point2D(*goal_point)), std::bind(&StateMachine::stop,& state_machine));

    state_machine.setState(new DriveToPoint);
}

void Idle::set_trajectory(StateMachine& state_machine, std::shared_ptr<ros_msgs::Trajectory> trajectory)
{
    //state_machine.controller_master.start_controller(new approxLinController(trajectory), std::bind(&StateMachine::stop,& state_machine));
    //state_machine.controller_master.start_controller(new statInOutLinController(trajectory), std::bind(&StateMachine::stop, &state_machine));
    state_machine.controller_master.start_controller(new dynInOutLinController(trajectory), std::bind(&StateMachine::stop, &state_machine));
    
    state_machine.setState(new FollowTrajectory);
}

void DriveWithVelocity::set_velocity(StateMachine& state_machine, std::shared_ptr<ros_msgs::Twist2D> vel_vector_msg)
{
    
    ros_msgs_lw::Twist2D vel_vector(*vel_vector_msg);
    
    if (vel_vector.v == 0 && vel_vector.w == 0)
        state_machine.setState(new Idle);

    state_machine.output_velocity.setVelocity(vel_vector);
    
}

void DriveWithVelocity::stop(StateMachine& state_machine)
{
    state_machine.setState(new Idle);

    ros_msgs_lw::Twist2D vel_vector(0, 0);
    state_machine.output_velocity.setVelocity(vel_vector);
}

void DriveToPoint::set_velocity(StateMachine& state_machine, std::shared_ptr<ros_msgs::Twist2D> vel_vector_msg)
{
    ros_msgs_lw::Twist2D vel_vector(*vel_vector_msg);

    if (vel_vector.v == 0 && vel_vector.w == 0)
    {
        state_machine.controller_master.stop_controller();

        state_machine.setState(new Idle);
    } 
}

void DriveToPoint::set_goal_point(StateMachine& state_machine, std::shared_ptr<ros_msgs::Point2D> goal_point)
{
    state_machine.controller_master.start_controller(new p2pController(ros_msgs_lw::Point2D(*goal_point)), std::bind(&StateMachine::stop, &state_machine));

    state_machine.setState(new DriveToPoint);
}

void DriveToPoint::stop(StateMachine& state_machine)
{
    state_machine.controller_master.stop_controller();
    state_machine.setState(new Idle);
}

void FollowTrajectory::set_velocity(StateMachine& state_machine, std::shared_ptr<ros_msgs::Twist2D> vel_vector_msg)
{
    ros_msgs_lw::Twist2D vel_vector(*vel_vector_msg);

    if (vel_vector.v == 0 && vel_vector.w == 0)
    {
        state_machine.controller_master.stop_controller();

        state_machine.setState(new Idle);
    } 
}

void FollowTrajectory::set_trajectory(StateMachine& state_machine, std::shared_ptr<ros_msgs::Trajectory> trajectory)
{
    //state_machine.controller_master.start_controller(new approxLinController(trajectory), std::bind(&StateMachine::stop, &state_machine));
    //state_machine.controller_master.start_controller(new statInOutLinController(trajectory), std::bind(&StateMachine::stop, &state_machine));
    state_machine.controller_master.start_controller(new dynInOutLinController(trajectory), std::bind(&StateMachine::stop, &state_machine));

    state_machine.setState(new FollowTrajectory);
}

void FollowTrajectory::stop(StateMachine& state_machine)
{
    state_machine.controller_master.stop_controller();
    state_machine.setState(new Idle);
}