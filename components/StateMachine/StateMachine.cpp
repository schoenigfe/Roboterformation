#include "StateMachine.h"
#include "States.h"

#include "esp_log.h"

#define TAG "StateMachine"

StateMachine* StateMachine::_state_machine_obj = nullptr;

StateMachine::StateMachine(ControllerMaster& controller_master, OutputVelocity& output_velocity) : 
    controller_master{controller_master}, output_velocity{output_velocity}, _current_state{new Idle} {}

StateMachine& StateMachine::init(ControllerMaster& controller_master, OutputVelocity& output_velocity)
{
    if(_state_machine_obj == nullptr)
        _state_machine_obj = new StateMachine(controller_master, output_velocity);
    
    return *_state_machine_obj;
}

void StateMachine::setState(State* new_state)
{
    ESP_LOGI(TAG, "%s -> %s", _current_state->getState().c_str(), new_state->getState().c_str());

    delete _current_state;
    _current_state = new_state;
}

void StateMachine::set_velocity(std::shared_ptr<ros_msgs::Twist2D> vel_vector)
{
    _current_state->set_velocity(*this, vel_vector);
}

void StateMachine::set_goal_point(std::shared_ptr<ros_msgs::Point2D> goal_point)
{
    _current_state->set_goal_point(*this, goal_point);
}

void StateMachine::set_trajectory(std::shared_ptr<ros_msgs::Trajectory> trajectory)
{
    _current_state->set_trajectory(*this, trajectory);
}

void StateMachine::stop()
{
    _current_state->stop(*this);
}