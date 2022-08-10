#include "ControllerMaster.h"

#include "esp_log.h"

#define TAG "ControllerMaster"


ControllerMaster* ControllerMaster::_controller_obj = nullptr;

ControllerMaster::ControllerMaster(OutputVelocity& output_velocity, SensorPose& sensor_pose) : _pos_controller{nullptr}, _output_velocity{output_velocity}, _sensor_pose{sensor_pose}
{   
    _pos_controller_mutx = xSemaphoreCreateMutex();

    xTaskCreate(_control_loop_task, "control_loop_task", 8096, this, 8, &_control_loop_task_handle);
    _control_loop_timer_handle = xTimerCreate("control_loop", pdMS_TO_TICKS(10), pdTRUE, NULL, _control_loop_timer);

    _controller_is_stopped = true;
    xTimerStop(_control_loop_timer_handle, portMAX_DELAY);
}

ControllerMaster::~ControllerMaster() 
{
    xTimerDelete(_control_loop_timer_handle, portMAX_DELAY);
    vTaskDelete(_control_loop_task_handle);
    vSemaphoreDelete(_pos_controller_mutx);

    if(_pos_controller != nullptr)
        delete _pos_controller;
    
    _pos_controller = nullptr; 
}

ControllerMaster& ControllerMaster::init(OutputVelocity& output_velocity, SensorPose& sensor_pose)
{
    if(_controller_obj == nullptr)
        _controller_obj = new ControllerMaster(output_velocity, sensor_pose);

    return *_controller_obj;
}

void ControllerMaster::start_controller(PositionController* pos_controller, std::function<void()>callback_function)
{   
    if(xSemaphoreTake(_pos_controller_mutx, portMAX_DELAY) == pdPASS)
    {
        if(_controller_obj->_pos_controller != nullptr)
            delete _controller_obj->_pos_controller;

        _controller_obj->_pos_controller = pos_controller;

        _controller_obj->_destination_reached_callback = callback_function;

        _sensor_pose.reInit();

        _controller_is_stopped = false;
        xTimerStart(_control_loop_timer_handle, portMAX_DELAY);

        xSemaphoreGive(_pos_controller_mutx);
    }
}

void ControllerMaster::_control_loop_timer(TimerHandle_t timer)
{   
    xTaskNotifyGive(_controller_obj->_control_loop_task_handle);
}

void ControllerMaster::stop_controller()
{
    _controller_is_stopped = true;
}

void ControllerMaster::_control_loop_task(void* pvParameters)
{
    ControllerMaster& controller_obj = *(reinterpret_cast<ControllerMaster*>(pvParameters));

    while(1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if(controller_obj._controller_is_stopped == true)
        {
            if(xTimerStop(controller_obj._control_loop_timer_handle, portMAX_DELAY) == pdPASS)
            {
                controller_obj._output_velocity.setVelocity(ros_msgs_lw::Twist2D(0, 0));

                ESP_LOGI(TAG, "Deleting Position Controller object.");
                delete controller_obj._pos_controller;
                controller_obj._pos_controller = nullptr;
            }
        }
        else if(xSemaphoreTake(controller_obj._pos_controller_mutx, 0) == pdPASS)
        {
            //Input
            ros_msgs_lw::Pose2D actual_pose;
            if(controller_obj._sensor_pose.getPose(actual_pose) == true)
            {

                uint64_t time = esp_timer_get_time();
                controller_obj.prev_time = time;

                //process position controller
                ros_msgs_lw::Twist2D output_vel = controller_obj._pos_controller->update(actual_pose);

                //Output
                controller_obj._output_velocity.setVelocity(output_vel);

                if(controller_obj._pos_controller->destination_reached() == true)
                {   
                    controller_obj._controller_is_stopped = true;
                    controller_obj._destination_reached_callback();
                }
            }
                
            xSemaphoreGive(controller_obj._pos_controller_mutx);
        }
    }
}