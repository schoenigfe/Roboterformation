#include "MotorController.h"

//#define DATA_LOGGING
#include "DataLogger.h"

#define ENABLE_PIN CONFIG_ENABLE_PIN

#define TIMER_DIVIDER 16
#define TIMER_TICKS_PER_US (TIMER_BASE_CLK / TIMER_DIVIDER / 1000000)
#define TIMER_PERIOD_US 1000

#define MAX_MOTOR_RPS (220. / 60.)

MotorController* MotorController::_motor_controller = nullptr;
timer_config_t MotorController::_timer_config = 
{
    .alarm_en = TIMER_ALARM_EN,
    .counter_en = TIMER_PAUSE,
    .intr_type = TIMER_INTR_LEVEL,
    .counter_dir = TIMER_COUNT_UP,
    .auto_reload = TIMER_AUTORELOAD_EN,
    //.clk_src = TIMER_SRC_CLK_APB,         //uncomment to remove warning in IDF v5.0
    .divider = TIMER_DIVIDER,
};
gpio_config_t MotorController::_enable_config = 
{
    .pin_bit_mask = BIT(ENABLE_PIN),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};

MotorController::MotorController(Motor& motor_a, Motor& motor_b) : _motor_a(motor_a), _motor_b(motor_b) 
{
    xTaskCreate(_motor_control_loop_task, "_motor_control_loop_task", 2048, this, 10, &_control_loop_task);

    ESP_ERROR_CHECK(timer_init(TIMER_GROUP_0, TIMER_0, &_timer_config));

    ESP_ERROR_CHECK(timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0));
    ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_PERIOD_US * TIMER_TICKS_PER_US));

    ESP_ERROR_CHECK(timer_enable_intr(TIMER_GROUP_0, TIMER_0));
    ESP_ERROR_CHECK(timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, _motor_control_interrupt, &_control_loop_task, ESP_INTR_FLAG_IRAM));

    ESP_ERROR_CHECK(timer_start(TIMER_GROUP_0, TIMER_0));

    _enable_pin = static_cast<gpio_num_t>(ENABLE_PIN);

    ESP_ERROR_CHECK(gpio_config(&_enable_config));
    gpio_set_level(_enable_pin, 1);
}

MotorController& MotorController::init(Motor& motor_a, Motor& motor_b)
{
    if(_motor_controller == nullptr)
        _motor_controller = new MotorController(motor_a, motor_b);

    return *_motor_controller;
}

void MotorController::enablePIcontrol()
{
    _enable_PI_control = true;
}

void MotorController::disablePIcontrol()
{
    _enable_PI_control = false;
}

void MotorController::setVelocity(float setpoint_velocity_a, float setpoint_velocity_b)
{
    if(abs(setpoint_velocity_a) < MAX_MOTOR_RPS && abs(setpoint_velocity_b) < MAX_MOTOR_RPS)
    {
        _motor_a.setSetpointVelocity(setpoint_velocity_a);
        _motor_b.setSetpointVelocity(setpoint_velocity_b);
    }
    
}

void MotorController::_motor_control_loop_task(void* pvParameters)
{
    MotorController& motor_controller = *(MotorController*)pvParameters;

    while(1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); 
        
        float output_duty_cycle_a = 0;
        float output_duty_cycle_b = 0;

        if(motor_controller._enable_PI_control)
        {
            float actual_velocity_a = motor_controller._motor_a.getActualVelocity();
            float actual_velocity_b = motor_controller._motor_b.getActualVelocity();
             
            output_duty_cycle_a = motor_controller._motor_a.updatePIControl(actual_velocity_a);
            output_duty_cycle_b = motor_controller._motor_b.updatePIControl(actual_velocity_b);       
        }
        else
        {
            output_duty_cycle_a = motor_controller._motor_a.getSetpointVelocity() / MAX_MOTOR_RPS * 100.;
            output_duty_cycle_b = motor_controller._motor_b.getSetpointVelocity() / MAX_MOTOR_RPS * 100.;
        }

        motor_controller._motor_a.setDuty(output_duty_cycle_a);
        motor_controller._motor_b.setDuty(output_duty_cycle_b);

        LOG_DATA("%.2f, %.2f, %lld\n", motor_controller._motor_a.getSetpointVelocity(), motor_controller._motor_a.getActualVelocity(), esp_timer_get_time())
    }

    vTaskDelete(NULL);
}

bool IRAM_ATTR MotorController::_motor_control_interrupt(void* args)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    TaskHandle_t* control_loop_task = (TaskHandle_t*)args;
 
    vTaskNotifyGiveFromISR(*control_loop_task, &xHigherPriorityTaskWoken);

    return xHigherPriorityTaskWoken;
}

