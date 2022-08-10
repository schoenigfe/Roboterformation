#include "DataLogger.h"

#include "esp_log.h"

#define DATA_LOG_SAMPLE_SIZE CONFIG_DATA_LOG_SAMPLE_SIZE
#define DATA_LOG_BUFFER_SIZE CONFIG_DATA_LOG_BUFFER_SIZE
#define DATA_LOG_QUEUE_SIZE CONFIG_DATA_LOG_QUEUE_SIZE
#define MAX_LOG_TIME_SECONDS CONFIG_MAX_LOG_TIME_SECONDS

#define TAG "DataLogger"

DataLogger* DataLogger::_data_logger = nullptr;


DataLogger::DataLogger(ros::Publisher<ros_msgs::String>& publisher) : _publisher{publisher} 
{
    _data_logger_task_handle = nullptr;

    _log_buffer_cntr = 0;

    _log_buffer_queue = xQueueCreate(DATA_LOG_QUEUE_SIZE, sizeof(char*));
    _data_logging_semphr = xSemaphoreCreateBinary();
}

DataLogger::~DataLogger() 
{
    vSemaphoreDelete(_data_logging_semphr);
    vQueueDelete(_log_buffer_queue);
}

DataLogger& DataLogger::init(ros::Publisher<ros_msgs::String>& publisher)
{
    if(_data_logger == nullptr)
        _data_logger = new DataLogger(publisher);

    return *_data_logger;
}

void DataLogger::startLogging(std::shared_ptr<ros_msgs::String> log_time)
{
    int data_logging_time_ms  = std::stof((std::string)*log_time, nullptr) * 1000;

    if(data_logging_time_ms > 0 && data_logging_time_ms <= MAX_LOG_TIME_SECONDS * 1000)
    {
        if(_data_logger_task_handle == nullptr)
        {
            ESP_LOGI(TAG, "Start logging for %.2f seconds!", data_logging_time_ms / 1000.);

            _data_logging_end_us = esp_timer_get_time() + data_logging_time_ms * 1000;

            _log_buffer = new char[DATA_LOG_BUFFER_SIZE];
            _log_buffer_cntr = 0;

            xSemaphoreGive(_data_logging_semphr);
            xTaskCreate(_data_logger_task, "_data_logger_task", 2048, this, 3, &_data_logger_task_handle);
        }
    } 
}

void DataLogger::logData(const char* format, ...)
{                                    
    va_list arg;

    if(xSemaphoreTake(_data_logging_semphr, 0) == pdPASS)
    {
        if(_log_buffer_cntr + DATA_LOG_SAMPLE_SIZE >= DATA_LOG_BUFFER_SIZE)
        {   
            if(xQueueSend(_log_buffer_queue, &_log_buffer, 0) == pdPASS)
            {
                _log_buffer = new char[DATA_LOG_BUFFER_SIZE];  
                _log_buffer_cntr = 0;
            }
        }

        if(_log_buffer_cntr + DATA_LOG_SAMPLE_SIZE < DATA_LOG_BUFFER_SIZE)
        {
            va_start(arg, format);
            int len = vsnprintf(_log_buffer + _log_buffer_cntr, DATA_LOG_SAMPLE_SIZE, format, arg);  
            va_end(arg);

            if(len  != -1)
                _log_buffer_cntr += len;
        }

        xSemaphoreGive(_data_logging_semphr);   
    }
}

void DataLogger::_data_logger_task(void *pvParameters)
{
    DataLogger& data_logger = *reinterpret_cast<DataLogger*>(pvParameters);

    char* log_buffer;

    while(esp_timer_get_time() < data_logger._data_logging_end_us)
    {
        if(xQueueReceive(data_logger._log_buffer_queue, &log_buffer, 1000 / portTICK_PERIOD_MS) == pdPASS)
        {
            ros_msgs::String msg;
            msg.data.assign(log_buffer);

            data_logger._publisher.publish(msg);

            delete[] log_buffer;
        }
    }

    //stop logging
    xSemaphoreTake(data_logger._data_logging_semphr, portMAX_DELAY);

    //empty queue
    while(xQueueReceive(data_logger._log_buffer_queue, &log_buffer, 0) == pdPASS)
    {
        ros_msgs::String msg;
        msg.data.assign(log_buffer);

        data_logger._publisher.publish(msg);

        delete[] log_buffer;
    }

    ESP_LOGI(TAG, "End logging!");

    delete[] data_logger._log_buffer;

    data_logger._data_logger_task_handle = nullptr;
    vTaskDelete(NULL);
}