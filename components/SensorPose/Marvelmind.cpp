#include "Marvelmind.h"

#include "esp_log.h"


#define UART_TX_PIN CONFIG_UART_TX_PIN
#define UART_RX_PIN CONFIG_UART_RX_PIN
#define UART_BAUDRATE CONFIG_UART_BAUDRATE
#define UART_RX_BUFFER CONFIG_UART_RX_BUFFER

#define TAG "Marvelmind_Pose"


//Marvelmind UART communication protcol
//see: https://marvelmind.com/pics/marvelmind_interfaces.pdf

struct __attribute__((packed, aligned(1))) Marvelmind_Msg_Header  
{
    uint8_t destination_addr;
    uint8_t packet_type;
    uint16_t packet_identifier;
    uint8_t packet_size;
};

struct __attribute__((packed, aligned(1)))  Marvelmind_Rx_Data 
{
    uint32_t timestamp_ms;
    int32_t x_coordinate_mm;
    int32_t y_coordinate_mm;
    int32_t z_coordinate_mm;
    uint8_t flags;
    uint8_t hedgehog_addr;
    uint16_t hedgehog_orientation_raw;
    uint16_t time_delay;
    uint16_t crc; 
};

struct __attribute__((packed, aligned(1)))  Marvelmind_IMU_Data 
{
    int16_t accelerometer_x_axis;
    int16_t accelerometer_y_axis;
    int16_t accelerometer_z_axis;
    
    int16_t gyroscope_x_axis;
    int16_t gyroscope_y_axis;
    int16_t gyroscope_z_axis;
    
    int16_t compass_x_axis;
    int16_t compass_y_axis;
    int16_t compass_z_axis;
    
    uint8_t beacon_addr;
    
    uint8_t reserved_byte_1;
    uint8_t reserved_byte_2;
    uint8_t reserved_byte_3;
    uint8_t reserved_byte_4;
    uint8_t reserved_byte_5;
    
    uint32_t timestamp;
    
    uint8_t flags;
    
    uint8_t reserved_byte_a;
    uint8_t reserved_byte_b;
    uint8_t reserved_byte_c;
    
};

struct __attribute__((packed, aligned(1)))  Marvelmind_Quality_Data 
{
    uint8_t device_adress;
    uint8_t positioning_quality;
    uint8_t geofencing_zone_alarm;
    
};

Marvelmind* Marvelmind::_marvelmind_sensor = nullptr;

const uart_port_t Marvelmind::_uart_port{UART_NUM_2};
const uart_config_t Marvelmind::_uart_conf = 
{
    .baud_rate = UART_BAUDRATE,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 0,
    .source_clk = UART_SCLK_APB     
};

Marvelmind::Marvelmind() : _measurement_noise_cov(3, 3)
{   
    _measurement_noise_cov(0, 0) = 0.000002; 
    _measurement_noise_cov(0, 1) = -0.000001;
    _measurement_noise_cov(0, 2) = 0.000013;
    _measurement_noise_cov(1, 0) = -0.000001;
    _measurement_noise_cov(1, 1) = 0.000007; 
    _measurement_noise_cov(1, 2) = -0.000021;
    _measurement_noise_cov(2, 0) = 0.000013;
    _measurement_noise_cov(2, 1) = -0.000021;
    _measurement_noise_cov(2, 2) = 0.000190;

    ESP_ERROR_CHECK(uart_driver_install(_uart_port, UART_RX_BUFFER, 0, 0, NULL, 0));

    ESP_ERROR_CHECK(uart_param_config(_uart_port, &_uart_conf));

    ESP_ERROR_CHECK(uart_set_pin(_uart_port, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    _current_pose_queue = xQueueCreate(1, sizeof(ros_msgs_lw::Pose2D));
    _peek_at_pose_queue = xQueueCreate(1, sizeof(ros_msgs_lw::Pose2D));
    _current_imu_queue = xQueueCreate(1, sizeof(ros_msgs_lw::Imu));

    xTaskCreate(_uart_read_data_task, "_uart_read_data_task", 2048, this, 5, NULL);
}

Marvelmind::~Marvelmind()
{
    vTaskDelete(_uart_read_data_task_handle);
    vQueueDelete(_current_pose_queue);
    vQueueDelete(_peek_at_pose_queue);
    vQueueDelete(_current_imu_queue);
}

Marvelmind& Marvelmind::init()
{
    if(_marvelmind_sensor == nullptr)
        _marvelmind_sensor = new Marvelmind;

    return *_marvelmind_sensor;
}

bool Marvelmind::peekAtPose(ros_msgs_lw::Pose2D& current_pose) const
{
    if(xQueuePeek(_peek_at_pose_queue, &current_pose, 0) == pdPASS)
        return true;

    return false;
}


bool Marvelmind::getIMU(ros_msgs_lw::Imu& current_imu) const
{
    if(xQueueReceive(_current_imu_queue, &current_imu, 0) == pdPASS)
        return true;

    return false;
}

bool Marvelmind::getPose(ros_msgs_lw::Pose2D& current_pose) const
{
    if(xQueueReceive(_current_pose_queue, &current_pose, 0) == pdPASS)
        return true;

    return false;
}

void Marvelmind::calculateMeasurementNoiseCov() const
{
    ros_msgs_lw::Pose2D* pose_measurements = new ros_msgs_lw::Pose2D[KALMAN_SENSOR_COVARIANCE_CALCULATION_SAMPLES];

    ros_msgs_lw::Pose2D mean;
    
    for(int i = 0; i < KALMAN_SENSOR_COVARIANCE_CALCULATION_SAMPLES; i++)
    {
        xQueueReceive(_current_pose_queue, &pose_measurements[i], portMAX_DELAY);

        mean = mean + 1. / KALMAN_SENSOR_COVARIANCE_CALCULATION_SAMPLES * pose_measurements[i];
    }
    
    dspm::Mat measurement_noise_cov(3, 3);

    for(int i = 0; i < KALMAN_SENSOR_COVARIANCE_CALCULATION_SAMPLES; i++)
    {
        dspm::Mat deviation(3, 1);
        deviation(0, 0) = pose_measurements[i].x - mean.x;
        deviation(1, 0) = pose_measurements[i].y - mean.y;
        deviation(2, 0) = pose_measurements[i].theta - mean.theta;

        deviation(2, 0) = atan2(sin(deviation(2, 0)), cos(deviation(2, 0)));

        measurement_noise_cov += deviation * deviation.t(); 
    }


    printf("Measurement Noise Cov: \n\n");

    for(int i = 0; i < 3; i++)
    {
        printf("|");

        for(int k = 0; k < 3; k++)
            printf("  %f", measurement_noise_cov(i, k));

        printf("  |\n");        
    }

    delete[] pose_measurements;

    vTaskDelay(10000 / portTICK_PERIOD_MS);

    esp_restart();
}

bool Marvelmind::calculateKalman(ros_msgs_lw::Pose2D const& a_priori_estimate, dspm::Mat const& a_priori_cov, ros_msgs_lw::Pose2D& a_posterior_estimate, dspm::Mat& a_posterior_cov) const
{
    ros_msgs_lw::Pose2D current_pose;

    if(xQueueReceive(_current_pose_queue, &current_pose, 0) == pdPASS)
    {
        float kalman_data[9];
        dspm::Mat kalman_gain(kalman_data, 3, 3);

        kalman_gain = a_priori_cov * (a_priori_cov + _measurement_noise_cov).inverse();
        
        a_posterior_estimate = a_priori_estimate + kalman_gain * (current_pose - a_priori_estimate);

        float identity_data[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        dspm::Mat identity(identity_data, 3, 3);

        a_posterior_cov = (identity - kalman_gain) * a_priori_cov;

        return true;
    }

    return false;
}

bool Marvelmind::getAbsolutePose(ros_msgs_lw::Pose2D& initial_pose) const
{
    if(xQueueReceive(_current_pose_queue, &initial_pose, 0) == pdPASS)
        return true;

    return false;
}

void Marvelmind::getMeasurementNoiseCov(dspm::Mat& measurement_cov) const
{
    measurement_cov = _measurement_noise_cov;
}

void Marvelmind::_uart_read_data_task(void* pvParameters)
{
    Marvelmind* marvelmind = (Marvelmind*)pvParameters;
    int status = ESP_OK;

    while(1)
    {

        Marvelmind_Msg_Header msg_header;

        int len = uart_read_bytes(_uart_port, &msg_header, sizeof(Marvelmind_Msg_Header), portMAX_DELAY);

        if(len == -1)
        {
            status = ESP_FAIL;
            break;
        }

        uint8_t data_buffer[msg_header.packet_size];

        len = uart_read_bytes(_uart_port, &data_buffer, msg_header.packet_size + 2, portMAX_DELAY);

        if(len == -1)
        {
            status = ESP_FAIL;
            break;
        }
        
        //printf("\nMessageheaderPacketIdentifier: %04x\n",msg_header.packet_identifier);
        switch(msg_header.packet_identifier)
        {   
            
            case 0x0011:
            {   printf("\ncase ultrasonic 0x0011\n");
                Marvelmind_Rx_Data* msg_data = (Marvelmind_Rx_Data*) data_buffer;

                ros_msgs_lw::Pose2D current_pose;

                current_pose.x = (static_cast<float>(msg_data->x_coordinate_mm) - 16.739) / 1000.;
                current_pose.y = static_cast<float>(msg_data->y_coordinate_mm) / 1000.;
                current_pose.theta = static_cast<float>(msg_data->hedgehog_orientation_raw & 0xFFF) / 10. * 2 * M_PI / 360. + M_PI / 2.;

                current_pose.theta = atan2(sin(current_pose.theta), cos(current_pose.theta));

                xQueueOverwrite(marvelmind->_current_pose_queue, &current_pose);
                xQueueOverwrite(marvelmind->_peek_at_pose_queue, &current_pose);

                break;
            }
            
            
            
            case 0x0003:
            {   printf("\ncase imu raw 0x0003\n");
                Marvelmind_IMU_Data* msg_data = (Marvelmind_IMU_Data*) data_buffer;
                ros_msgs_lw::Imu current_imu;
                
                current_imu.timestamp = msg_data->timestamp;
                current_imu.quaternion_orientation_x = 0;
                current_imu.quaternion_orientation_y = 0;
                current_imu.quaternion_orientation_z = 0;
                current_imu.quaternion_orientation_w = 0;
                current_imu.angular_velocity_x = static_cast<float>(msg_data->gyroscope_x_axis) / 1000.;
                current_imu.angular_velocity_y = static_cast<float>(msg_data->gyroscope_y_axis) / 1000.;
                current_imu.angular_velocity_z = static_cast<float>(msg_data->gyroscope_z_axis) / 1000.;
                current_imu.linear_acceleration_x = static_cast<float>(msg_data->accelerometer_x_axis) / 1000.;
                current_imu.linear_acceleration_y = static_cast<float>(msg_data->accelerometer_y_axis) / 1000.;
                current_imu.linear_acceleration_z = static_cast<float>(msg_data->accelerometer_z_axis) / 1000.;
                
                xQueueOverwrite(marvelmind->_current_imu_queue, &current_imu);
                //xQueueOverwrite(marvelmind->_peek_at_pose_queue, &current_pose);

                break;
            }
            
            
            
            case 0x0007:
            {   printf("\ncase imu 0x0007\n");
                Marvelmind_Quality_Data* msg_data = (Marvelmind_Quality_Data*) data_buffer;
                uint8_t positionquality;
                
                printf("positionquality:%d", msg_data->positioning_quality);

                positionquality = static_cast<uint8_t>(msg_data->positioning_quality);
                               
                //xQueueOverwrite(marvelmind->_current_pose_queue, &current_pose);
                //xQueueOverwrite(marvelmind->_peek_at_pose_queue, &current_pose);

                break;
            }
            //*/
            
            
            
            default:
                
                break;
        }

    }

    ESP_ERROR_CHECK(status);

    vTaskDelete(NULL);
}
