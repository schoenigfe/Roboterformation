#include "Marvelmind.h"
#include "esp_log.h"

#define UART_TX_PIN CONFIG_UART_TX_PIN
#define UART_RX_PIN CONFIG_UART_RX_PIN
#define UART_BAUDRATE CONFIG_UART_BAUDRATE
#define UART_RX_BUFFER CONFIG_UART_RX_BUFFER

SensorValue<ros_msgs_lw::Pose2D>* Marvelmind::pose = new SensorValue<ros_msgs_lw::Pose2D>;
SensorValue<ros_msgs_lw::PoseQual>* Marvelmind::poseQual = new SensorValue<ros_msgs_lw::PoseQual>;
SensorValue<ros_msgs_lw::Imu>* Marvelmind::imu = new SensorValue<ros_msgs_lw::Imu>;

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
    uint32_t timestamp_ms;    
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
void Marvelmind::_uart_read_data_task(void* pvParameters)
{
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
        switch(msg_header.packet_identifier)
        {                       
            case 0x0003:
            {   printf("\ncase imu 0x0003\n");
                Marvelmind_IMU_Data* msg_data = (Marvelmind_IMU_Data*) data_buffer;
                ros_msgs_lw::Imu current_imu;
				printf("IMU: \nax=%f, ay=%f, az=%f  //in mg\n",  
				 static_cast<float>(msg_data->accelerometer_x_axis), static_cast<float>(msg_data->accelerometer_y_axis), static_cast<float>(msg_data->accelerometer_z_axis));
				printf("     \nvx=%f, vy=%f, vz=%f  //in dsp\n", 
				 static_cast<float>(msg_data->gyroscope_x_axis),  static_cast<float>(msg_data->gyroscope_y_axis),  static_cast<float>(msg_data->gyroscope_z_axis));                
                current_imu.a_x = static_cast<float>(msg_data->accelerometer_x_axis);
                current_imu.a_y = static_cast<float>(msg_data->accelerometer_y_axis); 
                current_imu.v_theta = static_cast<float>(msg_data->gyroscope_z_axis); 
                imu->setTimestamp(msg_data->timestamp_ms);    
				imu->overwriteValue(current_imu);
                break;
            }                     
            case 0x0007:
            {   printf("\ncase quality 0x0007\n");
                Marvelmind_Quality_Data* msg_data = (Marvelmind_Quality_Data*) data_buffer;                
                ros_msgs_lw::PoseQual current_qual;                
                current_qual.q = static_cast<uint8_t>(msg_data->positioning_quality); 
				poseQual->overwriteValue(current_qual);
                break;
            }           
            case 0x0011:
            {   printf("\ncase ultrasonic 0x0011\n");
                Marvelmind_Rx_Data* msg_data = (Marvelmind_Rx_Data*) data_buffer;
                ros_msgs_lw::Pose2D current_pose;
                current_pose.x = (static_cast<float>(msg_data->x_coordinate_mm) - 16.739) / 1000.;
                current_pose.y = static_cast<float>(msg_data->y_coordinate_mm) / 1000.;
                current_pose.theta = static_cast<float>(msg_data->hedgehog_orientation_raw & 0xFFF) / 10. * 2 * M_PI / 360. + M_PI / 2.;
                current_pose.theta = atan2(sin(current_pose.theta), cos(current_pose.theta));                              
				poseQual->setTimestamp(msg_data->timestamp_ms);
				pose->overwriteValue(current_pose);
                break;
            }                    
            default:                
				break;
        }
    }
    ESP_ERROR_CHECK(status);
    vTaskDelete(NULL);
}
Marvelmind::Marvelmind() 
{
    ESP_ERROR_CHECK(uart_driver_install(_uart_port, UART_RX_BUFFER, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(_uart_port, &_uart_conf));
    ESP_ERROR_CHECK(uart_set_pin(_uart_port, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));	
    xTaskCreate(_uart_read_data_task, "_uart_read_data_task", 2048, this, 5, NULL);
}
Marvelmind::~Marvelmind() 
{
    vTaskDelete(_uart_read_data_task_handle);
	delete pose;
	delete poseQual;
	delete imu;
    pose = nullptr;
    poseQual = nullptr;
    imu = nullptr;

}

