#include "Imu.h"

#include "esp_log.h"

//static const char *TAG = "i2c-simple-example";

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          1000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define BMX160_SENSOR_ADDR                 0x68        /*!< Slave address of the BMX160 sensor */

#define TAG "Imu_BMX160"

const i2c_config_t Imu::_i2c_conf = 
{
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    //.master.clk_speed = I2C_MASTER_FREQ_HZ,
    .clk_flags = 0,
};

Imu::Imu()
{
    ESP_ERROR_CHECK(i2c_master_init());
}
/**
 * @brief Read a sequence of bytes from a BMX160 sensor registers
 
static esp_err_t Imu::BMX160_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, BMX160_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}
*/

/**
 * @brief i2c master initialization
 */
esp_err_t Imu::i2c_master_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_param_config(i2c_master_port, &_i2c_conf);

    return i2c_driver_install(i2c_master_port, _i2c_conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


void Imu::test()
{
    uint8_t data[2];
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
}
