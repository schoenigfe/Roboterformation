#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"

//#include "RosMsgsLw.h"
//#include "SensorPose.h"

struct Imu 
{
    public: static esp_err_t i2c_master_init();
        void test();

    private:
        Imu();
        ~Imu();
		static esp_err_t BMX160_register_read(uint8_t reg_addr, uint8_t *data, size_t len);
        static const i2c_config_t _i2c_conf;
};
