#pragma once

#include <cstring>
#include <cmath>
#include <stdint.h>
#include <memory>

#include "driver/rmt.h"
#include "esp_attr.h"
#include <sys/cdefs.h>
#include "sdkconfig.h"
#include "freertos/message_buffer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "RosMsgs.h"
#include "Subscriber.h"



struct led_strip_config_t{
    uint32_t max_leds;          /* Maximum LEDs in a single strip */
    rmt_channel_t dev;          /* LED strip device (e.g. RMT channel, PWM channel, etc) */  
} ;

struct LedStrip {

    uint32_t max_leds;
    rmt_channel_t rmt_channel;
    uint8_t *buffer;

    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;
    uint16_t hue = 0;
    uint16_t value = 0;

    int circle_inner[3] = {0, 1, 2};
    int circle_middle_1[8] {3, 4, 5, 6, 7, 8, 9, 10};
    int circle_middle_2[12]{11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
    int circle_outer[20] {23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42};

    bool max_value = false;

    int current_index = 0;
    int counter_circle = 0;

    void animation_callback(std::shared_ptr<ros_msgs::String> msg);

    // Animations
    void animation_rainbow();
    void animation_line();
    void animation_pulse();
    void animation_circle();
    void animation_specific();

    static void led_handler(void *arg);

    static TaskHandle_t led_handler_thread;
    MessageBufferHandle_t xMessageBuffer;
    const size_t xMessageBufferSizeBytes = 100;
    const TickType_t x0ms = pdMS_TO_TICKS( 0 );

    /**
    * @brief Set RGB for a specific pixel
    *
    * @param index: index of pixel to set
    * @param red: red part of color
    * @param green: green part of color
    * @param blue: blue part of color
    *
    * @return
    *      - ESP_OK: Set RGB for a specific pixel successfully
    *      - ESP_ERR_INVALID_ARG: Set RGB for a specific pixel failed because of invalid parameters
    *      - ESP_FAIL: Set RGB for a specific pixel failed because other error occurred
    */
    esp_err_t set_pixel(uint32_t index, uint32_t red, uint32_t green, uint32_t blue);       
   
    /**
    * @brief Refresh memory colors to LEDs
    *
    * @param timeout_ms: timeout value for refreshing task
    *
    * @return
    *      - ESP_OK: Refresh successfully
    *      - ESP_ERR_TIMEOUT: Refresh failed because of timeout
    *      - ESP_FAIL: Refresh failed because some other error occurred
    *
    * @note:
    *      After updating the LED colors in the memory, a following invocation of this API is needed to flush colors to strip.
    */
    esp_err_t refresh(uint32_t timeout_ms);                                                 


    /**
    * @brief Clear LED strip (turn off all LEDs)
    *
    * @param timeout_ms: timeout value for clearing task
    *
    * @return
    *      - ESP_OK: Clear LEDs successfully
    *      - ESP_ERR_TIMEOUT: Clear LEDs failed because of timeout
    *      - ESP_FAIL: Clear LEDs failed because some other error occurred
    */
    esp_err_t clear(uint32_t timeout_ms); 

    /**
    * @brief Install a new ws2815 driver 
    *
    * @param config: LED strip configuration
    * @return
    *      LED strip instance or NULL
    */
    esp_err_t led_strip_new_rmt(const led_strip_config_t *config);

    /**
    * @brief Convert hsv values to rgb values
    *
    * @param h: hue value
    * @param s: saturation
    * @param v: value
    * 
    * @param r: pointer of the color red
    * @param g: pointer of the color green
    * @param b: pointer of the color blue
    * @return
    *      void
    *
    */
    void hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b);

    /**
    * @brief sets the led strip up ready for animations
    *
    */

    void rgb2hue();
    static LedStrip& init();

    private:
        LedStrip();
        ~LedStrip();

        LedStrip(LedStrip const&) = delete;

        static LedStrip* _led_strip;
};






