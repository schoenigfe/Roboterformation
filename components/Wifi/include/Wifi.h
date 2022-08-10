#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_err.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"

#include <cstring>

/**
 * @brief The Wifi class creates a wifi station and handles the wifi connection to an access point.
 * The _event_handler() function defines what to do in certain events.
 */
class Wifi {
    public:
        static Wifi& init();
        void begin();

    private:
        Wifi();
        Wifi(Wifi const&) = delete;
        ~Wifi() {}

        static void _event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

        static Wifi* _wifi;

        EventGroupHandle_t _wifi_event_group;
        static wifi_init_config_t _wifi_init_config;
        static wifi_config_t _wifi_config;
};