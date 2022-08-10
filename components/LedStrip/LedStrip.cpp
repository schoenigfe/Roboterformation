#include "LedStrip.h"

#include "esp_log.h"

#define TAG "LedStrip"


#define WS2815_T0H_NS (350)
#define WS2815_T0L_NS (1000)
#define WS2815_T1H_NS (1000)
#define WS2815_T1L_NS (350)
#define WS2815_RESET_US (280)
#define LEDTAG "LedStrip"

static uint32_t ws2815_t0h_ticks = 0;
static uint32_t ws2815_t1h_ticks = 0;
static uint32_t ws2815_t0l_ticks = 0;
static uint32_t ws2815_t1l_ticks = 0;

#define RMT_TX_CHANNEL RMT_CHANNEL_0

LedStrip* LedStrip::_led_strip = nullptr;
TaskHandle_t LedStrip::led_handler_thread{};


static void IRAM_ATTR ws2815_rmt_adapter(const void *src, rmt_item32_t *dest, size_t src_size,
        size_t wanted_num, size_t *translated_size, size_t *item_num)
{
    if (src == NULL || dest == NULL) {
        *translated_size = 0;
        *item_num = 0;
        return;
    }
    const rmt_item32_t bit0 = {{{ ws2815_t0h_ticks, 1, ws2815_t0l_ticks, 0 }}}; //Logical 0
    const rmt_item32_t bit1 = {{{ ws2815_t1h_ticks, 1, ws2815_t1l_ticks, 0 }}}; //Logical 1
    size_t size = 0;
    size_t num = 0;
    uint8_t *psrc = (uint8_t *)src;
    rmt_item32_t *pdest = dest;
    while (size < src_size && num < wanted_num) {
        for (int i = 0; i < 8; i++) {
            // MSB first
            if (*psrc & (1 << (7 - i))) {
                pdest->val =  bit1.val;
            } else {
                pdest->val =  bit0.val;
            }
            num++;
            pdest++;
        }
        size++;
        psrc++;
    }
    *translated_size = size;
    *item_num = num;
}

esp_err_t LedStrip::set_pixel(uint32_t index, uint32_t red, uint32_t green, uint32_t blue)  
{   
    if(!(index < max_leds))
    {
        ESP_LOGE(TAG, "index out of the maximum number of leds! " );
        return ESP_FAIL;
    }
    uint32_t start = index * 3;
    // In the order of GRB
    buffer[start + 0] = green & 0xFF;
    buffer[start + 1] = red & 0xFF;
    buffer[start + 2] = blue & 0xFF;
    return ESP_OK;
}


esp_err_t LedStrip::refresh(uint32_t timeout_ms)
{
    if(!(rmt_write_sample(rmt_channel, buffer, max_leds * 3, true) == ESP_OK))
    {
        ESP_LOGE(TAG, "transmit RMT samples failed " );
        return ESP_FAIL;
    }
    return rmt_wait_tx_done(rmt_channel, pdMS_TO_TICKS(timeout_ms));

}

esp_err_t LedStrip::clear(uint32_t timeout_ms)
{
    memset(buffer, 0 , max_leds * 3);
    return refresh(timeout_ms);
}

LedStrip::LedStrip() 
{ 
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX((gpio_num_t)CONFIG_LED_RMT_TX_GPIO, RMT_TX_CHANNEL);
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    
    led_strip_config_t strip_config;
    strip_config.max_leds = CONFIG_LED_STRIP_LED_NUMBER;
    strip_config.dev = config.channel;

    ESP_ERROR_CHECK(led_strip_new_rmt(&strip_config));

    // Clear LED strip (turn off all LEDs)
    clear(0);

    xTaskCreate(led_handler, "led_handler", 4096, this, 2, &led_handler_thread);

    xMessageBuffer = xMessageBufferCreate(xMessageBufferSizeBytes);
}

LedStrip::~LedStrip()
{
    delete[] buffer;
    vTaskDelete(led_handler_thread);
}

void LedStrip::animation_callback(std::shared_ptr<ros_msgs::String> msg)
{
    const char *message = msg->data.c_str();

   xMessageBufferSend(xMessageBuffer, message, msg->data.size(), 0);

}


esp_err_t LedStrip::led_strip_new_rmt(const led_strip_config_t *config)
{   
    rmt_channel = config->dev;       
    max_leds = config->max_leds;


    if(!config)
    {
        ESP_LOGE(TAG, "configuration can't be null");
        return ESP_FAIL;
    }
    // 24 bits per leds TODO
    uint32_t buffer_size = max_leds * 3;
    buffer = new uint8_t[buffer_size];
    if(!buffer)
    {
        ESP_LOGE(TAG, "request memory for ws2812 failed");
        return ESP_FAIL;
    }

    uint32_t counter_clk_hz = 0;

    if(rmt_get_counter_clock(config->dev, &counter_clk_hz)) 
    {
        ESP_LOGE(TAG, "get rmt counter clock failed");
        return ESP_FAIL;
    } 
    

    //ns -> ticks
    float ratio = (float)counter_clk_hz / 1e9;

    ws2815_t0h_ticks = (uint32_t)(ratio * WS2815_T0H_NS);
    ws2815_t0l_ticks = (uint32_t)(ratio * WS2815_T0L_NS);
    ws2815_t1h_ticks = (uint32_t)(ratio * WS2815_T1H_NS);
    ws2815_t1l_ticks = (uint32_t)(ratio * WS2815_T1L_NS);  

    //set ws2815 to rmt adapter
    rmt_translator_init(config->dev, ws2815_rmt_adapter);   

    return ESP_OK;
}

LedStrip& LedStrip::init()
{
    if(_led_strip == nullptr)
        _led_strip = new LedStrip;

    return *_led_strip;
}

void LedStrip::hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}

void LedStrip::rgb2hue()
{
    int max_value;
    int min_value;
    float temp_hue;

    max_value = fmax(fmax(red, green), blue);
    min_value = fmin(fmin(red, green), blue);

    if(max_value == min_value)
        temp_hue =  0;

    if(max_value == red)
    {   
        temp_hue = ((float)green - (float)blue) / (float)(max_value - min_value);

    }else if(max_value == green)
    {
        temp_hue = 2 + ((float)blue - (float)red) / (float)(max_value - min_value);

    }else if(max_value == blue)
    {
        temp_hue = 4 + ((float)red - (float)green) / (float)(max_value - min_value);
    }

    temp_hue = temp_hue * 60;

    if(temp_hue < 0)    
        temp_hue += 360;

    hue = temp_hue;

}

void LedStrip::animation_rainbow()
{

    hsv2rgb(hue, 100, 50, &red, &green, &blue);

    for (int i = 0; i < max_leds ; i++)
    {
        set_pixel(i, red, green, blue);
    }     

    refresh(0);

    hue += 3;

}

void LedStrip::animation_line()
{
    memset(buffer, 0 , max_leds * 3);

    if(current_index == 0)
    {   
        set_pixel(current_index, red, green, blue);
    }
    else if(current_index == 1)
    {
        set_pixel(current_index - 1, red, green, blue);
        set_pixel(current_index, red, green, blue);
        
    }else if(current_index == 2)
    {
        set_pixel(current_index - 2, red, green, blue);
        set_pixel(current_index - 1, red, green, blue);
        set_pixel(current_index, red, green, blue);
        
    }else if(current_index >= 3 && current_index < max_leds)
    {   
        set_pixel(current_index - 3, 0, 0, 0);

        set_pixel(current_index - 2, red, green, blue);
        set_pixel(current_index - 1, red, green, blue);
        set_pixel(current_index, red, green, blue);
        
    }else if(current_index == max_leds)
    {
        set_pixel(max_leds - 3, 0, 0, 0);   

        set_pixel(max_leds - 2, red, green, blue);  
        set_pixel(max_leds - 1, red, green, blue);  
        
    }else if(current_index == max_leds + 1)
    {
        set_pixel(max_leds - 2, 0, 0, 0);           

        set_pixel(max_leds - 1, red, green, blue);  
        
    }else if(current_index == max_leds + 2)
    {
        set_pixel(max_leds - 1, 0, 0, 0);               
    
    }

    refresh(0);

    current_index++;

    if(current_index >= max_leds + 3)
        current_index = 0;

}

void LedStrip::animation_pulse()
{
    hsv2rgb(hue, 100, value, &red, &green, &blue);

    memset(buffer, 0 , max_leds * 3);

    for (int i = 0; i < max_leds; i++)
    {
        set_pixel(i, red, green, blue);
    }
    
    refresh(0);


    if(max_value == false)
    {
        value += 2;

        if(value > 70)
            max_value = true;
    }

    if(max_value == true)
    {
        value -= 2;

        if(value < 4)
            max_value = false;
    }
    
}

void LedStrip::animation_circle()
{
    memset(buffer, 0 , max_leds * 3);

    if(counter_circle == 0)
    {
        for (int i = 0; i < sizeof(circle_inner)/sizeof(int); i++)
        {
            set_pixel(circle_inner[i], red, green, blue);
        }
        refresh(0);
    
    }

    if(counter_circle == 1)
    {
        for (int i = 0; i < sizeof(circle_inner)/sizeof(int); i++)
        {
            set_pixel(circle_inner[i], 0, 0, 0);
        }

        for (int i = 0; i < sizeof(circle_middle_1)/sizeof(int); i++)
        {
            set_pixel(circle_middle_1[i], red, green, blue);
        }
        refresh(0);  
    }

    if(counter_circle == 2)
    {
        for (int i = 0; i < sizeof(circle_middle_1)/sizeof(int); i++)
        {
            set_pixel(circle_middle_1[i], 0, 0, 0);
        }

        for (int i = 0; i < sizeof(circle_middle_2)/sizeof(int); i++)
        {
            set_pixel(circle_middle_2[i], red, green, blue);
        }

        refresh(0);   
    }

    if(counter_circle == 3)
    {
        for (int i = 0; i < sizeof(circle_middle_2)/sizeof(int); i++)
        {
            set_pixel(circle_middle_2[i], 0, 0, 0);
        }

        for (int i = 0; i < sizeof(circle_outer)/sizeof(int); i++)
        {
            set_pixel(circle_outer[i], red, green, blue);
        }

        refresh(0);   
    }

    counter_circle++;

    if(counter_circle > 3)
        counter_circle = 0;

}

void LedStrip::animation_specific()
{
    for(int i = 0; i < max_leds; i++)
    {
        set_pixel(i, red, green, blue);
    }

    refresh(0);

}

void LedStrip::led_handler(void *arg)
{   
    LedStrip &ledStrip = *reinterpret_cast<LedStrip*>(arg);

    char received_message[ledStrip.xMessageBufferSizeBytes] = {0};
    char animation[32];
    char *red;
    char *green;
    char *blue;

    while(1)
    {
        size_t xReceivedBytes = 0;

        if((xReceivedBytes = xMessageBufferReceive( ledStrip.xMessageBuffer, received_message, ledStrip.xMessageBufferSizeBytes, 0)) > 0)
        {
            
            //Getting Animation and RGB values
            strcpy(animation, strtok(received_message, ","));
            red = strtok(NULL, ",");
            green = strtok(NULL, ",");
            blue = strtok(NULL, ",");

            ledStrip.red = strtoul(red, NULL, 0);
            ledStrip.green = strtoul(green, NULL, 0);
            ledStrip.blue = strtoul(blue, NULL, 0);

            ledStrip.rgb2hue();

            for(int i = 0; i < ledStrip.xMessageBufferSizeBytes; i++)
                received_message[i] = '\0';
        }

        if(animation != nullptr)
        {
            if(!strcmp(animation ,"rainbow"))
                ledStrip.animation_rainbow();
            else if(!strcmp(animation ,"pulse"))
                ledStrip.animation_pulse();
            else if(!strcmp(animation ,"line"))
                ledStrip.animation_line();
            else if(!strcmp(animation ,"circle"))
                ledStrip.animation_circle();
            else if(!strcmp(animation ,"specific"))
                ledStrip.animation_specific();
        }

        vTaskDelay(10);
    }
}