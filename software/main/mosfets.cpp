constexpr const char * const TAG = "MOSFETS";
#include "mosfets.h"

// esp-idf includes
#include <driver/ledc.h>
#include <esp_log.h>

// local includes
#include "pins.h"

namespace {
    bool initialized;
} // namespace

namespace mosfets {

void init()
{
    ledc_timer_config_t ledc_timer{
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    if (const auto err = ledc_timer_config(&ledc_timer); err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(err));
        return;
    }

    ledc_channel_config_t ledc_channel[2] = {
        {
            .gpio_num = MOSFET_1,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .channel = LEDC_CHANNEL_0,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        },
        {

        }
    };

    initialized = true;
}

void update()
{
    if (!initialized)
        return;
}

} // namespace measure
