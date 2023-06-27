constexpr const char * const TAG = "BOBBYCAR-LED-BOARD";

// esp-idf includes
#include <esp_log.h>
#include <tickchrono.h>

// 3rdparty lib includes
#include <espchrono.h>

// local includes
#include "taskmanager.h"
#include "measurements.h"

using namespace std::chrono_literals;

extern "C" [[noreturn]] void app_main()
{
    for (auto &task : schedulerTasks)
    {
        ESP_LOGI(TAG, "Init task %s", task.name());
        task.setup();
    }

    while (true)
    {
        for (auto &task : schedulerTasks)
        {
            task.loop();
        }

        const auto now = espchrono::millis_clock::now();
        static auto last = now;

        if (espchrono::ago(last) > 100ms)
        {
            last = now;
            ESP_LOGI(TAG, "V5V_RAW: %i, V5V_VALUE: %f - VBAT_RAW: %i, VBAT_VALUE: %f - I_RAW: %i, I_VALUE: %f, Temperature: %f, Humidity: %f",
                     measure::measure_v5v.raw,
                     measure::measure_v5v.value,
                     measure::measure_vbat.raw,
                     measure::measure_vbat.value,
                     measure::measure_current.raw,
                     measure::measure_current.value,
                    measure::measure_sht3x.temperature,
                    measure::measure_sht3x.humidity
            );
        }

        vTaskDelay(1);
    }
}
