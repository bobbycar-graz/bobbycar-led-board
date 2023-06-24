#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <tickchrono.h>

extern "C" void app_main()
{
    ESP_LOGI("main", "Hello world!");

    while (true)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
