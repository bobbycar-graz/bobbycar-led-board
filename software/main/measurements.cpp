constexpr const char * const TAG = "MEASUREMENTS";
#include "measurements.h"

// esp-idf includes
#include <esp_log.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <driver/i2c.h>

// 3rdparty lib includes
#include <espchrono.h>
#include <sht3x.h>

// local includes
#include "pins.h"

/*
constexpr pin_t BAT_VOLTAGE_SENSOR = 32;
constexpr uint8_t BAT_VOLTAGE_RATIO = 23; // 22k + 1k voltage divider

constexpr pin_t V5V_VOLTAGE_SENSOR = 33;
constexpr uint8_t V5V_VOLTAGE_RATIO = 2; // 1k + 1k voltage divider

constexpr pin_t CURRENT_SENSOR = 35;
constexpr uint8_t CURRENT_SENSOR_RATIO = 132; // mV/A
*/

using namespace std::chrono_literals;

namespace measure {

AdcMeasurementResult measure_v5v;
AdcMeasurementResult measure_vbat;
AdcMeasurementResult measure_current;
SHT3XMeasurementResult measure_sht3x;

namespace {

bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = nullptr;
    esp_err_t ret = ESP_FAIL;
    bool calibrated{false};

#if ADC_CALI_CURVE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ADC calibration failed: %s", esp_err_to_name(ret));
    }

    return calibrated;
}

// unit handle
adc_oneshot_unit_handle_t adc1_handle{nullptr};

// cali handles
adc_cali_handle_t adc1_cali_chan5_handle{nullptr}; // V5V
adc_cali_handle_t adc1_cali_chan4_handle{nullptr}; // VBAT
adc_cali_handle_t adc1_cali_chan7_handle{nullptr}; // I

// do calibration flags
bool do_calibration_chan5{false};
bool do_calibration_chan4{false};
bool do_calibration_chan7{false};

bool i2c_initialized{false};

} // namespace

void init()
{
    adc_oneshot_unit_init_cfg_t adc1_unit_config = {
        .unit_id = ADC_UNIT_1,
    };

    if (const auto res = adc_oneshot_new_unit(&adc1_unit_config, &adc1_handle); res != ESP_OK)
    {
        ESP_LOGE(TAG, "Error initializing ADC: %s", esp_err_to_name(res));
        return;
    }

    adc_oneshot_chan_cfg_t adc1_channel_config = {
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12,
    };

    // V5V - pin 33 => ADC1_CHANNEL_5
    if (const auto res = adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_5, &adc1_channel_config); res != ESP_OK)
    {
        ESP_LOGE(TAG, "Error configuring ADC channel 5: %s", esp_err_to_name(res));
        return;
    }

    // VBAT - pin 32 => ADC1_CHANNEL_4
    if (const auto res = adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &adc1_channel_config); res != ESP_OK)
    {
        ESP_LOGE(TAG, "Error configuring ADC channel 4: %s", esp_err_to_name(res));
        return;
    }

    // I - pin 35 => ADC1_CHANNEL_7
    if (const auto res = adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_7, &adc1_channel_config); res != ESP_OK)
    {
        ESP_LOGE(TAG, "Error configuring ADC channel 7: %s", esp_err_to_name(res));
        return;
    }

    do_calibration_chan5 = adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_5, ADC_ATTEN_DB_11, &adc1_cali_chan5_handle);
    do_calibration_chan4 = adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_4, ADC_ATTEN_DB_11, &adc1_cali_chan4_handle);
    do_calibration_chan7 = adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_7, ADC_ATTEN_DB_11, &adc1_cali_chan7_handle);

    ESP_LOGI(TAG, "ADC configured");

    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = 100000,
        },
    };

    if (const auto res = i2c_param_config(I2C_NUM_0, &i2c_config); res != ESP_OK)
    {
        ESP_LOGE(TAG, "Error configuring I2C: %s", esp_err_to_name(res));
        return;
    }

    if (const auto res = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0); res != ESP_OK)
    {
        ESP_LOGE(TAG, "Error installing I2C driver: %s", esp_err_to_name(res));
        return;
    }

    i2c_initialized = true;

    ESP_LOGI(TAG, "I2C configured");
}

void update()
{
    if (const auto res = adc_oneshot_read(adc1_handle, ADC_CHANNEL_5, &measure_v5v.raw); res != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading ADC channel 5: %s", esp_err_to_name(res));
        return;
    }
    else
    {
        if (do_calibration_chan5)
        {
            int voltage_mv = 0;
            adc_cali_raw_to_voltage(adc1_cali_chan5_handle, measure_v5v.raw, &voltage_mv);
            measure_v5v.value = static_cast<float>(voltage_mv) / 1000.0f * V5V_VOLTAGE_RATIO;
        }
        else
        {
            measure_v5v.value = static_cast<float>(measure_v5v.raw) * 3.3f / 4095.0f * V5V_VOLTAGE_RATIO;
        }
        measure_v5v.timestamp = espchrono::millis_clock::now();
        measure_v5v.valid = true;
    }

    if (const auto res = adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &measure_vbat.raw); res != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading ADC channel 4: %s", esp_err_to_name(res));
        return;
    }
    else
    {
        if (do_calibration_chan4)
        {
            int voltage_mv = 0;
            adc_cali_raw_to_voltage(adc1_cali_chan4_handle, measure_vbat.raw, &voltage_mv);
            measure_vbat.value = static_cast<float>(voltage_mv) / 1000.0f * BAT_VOLTAGE_RATIO;
        }
        else
        {
            measure_vbat.value = static_cast<float>(measure_vbat.raw) * 3.3f / 4095.0f * BAT_VOLTAGE_RATIO;
        }
        measure_vbat.timestamp = espchrono::millis_clock::now();
        measure_vbat.valid = true;
    }

    if (const auto res = adc_oneshot_read(adc1_handle, ADC_CHANNEL_7, &measure_current.raw); res != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading ADC channel 7: %s", esp_err_to_name(res));
        return;
    }
    else
    {
        measure_current.value = (static_cast<float>(measure_current.raw) - 1970.f) / CURRENT_SENSOR_RATIO;
        measure_current.timestamp = espchrono::millis_clock::now();
        measure_current.valid = true;
    }

    if (!i2c_initialized)
        return;

    sht3x_start_periodic_measurement();

    sht3x_sensors_values_t sht3x_values = {
        .temperature = 0.f,
        .humidity = 0.f,
    };

    // delay 30ms
    vTaskDelay(pdMS_TO_TICKS(30));

    if (const auto res = sht3x_read_measurement(&sht3x_values); res != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading SHT3x values: %s", esp_err_to_name(res));
        return;
    }
    else
    {
        measure_sht3x.temperature = sht3x_values.temperature;
        measure_sht3x.humidity = sht3x_values.humidity;
        measure_sht3x.timestamp = espchrono::millis_clock::now();
        measure_sht3x.valid = true;
    }

    sht3x_stop_periodic_measurement();
}

} // namespace measure
