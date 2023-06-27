#pragma once

// system includes
#include <optional>

// 3rdparty lib includes
#include <espchrono.h>

namespace measure {

struct AdcMeasurementResult {
    espchrono::millis_clock::time_point timestamp;
    float value{};
    int raw{};
    bool valid{false};
};

struct SHT3XMeasurementResult {
    espchrono::millis_clock::time_point timestamp;
    float temperature{};
    float humidity{};
    bool valid{false};
};

extern AdcMeasurementResult measure_v5v;
extern AdcMeasurementResult measure_vbat;
extern AdcMeasurementResult measure_current;
extern SHT3XMeasurementResult measure_sht3x;

void init();

void update();

} // namespace measure
