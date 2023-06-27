#include "ledstrip.h"

// system includes
#include <array>

// 3rdparty lib includes
#include <FastLED.h>

// local includes
#include "pins.h"

namespace ledstrip {

constexpr auto LED_COUNT = 30;

std::array<CRGB, LED_COUNT> leds;

void init()
{
    FastLED.addLeds<NEOPIXEL, WS2812B_DATA_PIN>(leds.data(), leds.size()).setCorrection(TypicalSMD5050);
    FastLED.setBrightness(255);
    FastLED.show();
}

void update()
{
    // rainbow
    fill_rainbow(leds.data(), leds.size(), 0, 7);
}

} // namespace ledstrip
