// ====== LED FUNCTIONS ======

#include <FastLED.h>
#define NUM_LEDS 2            // Number of LEDs on your board
#define PIN_RBGLED 4          // LED Pin
CRGB leds[NUM_LEDS];          // Current LED Color values

void ledOn(CRGB color) {
  leds[0] = color;
  FastLED.show();
}

void ledOff() {
  leds[0] = CRGB::Black;
  FastLED.show();
}