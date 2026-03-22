#include <Arduino.h>
#include <FastLED.h>

// Number of LEDs in your strip
#define NUM_LEDS 27

// Data pin for your LED strip (adjust based on your wiring)
#define DATA_PIN 4

uint8_t g_animation_mode = 0; // Global variable for animation selection

CRGB leds[NUM_LEDS];

void rainbow();
void confetti();

// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();

SimplePatternList gPatterns = {rainbow, confetti};

void sketchSetup() {
  // Give yourself a moment to connect a serial monitor or re-flash
  delay(2000);

  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(60);
}

uint8_t hue = 0;

void sketchLoop() {

  // animation
  gPatterns[g_animation_mode]();

  hue++;
  FastLED.show();
}

void rainbow(){
  fill_rainbow(leds,NUM_LEDS,hue, 2);
}

void confetti() {
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds, NUM_LEDS, 12);
  int pos = random8(NUM_LEDS);
  leds[pos] += CHSV( hue + random8(64), 200, 255);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}