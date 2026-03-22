#include <Arduino.h>
#include <FastLED.h>
#include "esp_log.h"

#include "orchestra_shared_config.h"

// Number of LEDs in your strip
#define NUM_LEDS 27

// Data pin for your LED strip (adjust based on your wiring)
#define DATA_PIN 4

uint8_t g_animation_mode = ORCHESTRA_DEFAULT_ANIMATION_MODE; // Global variable for animation selection

CRGB leds[NUM_LEDS];

void rainbow();
void confetti();

// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();

SimplePatternList gPatterns = {rainbow, confetti};

// Number of animation patterns available (2: rainbow and confetti)
static constexpr uint8_t NUM_ANIMATION_PATTERNS = 2;

// Master animation mode cycling
static uint32_t g_last_animation_change_ms = 0;
static constexpr uint32_t ANIMATION_CYCLE_MS = ORCHESTRA_CYCLE_ADV_MS + ORCHESTRA_CYCLE_OFF_MS;

void sketchSetup() {
  // Give yourself a moment to connect a serial monitor or re-flash
  delay(2000);

  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(60);
  
  // Initialize animation change timer
  g_last_animation_change_ms = millis();
}

// Update animation mode for master nodes based on the Orchestra cycle timing.
// Changes animation every ORCHESTRA_CYCLE_ADV_MS + ORCHESTRA_CYCLE_OFF_MS (40 seconds).
static void update_master_animation_mode() {
  if (ORCHESTRA_ROLE != OrchestraRole::Master) {
    return;  // Only masters cycle animation modes
  }
  
  uint32_t now_ms = millis();
  uint32_t elapsed_ms = now_ms - g_last_animation_change_ms;
  
  if (elapsed_ms >= ANIMATION_CYCLE_MS) {
    g_last_animation_change_ms = now_ms;
    g_animation_mode = (g_animation_mode + 1) % NUM_ANIMATION_PATTERNS;
    ESP_LOGI("SKETCH", "Master animation mode changed to %u (cycle period: %lu ms)", 
             g_animation_mode, (unsigned long)ANIMATION_CYCLE_MS);
  }
}

uint8_t hue = 0;

void sketchLoop() {
  // Update master animation mode based on Orchestra cycle timing
  update_master_animation_mode();

  // Ensure animation mode is within valid bounds
  if (g_animation_mode >= NUM_ANIMATION_PATTERNS) {
    g_animation_mode = 0;
  }

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