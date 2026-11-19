// Standalone 8-pixel WS2812B test for the two machine camera-light strips.
// No FluidNC, CAN, motors, pumps or valves are initialised.
// GPIO11 = top strip data, GPIO12 = bottom strip data.

#include <Arduino.h>
#include "esp32-hal-rmt.h"

static constexpr uint8_t TOP_PIN    = 11;
static constexpr uint8_t BOTTOM_PIN = 12;
static constexpr int     PIXELS     = 8;

static bool showStrip(uint8_t pin, uint8_t r, uint8_t g, uint8_t b) {
    static rmt_data_t symbols[PIXELS * 24];

    size_t n = 0;
    for (int p = 0; p < PIXELS; ++p) {
        // WS2812B wire order: G R B
        const uint8_t bytes[3] = { g, r, b };
        for (int i = 0; i < 3; ++i) {
            uint8_t value = bytes[i];
            for (int bit = 7; bit >= 0; --bit) {
                const bool one = value & (1u << bit);
                symbols[n].level0    = 1;
                symbols[n].duration0 = one ? 8 : 4;
                symbols[n].level1    = 0;
                symbols[n].duration1 = one ? 4 : 8;
                ++n;
            }
        }
    }

    if (!rmtInit(pin, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_4, 10000000)) {
        Serial.printf("LED_TEST: rmtInit failed pin %u\n", pin);
        return false;
    }
    bool ok = rmtWrite(pin, symbols, n, 1000);
    rmtDeinit(pin);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    if (!ok) {
        Serial.printf("LED_TEST: rmtWrite failed pin %u\n", pin);
    }
    return ok;
}

void setup() {
    Serial.begin(115200);
    pinMode(TOP_PIN, OUTPUT);
    pinMode(BOTTOM_PIN, OUTPUT);
    digitalWrite(TOP_PIN, LOW);
    digitalWrite(BOTTOM_PIN, LOW);
    delay(1500);
    Serial.println();
    Serial.println("LED_TEST: 8-pixel WS2812B, GPIO11=top GPIO12=bottom");
    Serial.println("LED_TEST: no CAN, no motors, no pumps, no valves");
}

void loop() {
    Serial.println("LED_TEST: top red / bottom green");
    showStrip(TOP_PIN, 255, 0, 0);
    showStrip(BOTTOM_PIN, 0, 255, 0);
    delay(4000);

    Serial.println("LED_TEST: both white");
    showStrip(TOP_PIN, 255, 255, 255);
    showStrip(BOTTOM_PIN, 255, 255, 255);
    delay(4000);

    Serial.println("LED_TEST: both off");
    showStrip(TOP_PIN, 0, 0, 0);
    showStrip(BOTTOM_PIN, 0, 0, 0);
    delay(3000);
}
