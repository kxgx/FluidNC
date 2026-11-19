// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Pin.h"

#include <cstdint>
#include <vector>

namespace Machine {
    // A single WS2812B strip (GRB order, 800 kHz).  The ESP32 backend uses one
    // RMT channel at a time and releases it after each show(), so two strips do
    // not need two simultaneous RMT memory allocations.
    class WS2812Strip {
    public:
        struct RGB {
            uint8_t r = 0;
            uint8_t g = 0;
            uint8_t b = 0;
        };

        WS2812Strip() = default;

        bool begin(const Pin& pin, uint8_t count);
        bool ready() const { return _begun; }
        uint8_t count() const { return _count; }

        void setPixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
        void setColor(uint8_t r, uint8_t g, uint8_t b);
        void setBrightness(uint8_t brightness) { _brightness = brightness; }
        uint8_t brightness() const { return _brightness; }
        const RGB& pixel(uint8_t index) const {
            static const RGB zero {};
            return index < _pixels.size() ? _pixels[index] : zero;
        }
        void clear();

        // Sends the current pixel buffer.  Returns false if the strip is not
        // configured or the RMT transaction failed.
        bool show();

    private:
        int16_t              _pin        = -1;
        std::vector<RGB>     _pixels;
        uint8_t              _count      = 0;
        uint8_t              _brightness = 255;
        bool                 _begun      = false;
    };
}
