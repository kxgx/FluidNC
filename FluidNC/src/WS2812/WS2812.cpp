// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "WS2812.h"

#include "Config.h"

#if defined(ESP32)
#    include <soc/soc_caps.h>
#    if SOC_RMT_SUPPORTED
#        include "esp32-hal-rmt.h"
#    endif
#endif

namespace Machine {
    bool WS2812Strip::begin(const Pin& pin, uint8_t count) {
        if (count == 0) {
            log_error("WS2812: strip count must be greater than zero");
            _begun = false;
            return false;
        }
        if (!pin.capabilities().has(Pin::Capabilities::Native) || !pin.capabilities().has(Pin::Capabilities::Output)) {
            log_error("WS2812: data pin is not a native output");
            _begun = false;
            return false;
        }
        if (count > 8) {
            // One 8-pixel strip needs 192 RMT symbols, which is exactly the
            // four TX memory blocks available on the ESP32-S3.  Larger strips
            // need a DMA/RMT driver and are intentionally rejected here.
            log_error("WS2812: at most 8 pixels per strip are supported");
            _begun = false;
            return false;
        }
        _pin   = int16_t(pin.getNative(Pin::Capabilities::Native | Pin::Capabilities::Output));
        if (_pin < 0) {
            log_error("WS2812: invalid data pin");
            _begun = false;
            return false;
        }
        _count = count;
        _pixels.assign(count, RGB {});
        _begun = true;
        clear();
        return true;
    }

    void WS2812Strip::setPixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
        if (index >= _pixels.size()) {
            return;
        }
        _pixels[index] = RGB { r, g, b };
    }

    void WS2812Strip::setColor(uint8_t r, uint8_t g, uint8_t b) {
        for (auto& pixel : _pixels) {
            pixel = RGB { r, g, b };
        }
    }

    void WS2812Strip::clear() {
        for (auto& pixel : _pixels) {
            pixel = RGB {};
        }
    }

    bool WS2812Strip::show() {
        if (!_begun) {
            return false;
        }

#if defined(ESP32) && SOC_RMT_SUPPORTED
        const int native = _pin;

        std::vector<rmt_data_t> symbols;
        symbols.resize(size_t(_count) * 24);
        size_t n = 0;

        auto addBit = [&](bool one) {
            symbols[n].level0    = 1;
            symbols[n].duration0 = one ? 8 : 4;  // 0.8 us / 0.4 us at 10 MHz
            symbols[n].level1    = 0;
            symbols[n].duration1 = one ? 4 : 8;  // 0.4 us / 0.8 us
            ++n;
        };

        for (const auto& pixel : _pixels) {
            uint8_t g = uint8_t((uint16_t(pixel.g) * _brightness + 127) / 255);
            uint8_t r = uint8_t((uint16_t(pixel.r) * _brightness + 127) / 255);
            uint8_t b = uint8_t((uint16_t(pixel.b) * _brightness + 127) / 255);
            uint8_t bytes[3] = { g, r, b };  // WS2812B is GRB
            for (uint8_t byte : bytes) {
                for (int bit = 7; bit >= 0; --bit) {
                    addBit((byte >> bit) & 1);
                }
            }
        }

        // 8 pixels * 24 symbols = 192 symbols; one ESP32-S3 RMT TX memory
        // allocation is 48 symbols, so four blocks are required.
        bool ok = rmtInit(native, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_4, 10000000);
        if (!ok) {
            log_error("WS2812: RMT init failed on pin " << int(native));
            return false;
        }
        ok = rmtWrite(native, symbols.data(), symbols.size(), 100);
        rmtDeinit(native);
        pinMode(native, OUTPUT);
        digitalWrite(native, LOW);
        if (!ok) {
            log_error("WS2812: RMT write failed on pin " << int(native));
        }
        return ok;
#else
        log_error("WS2812: no supported backend on this platform");
        return false;
#endif
    }
}
