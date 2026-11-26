// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "LumenCompat.h"

#include "Config.h"
#include "GCode.h"  // MaxUserDigitalPin
#include "Protocol.h"
#include "System.h"
#include "Machine/Axes.h"
#if MAX_N_I2C
#    include "Machine/I2CBus.h"
#endif
#include "Machine/MachineConfig.h"
#include "Machine/UserOutputs.h"

#include <ctype.h>
#include <cstdio>
#include <cstring>

namespace Machine {
    namespace {
        constexpr int kM150Len = 4;
        constexpr int kM106Len = 4;
        constexpr int kM114Len = 4;
        constexpr int kM115Len = 4;
        constexpr int kM204Len = 4;
        constexpr int kM260Len = 4;
        constexpr int kM261Len = 4;
        constexpr int kM400Len = 4;

        bool isCodeBoundary(char c) {
            return c == '\0' || isalpha(uint8_t(c));
        }

        // Marlin's Lumen firmware warns for the eight reserved 7-bit I2C addresses
        // (0..7); use the same valid range so a host cannot address a reserved
        // device.
        bool validI2cAddress(int address) {
            return address >= 8 && address <= 127;
        }
    }

    bool LumenCompat::startsWithCode(const char* line, const char* code) {
        size_t len = strlen(code);
        return strncmp(line, code, len) == 0 && isCodeBoundary(line[len]);
    }

    bool LumenCompat::parseParams(const char* line, int offset, int32_t value[26], bool present[26]) {
        for (int i = 0; i < 26; ++i) {
            value[i]   = 0;
            present[i] = false;
        }

        const char* p = line + offset;
        while (*p) {
            if (!isalpha(uint8_t(*p))) {
                return false;
            }
            const int slot = toupper(uint8_t(*p)) - 'A';
            if (slot < 0 || slot >= 26) {
                return false;
            }
            ++p;
            if (!isdigit(uint8_t(*p))) {
                // Marlin allows a bare word to mean 255 (M150 R U B).
                value[slot]   = 255;
                present[slot] = true;
                continue;
            }
            int32_t v = 0;
            while (isdigit(uint8_t(*p))) {
                v = v * 10 + (*p - '0');
                if (v > 1000000) {
                    return false;
                }
                ++p;
            }
            value[slot]   = v;
            present[slot] = true;
        }
        return true;
    }

    void LumenCompat::init() {
        if (_initialized || !_configured) {
            return;
        }
        _initialized = true;

        if (_topPin.defined()) {
            if (_top.begin(_topPin, _count)) {
                log_info("LumenCompat: top WS2812 strip on " << _topPin.name() << ", " << int(_count) << " pixels");
            }
        }
        if (_bottomPin.defined()) {
            if (_bottom.begin(_bottomPin, _count)) {
                log_info("LumenCompat: bottom WS2812 strip on " << _bottomPin.name() << ", " << int(_count) << " pixels");
            }
        }

        if (_startupTest) {
            // Match the stock LumenPnP Marlin startup test
            // (NEOPIXEL_STARTUP_TEST then NEOPIXEL2_STARTUP_TEST): each ready
            // strip runs red, green, blue for 500 ms each and then turns off.
            // The stock firmware initializes the two strips sequentially, so
            // two configured strips take 3 s total.
            auto runStrip = [](WS2812Strip& strip) {
                if (!strip.ready()) {
                    return;
                }
                strip.setColor(255, 0, 0);
                strip.show();
                delay(500);
                strip.setColor(0, 255, 0);
                strip.show();
                delay(500);
                strip.setColor(0, 0, 255);
                strip.show();
                delay(500);
                strip.setColor(0, 0, 0);
                strip.show();
            };
            runStrip(_top);
            runStrip(_bottom);
        }
    }

    void LumenCompat::preprocessLine(char* line, size_t size) {
        if (!_configured || !line) {
            return;
        }
        const bool stripA = Axes::_numberAxis <= A_AXIS;
        const bool stripB = Axes::_numberAxis <= B_AXIS;
        if (!stripA && !stripB) {
            return;
        }
        if (line[0] != 'G') {
            return;
        }
        char* p = line + 1;
        int   code = 0;
        while (isdigit(uint8_t(*p))) {
            code = code * 10 + (*p - '0');
            ++p;
        }
        if (code != 0 && code != 1 && code != 92) {
            return;
        }

        char   out[128];
        size_t n = 0;
        p        = line;
        bool removed = false;
        while (*p && n + 1 < sizeof(out)) {
            if (((*p == 'A' && stripA) || (*p == 'B' && stripB))) {
                char* q = p + 1;
                if (*q == '+' || *q == '-') {
                    ++q;
                }
                if (isdigit(uint8_t(*q)) || *q == '.') {
                    // Skip the whole axis word.
                    p = q;
                    while (*p && (isdigit(uint8_t(*p)) || *p == '.')) {
                        ++p;
                    }
                    removed = true;
                    continue;
                }
            }
            out[n++] = *p++;
        }
        out[n] = '\0';
        if (removed && n < size) {
            strncpy(line, out, size);
            line[size - 1] = '\0';
            if (!_abWarned) {
                _abWarned = true;
                log_info("LumenCompat: ignored A/B words (no A/B axes configured)");
            }
        }
    }

    Error LumenCompat::executeLine(const char* line, Channel& channel, bool& handled) {
        handled = false;
        if (!_configured || !line) {
            return Error::Ok;
        }

        if (startsWithCode(line, "M150")) {
            handled = true;
            return ledCommand(line, kM150Len, channel);
        }
        if (startsWithCode(line, "M106") || startsWithCode(line, "M107")) {
            handled = true;
            return pumpCommand(line, kM106Len, line[2] == '0' && line[3] == '6');
        }
        if (startsWithCode(line, "M114")) {
            handled = true;
            return positionCommand(channel);
        }
        if (startsWithCode(line, "M115")) {
            handled = true;
            return infoCommand(channel);
        }
        if (startsWithCode(line, "M204")) {
            handled = true;
            // The planner acceleration is configured in the machine config.
            // LumenPnP sends M204 T3500 at connect time and then expects ok.
            return Error::Ok;
        }
        if (startsWithCode(line, "M400")) {
            handled = true;
            protocol_buffer_synchronize();
            return Error::Ok;
        }
        if (startsWithCode(line, "M260")) {
            handled = true;
            return i2cSendCommand(line, kM260Len);
        }
        if (startsWithCode(line, "M261")) {
            handled = true;
            return i2cRequestCommand(line, kM261Len, channel);
        }
        return Error::Ok;
    }

    Error LumenCompat::ledCommand(const char* line, int codeLen, Channel& channel) {
        int32_t value[26];
        bool    present[26];
        if (!parseParams(line, codeLen, value, present)) {
            log_info("M150: malformed parameters");
            return Error::InvalidValue;
        }
        for (int i = 0; i < 26; ++i) {
            const char c = char('A' + i);
            if (present[i] && !strchr("PRUBWSIK", c)) {
                log_info("M150: unsupported parameter " << c);
                return Error::InvalidValue;
            }
        }

        for (char c : { 'P', 'R', 'U', 'B', 'W' }) {
            if (present[c - 'A'] && (value[c - 'A'] < 0 || value[c - 'A'] > 255)) {
                log_info("M150: " << c << " must be 0..255");
                return Error::InvalidValue;
            }
        }

        const bool hasBrightness = present['P' - 'A'];
        const bool hasStrip      = present['S' - 'A'];
        const bool hasIndex      = present['I' - 'A'];
        const bool keep          = present['K' - 'A'];

        int strip = -1;
        if (hasStrip) {
            if (value['S' - 'A'] < 0 || value['S' - 'A'] > 1) {
                log_info("M150: S must be 0 (bottom) or 1 (top)");
                return Error::InvalidValue;
            }
            strip = value['S' - 'A'];
        }

        int index = -1;
        if (hasIndex) {
            if (value['I' - 'A'] < 0 || value['I' - 'A'] >= int(_count)) {
                log_info("M150: I out of range 0.." << int(_count) - 1);
                return Error::InvalidValue;
            }
            index = value['I' - 'A'];
        }

        const uint8_t brightness = hasBrightness ? uint8_t(value['P' - 'A'] & 0xFF) : 0;
        const bool    hasColor   = present['R' - 'A'] || present['U' - 'A'] || present['B' - 'A'] || present['W' - 'A'];

        bool ledOk = true;
        auto apply = [&](WS2812Strip& target) {
            if (!target.ready() || (strip >= 0 && strip != (&target == &_bottom ? 0 : 1))) {
                return;
            }

            const uint8_t r = present['R' - 'A'] ? uint8_t(value['R' - 'A'] & 0xFF) : 0;
            const uint8_t g = present['U' - 'A'] ? uint8_t(value['U' - 'A'] & 0xFF) : 0;
            const uint8_t b = present['B' - 'A'] ? uint8_t(value['B' - 'A'] & 0xFF) : 0;

            // Stock LumenPnP Marlin treats an omitted P as "keep the
            // current brightness" and an omitted color word as zero.  A
            // brightness-only M150 (for example M150 P0 or M150 P255) therefore
            // writes black at that brightness, matching the reference firmware.
            if (!hasColor && !keep) {
                if (index >= 0) {
                    // M150 I<n> with no color words clears only that pixel.
                    target.setPixel(uint8_t(index), 0, 0, 0);
                    if (hasBrightness) {
                        target.setBrightness(brightness);
                    }
                    if (!target.show()) {
                        ledOk = false;
                    }
                } else if (hasBrightness) {
                    target.setColor(0, 0, 0);
                    target.setBrightness(brightness);
                    if (!target.show()) {
                        ledOk = false;
                    }
                } else {
                    // Bare M150 is Marlin's "all LEDs off".
                    target.setColor(0, 0, 0);
                    if (!target.show()) {
                        ledOk = false;
                    }
                }
                return;
            }

            if (index >= 0) {
                if (keep) {
                    auto old = target.pixel(uint8_t(index));
                    if (!hasColor) {
                        // K with no color words keeps the old pixel unchanged.
                        if (hasBrightness) {
                            target.setBrightness(brightness);
                            if (!target.show()) {
                        ledOk = false;
                    }
                        }
                        return;
                    }
                    target.setPixel(uint8_t(index),
                                    present['R' - 'A'] ? r : old.r,
                                    present['U' - 'A'] ? g : old.g,
                                    present['B' - 'A'] ? b : old.b);
                } else {
                    target.setPixel(uint8_t(index), r, g, b);
                }
            } else if (keep) {
                for (uint8_t i = 0; i < target.count(); ++i) {
                    auto old = target.pixel(i);
                    target.setPixel(i,
                                    present['R' - 'A'] ? r : old.r,
                                    present['U' - 'A'] ? g : old.g,
                                    present['B' - 'A'] ? b : old.b);
                }
            } else {
                target.setColor(r, g, b);
            }

            if (hasBrightness) {
                target.setBrightness(brightness);
            }
            if (!target.show()) {
                ledOk = false;
            }
        };

        if (strip == 0) {
            apply(_bottom);
        } else if (strip == 1) {
            apply(_top);
        } else {
            apply(_bottom);
            apply(_top);
        }

        (void)channel;
        return ledOk ? Error::Ok : Error::InvalidValue;
    }

    Error LumenCompat::pumpCommand(const char* line, int codeLen, bool on) {
        int32_t value[26];
        bool    present[26];
        if (!parseParams(line, codeLen, value, present)) {
            return Error::InvalidValue;
        }
        for (int i = 0; i < 26; ++i) {
            const char c = char('A' + i);
            if (present[i] && !strchr("PS", c)) {
                log_info("M106/M107: unsupported parameter " << c);
                return Error::InvalidValue;
            }
        }

        const int index = present['P' - 'A'] ? value['P' - 'A'] : 0;
        // LumenPnP TPS4H160 outputs: P0=Left_V, P1=Left_P,
        // P2=Right_V, P3=Right_P.  They are switched, not PWM.
        constexpr int LUMEN_OUTPUTS = 4;
        if (index < 0 || index >= LUMEN_OUTPUTS) {
            log_info("M106/M107: P must be 0.." << LUMEN_OUTPUTS - 1);
            return Error::InvalidValue;
        }

        const bool enabled = on && (!present['S' - 'A'] || value['S' - 'A'] != 0);
        if (!config || !config->_userOutputs) {
            log_info("M106/M107: no user_outputs configured");
            return Error::InvalidValue;
        }
        config->_userOutputs->setDigital(size_t(index), enabled);
        return Error::Ok;
    }

    Error LumenCompat::positionCommand(Channel& channel) {
        if (!config || !config->_axes) {
            return Error::InvalidValue;
        }
        float* mpos = get_mpos();
        float* wco  = get_wco();
        // Marlin M114 reports the current (work) position, not machine
        // position.  G92 changes the WCO in FluidNC, so subtract it here.
        // The stock LumenPnP regex expects all five of X/Y/Z/A/B; report
        // missing axes as 0.000 until their motors are configured.
        static const char axisLetters[] = "XYZAB";
        for (size_t i = 0; i < 5; ++i) {
            if (i) {
                channel.print(' ');
            }
            channel.print(axisLetters[i]);
            channel.print(':');
            const float value = axis_t(i) < Axes::_numberAxis ? mpos[i] - wco[i] : 0.0f;
            channel.print(value, 3);
        }
        channel.println();
        return Error::Ok;
    }

    Error LumenCompat::infoCommand(Channel& channel) {
        channel.println("FIRMWARE_NAME:FluidNC MACHINE_TYPE:LumenPnP COMPATIBLE PROTOCOL_VERSION:1.0");
        return Error::Ok;
    }

    Error LumenCompat::i2cSendCommand(const char* line, int codeLen) {
#if !MAX_N_I2C
        (void)line;
        (void)codeLen;
        log_info("M260: I2C is not available on this platform");
        return Error::InvalidValue;
#else
        int32_t value[26];
        bool    present[26];
        if (!parseParams(line, codeLen, value, present)) {
            return Error::InvalidValue;
        }
        for (int i = 0; i < 26; ++i) {
            const char c = char('A' + i);
            if (present[i] && !strchr("ABSR", c)) {
                return Error::InvalidValue;
            }
        }
        if (present['R' - 'A']) {
            _i2cLength = 0;
        }

        if (present['A' - 'A']) {
            const int address = value['A' - 'A'];
            if (!validI2cAddress(address)) {
                // Never leave the previous address armed after a rejected A
                // word; the next M260 B/S sequence must not write to the old
                // device.
                _i2cAddress = 0;
                _i2cLength  = 0;
                return Error::InvalidValue;
            }
            _i2cAddress = uint8_t(address);
            _i2cLength  = 0;
        }
        if (present['B' - 'A']) {
            if (!present['A' - 'A'] && _i2cAddress == 0) {
                return Error::InvalidValue;
            }
            if (_i2cLength >= sizeof(_i2cBuffer)) {
                return Error::InvalidValue;
            }
            _i2cBuffer[_i2cLength++] = uint8_t(value['B' - 'A'] & 0xFF);
        }
        if (present['S' - 'A']) {
            if (_i2cBus >= MAX_N_I2C || !config->_i2c[_i2cBus]) {
                log_info("M260: I2C bus " << int(_i2cBus) << " is not configured");
                return Error::InvalidValue;
            }
            const size_t length = _i2cLength;
            int          result = config->_i2c[_i2cBus]->write(_i2cAddress, _i2cBuffer, length);
            _i2cLength = 0;
            if (result != int(length)) {
                return Error::InvalidValue;
            }
        }
        return Error::Ok;
#endif
    }

    Error LumenCompat::i2cRequestCommand(const char* line, int codeLen, Channel& channel) {
#if !MAX_N_I2C
        (void)line;
        (void)codeLen;
        (void)channel;
        log_info("M261: I2C is not available on this platform");
        return Error::InvalidValue;
#else
        int32_t value[26];
        bool    present[26];
        if (!parseParams(line, codeLen, value, present) || !present['A' - 'A'] || !present['B' - 'A']) {
            return Error::InvalidValue;
        }
        const int address = value['A' - 'A'];
        const int count   = value['B' - 'A'];
        const int style   = present['S' - 'A'] ? value['S' - 'A'] : 0;
        if (!validI2cAddress(address) || count <= 0 || count > int(sizeof(_i2cBuffer))) {
            return Error::InvalidValue;
        }
        if (_i2cBus >= MAX_N_I2C || !config->_i2c[_i2cBus]) {
            log_info("M261: I2C bus " << int(_i2cBus) << " is not configured");
            return Error::InvalidValue;
        }

        uint8_t data[sizeof(_i2cBuffer)];
        int     result = config->_i2c[_i2cBus]->read(uint8_t(address), data, size_t(count));
        if (result != count) {
            return Error::InvalidValue;
        }

        if (style == 2 && count <= 2) {
            uint16_t v = 0;
            for (int i = 0; i < count; ++i) {
                v = uint16_t((v << 8) | data[i]);
            }
            channel.print("data:");
            channel.println(v);
        } else if (style == 1) {
            channel.print("data:");
            static const char hex[] = "0123456789ABCDEF";
            for (int i = 0; i < count; ++i) {
                if (i) {
                    channel.print(' ');
                }
                channel.print(hex[data[i] >> 4]);
                channel.print(hex[data[i] & 0x0F]);
            }
            channel.println();
        } else {
            channel.print("data:");
            for (int i = 0; i < count; ++i) {
                channel.print(data[i]);
                if (i + 1 < count) {
                    channel.print(' ');
                }
            }
            channel.println();
        }
        return Error::Ok;
#endif
    }

    void LumenCompat::group(Configuration::HandlerBase& handler) {
        if (handler.handlerType() == Configuration::HandlerType::Parser) {
            _configured = true;
        }
        // @config top_pin
        // @default NO_PIN
        // @tuning per-machine
        // WS2812B data pin for the top camera-light strip.
        handler.item("top_pin", _topPin);

        // @config bottom_pin
        // @default NO_PIN
        // @tuning per-machine
        // WS2812B data pin for the bottom camera-light strip.
        handler.item("bottom_pin", _bottomPin);

        // @config count
        // @default 8
        // @tuning per-machine
        // Number of pixels in each strip (1..8).
        handler.item("count", _count, 1, 8);

        // @config startup_test
        // @default false
        // @tuning typical
        // Run the stock top-then-bottom red/green/blue startup test (1.5 s each).
        handler.item("startup_test", _startupTest);
#if MAX_N_I2C
        // @config i2c_bus
        // @default 0
        // @tuning per-machine
        // I2C bus used by M260/M261 commands.
        handler.item("i2c_bus", _i2cBus, 0, 1);
#endif
    }
}
