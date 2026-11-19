// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Config.h"
#include "Configuration/Configurable.h"
#include "Error.h"
#include "Pin.h"
#include "WS2812/WS2812.h"

#include <cstddef>
#include <cstdint>

class Channel;

namespace Machine {
    // Compatibility layer for the G-code subset used by the LumenPnP OpenPnP
    // configuration.  It is deliberately kept outside the normal FluidNC GCode
    // parser so that Marlin-only words such as U (green) and the M150/M106/M260
    // command semantics do not collide with FluidNC's own CAN diagnostic M-codes.
    class LumenCompat : public Configuration::Configurable {
    public:
        LumenCompat() = default;

        void init();
        bool configured() const { return _configured; }

        // Remove A/B words from G0/G1/G92 while the machine has no A/B axes.
        // The stock LumenPnP OpenPnP machine.xml always sends A/B words; this
        // keeps a 3-axis bring-up machine usable.  No-op once A/B exist.
        void preprocessLine(char* line, size_t size);

        // Returns true when this line was consumed by the compatibility layer.
        // The normal GCode parser must then not process it.
        Error executeLine(const char* line, Channel& channel, bool& handled);

        void group(Configuration::HandlerBase& handler) override;

        Pin     _topPin;
        Pin     _bottomPin;
        uint8_t _count       = 8;
        bool    _startupTest = false;
        uint8_t  _i2cBus     = 0;

    private:
        Error ledCommand(const char* line, int codeLen, Channel& channel);
        Error pumpCommand(const char* line, int codeLen, bool on);
        Error positionCommand(Channel& channel);
        Error infoCommand(Channel& channel);
        Error i2cSendCommand(const char* line, int codeLen);
        Error i2cRequestCommand(const char* line, int codeLen, Channel& channel);

        static bool startsWithCode(const char* line, const char* code);
        static bool parseParams(const char* line, int offset, int32_t value[26], bool present[26]);

        WS2812Strip _top;
        WS2812Strip _bottom;
        bool        _initialized = false;
        bool        _configured  = false;
        bool        _abWarned    = false;

#if MAX_N_I2C
        uint8_t _i2cAddress = 0;
        uint8_t _i2cBuffer[32];
        size_t  _i2cLength = 0;
#endif
    };
}
