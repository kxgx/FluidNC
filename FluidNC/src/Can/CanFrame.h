// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include <cstddef>
#include <cstdint>

// One physical CAN frame.  This is a transport-level value type only; it does
// not know any vendor protocol.  Protocol codecs fill frames, CanBus sends
// them.  Keeping this separate lets protocol headers stay independent of the
// ESP32 TWAI/FreeRTOS implementation.
struct CanFrame {
    static constexpr size_t MAX_DATA = 8;

    uint32_t id       = 0;
    bool     extended = true;
    uint8_t  length   = 0;
    uint8_t  data[MAX_DATA] = {};
};
