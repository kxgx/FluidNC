// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Config.h"

#if MAX_N_CAN

namespace Machine {
    // How a CAN device speaks to the controller.
    //
    // Standard protocols are shared by every vendor that implements them and
    // should be implemented once in Can/Protocols/Standard/.  Private protocols
    // belong to one vendor and live in Can/Protocols/Private/<vendor>/.
    enum class CanProtocolClass {
        Standard,
        Private,
    };
}

#endif
