// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Error.h"
#include "System.h"

class Channel;

// Synchronous $H-equivalent used by compatibility layers that need Marlin's
// blocking G28 semantics.  Runs the full safety checks and waits until homing
// reaches Idle/Alarm before returning.
Error home_axes(AxisMask axisMask, Channel& out);
