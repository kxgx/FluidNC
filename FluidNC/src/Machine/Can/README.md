# CAN motor drivers

The CAN subsystem has three separate concerns:

```text
CanBus (Machine/CanBus.*)
  TWAI transport, worker task, transaction lock, client list.
  No vendor command bytes belong here.

CanDriver (Machine/CanDriver.h)
  The command/diagnostic surface used by $CAN... and M15x.
  can.driver selects an implementation from CanDriverRegistry.

<vendor>Motor (Motors/<vendor>Motor.*)
  MotorDriver + CanBusClient.  Owns the axis state machine: step counting,
  target streaming, enable/disable, homing mode and stop handling.

<vendor>Protocol (Motors/<vendor>Protocol.*)
  Pure frame encoder/decoder with no FreeRTOS or Pin dependencies.  Keep it
  host-testable.
```

## Adding a vendor

1. Add `<vendor>Protocol.h/.cpp` with frame builders/parsers and unit tests.
2. Add `<vendor>Motor.h/.cpp` implementing `MotorDriver` and
   `Machine::CanBusClient`.  Register it with
   `MotorFactory::InstanceBuilder<...>("<vendor>")`.
3. Add `<vendor>CanDriver.h/.cpp` implementing `Machine::CanDriver`.  Register
   it with `CanDriverRegistration("<vendor>", factory)`.
4. Make the motor constructor call `Machine::CanBus::mark_driver_configured()`
   and its validator call `register_configured_address()`.
5. Add a section to `tools/build_config_docs.py` and regenerate
   `docs/config_items.yaml/.json`.
6. Add the new `.cpp` files to `FluidNC/src/CMakeLists.txt` (PlatformIO uses a
   glob, ESP-IDF does not).
7. Document wiring, addresses, protocol family and checksum in a vendor `.md`.

A standalone `CanDriver` is initialized by `CanDriver::init_driver()` after the
shared TWAI transport starts.  The built-in `pd42s1` driver keeps its legacy
auto-configuration path; new drivers must not run PD42S1 probing on a different
protocol.

## Current drivers

- `pd42s1`: ALIENTEK PD42S1 (legacy in-place implementation in CanBus).
- `zdt`: Zhang Datou XS-series.  Per-motor `firmware` selects the Emm
  (pulse/default) or X (0.1 degree) command family, and `checksum` selects
  6B/XOR/CRC8.  Motion is streamed as relative targets, so no absolute
  position base can be invalidated by a soft reset.
