# CAN motor architecture

The CAN subsystem separates transport, protocol and axis motion.  No vendor
driver is privileged.

```text
CanBus (Can/CanBus.*)
  TWAI transport, worker task, transaction lock, client registry and generic
  bus state.  It contains no vendor command bytes and no vendor names.

CanDriver (Can/CanDriver.h + CanDriverRegistry.h)
  The command/diagnostic surface used by $CAN... and M15x.  can.driver selects
  one implementation from the registry.

Can/Protocols/Standard/
  Protocols shared by multiple vendors.  CANopen/CiA402 is the standard protocol
  family; a driver supporting it reuses CanopenProtocol/CanopenMotor.

Can/Protocols/Private/<vendor>/
  Vendor-only framing and payloads (for example PD42S1 and ZDT).

Motors/<vendor>Motor.*
  MotorDriver + CanBusClient.  Owns axis state: step counting, target streaming,
  enable/disable, homing mode and stop handling.
```

## Protocol classes

`CanDriver::protocol_class()` returns `CanProtocolClass::Standard` or
`CanProtocolClass::Private`.

- Standard: CANopen/CiA402, reusable by any compliant drive.  The object
  dictionary indices may still be overridden by motor configuration.
- Private: PD42S1 free protocol, ZDT free protocol, etc.

## Adding a vendor

1. Choose `Standard` or `Private`.
2. For `Private`, copy `Can/Protocols/PROTOCOL_TEMPLATE.h` to
   `Can/Protocols/Private/<vendor>/<Vendor>Protocol.*`; fill the metadata block
   and all protocol limits before writing the implementation.  See
   [`Protocols/README.md`](Protocols/README.md).  Add unit tests.
3. Add `Can/Drivers/<vendor>/<Vendor>Driver.*` implementing `CanDriver` and
   `protocol_class()`.  Register it with
   `CanDriverRegistration("<vendor>", factory)`.
4. Add `Motors/<Vendor>Motor.*` implementing `MotorDriver` and
   `Machine::CanBusClient`.  Register it with
   `MotorFactory::InstanceBuilder<...>("<vendor>")`.
5. Add the source files to `FluidNC/src/CMakeLists.txt`.
6. Add a section to `tools/build_config_docs.py` and regenerate
   `docs/config_items.yaml/.json`.
7. Document wiring, node address, protocol family and object dictionary in a
   vendor `.md`.

For a **standard** protocol, implement the protocol once under
`Can/Protocols/Standard/` and keep vendor files thin; only object-dictionary
defaults and wiring should differ.

## Current drivers

- `pd42s1` — private ALIENTEK PD42S1 CAN protocol.
- `zdt` — private Zhang Datou XS CAN protocol (Emm/X firmware, 6B/XOR/CRC8); see [`../Motors/ZdtMotor.md`](../Motors/ZdtMotor.md) for the position-stream contract, speed sources and rollback.
- `canopen` — standard CANopen/CiA402 protocol layer.
- `null_can` — command-surface stub; no transport.

Standard CANopen is implemented from the public CiA402 conventions because the
ZDT manuals in this workspace do not contain a CANopen tutorial.  Verify the
actual object dictionary and COB-IDs on the X42S V2.0 / X28S V2.0 before motion.
