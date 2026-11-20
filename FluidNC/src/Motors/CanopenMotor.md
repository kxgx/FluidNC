## CANopen motor driver

`canopen` drives a standard CANopen (CiA301) / CiA402 closed-loop stepper over
the shared CAN bus.  It is a **standard-protocol** driver: it is not tied to one
vendor.  Any drive that really implements CANopen with the CiA301 default
COB-IDs and a CiA402 drive-profile object dictionary can be used, and its object
indices/COB-IDs can be overridden per motor when they differ.

> **UNVERIFIED / 未上机验证**: 张大头 X42S V2.0 / X28S V2.0 的厂商手册没有
> CANopen 教程，也没有给出对象字典。本驱动按标准 CiA402 和默认对象索引
> （0x6040/0x6041/0x6060/0x6061/0x607A/0x6064/0x60FF/0x6502）实现，
> 上机前必须核对驱动器的实际对象字典和 PDO/SDO COB-ID。本固件未与任何
> CANopen 硬件联调，未做运动测试。

## How it differs from the vendor protocols

| | `pd42s1` / `zdt` | `canopen` |
| --- | --- | --- |
| Reuse | one vendor's private protocol | any CANopen/CiA402 vendor |
| CAN frames | 29-bit extended | 11-bit standard (CiA301) |
| Command framing | vendor byte stream over CAN | NMT / SDO / heartbeat |
| Position command | vendor function code | SDO write to 0x607A |
| Status | vendor function code | SDO read of 0x6064 / 0x6041 |

There is deliberately **no private-protocol fallback**: a drive that does not
answer standard CANopen SDOs is reported as absent rather than being poked with
a guessed command.  `$CANScan`, `$CANStatus`, `$CANFaults`, `$CANPositions` and
`$CANConfig` use the same node configuration the motion path uses.

## Wiring

The ESP32 has a CAN controller but no transceiver, so an external 3.3 V
transceiver is required (SN65HVD230 or equivalent).  Wiring is identical to the
other CAN motor drivers -- see `Pd42s1Motor.md` for the pin table.  Terminate
the bus with 120 ohms at both physical ends.

The CAN bit rate is not negotiated by CANopen: every node (and `can: baud`) must
already use the same rate.  Node IDs must be unique on the bus.

## Configuration

```yaml
can:
  driver: canopen        # required; the default driver is still pd42s1
  tx_pin: gpio.4
  rx_pin: gpio.5
  baud: 500000

axes:
  x:
    steps_per_mm: 3200
    max_rate_mm_per_min: 3000
    acceleration_mm_per_sec2: 500
    motor0:
      canopen:
        address: 1              # CANopen node id (CiA301 node_id), 1..16 here
        counts_per_step: 1.0
        counts_per_rev: 10000
        invert_direction: false
        update_ms: 20
        status_poll_ms: 1000
```

`address` is the drive's CANopen node id.  `node_id` is accepted as an alias
(when both are set they must be equal).  The shared FluidNC CAN address
bookkeeping limits it to 1..16 even though CiA301 allows 1..127.

`counts_per_step` converts FluidNC steps to the drive's position units:
`counts_per_step = counts_per_rev / (steps_per_mm * mm_per_revolution)`.  The
X42S/X28S manuals do not state a CANopen position-unit default; `counts_per_rev`
(default 10000) is the value used for the optional `speed_rpm` conversion and
for sizing the idle following-error window, and must be checked on the drive.

## Default object dictionary and COB-IDs

These are the CiA301/CiA402 defaults.  Override the `*_index` / `sdo_*_cobid`
items when the drive's dictionary deviates.

| Item | Default | Meaning |
| --- | --- | --- |
| SDO request COB-ID | `0x600 + node` | client -> server (`sdo_request_cobid`) |
| SDO response COB-ID | `0x580 + node` | server -> client (`sdo_response_cobid`) |
| RxPDO1 / TxPDO1 | `0x200 + node` / `0x180 + node` | PDO1 (codec only, see below) |
| RxPDO2 / TxPDO2 | `0x300 + node` / `0x280 + node` | PDO2 (codec only) |
| Heartbeat | `0x700 + node` | node state |
| `controlword_index` | 0x6040 | CiA402 controlword |
| `statusword_index` | 0x6041 | CiA402 statusword |
| `mode_index` | 0x6060 | modes of operation (read-only diagnostic; display is mode+1 = 0x6061) |
| `target_position_index` | 0x607A | absolute target position |
| `actual_position_index` | 0x6064 | position actual value |
| Device type | 0x1000 | read during `$CANScan` |
| Supported drive modes | 0x6502 | read during `$CANConfig` |
| Profile velocity | 0x6081 | written only when `speed_rpm > 0` |
| Error register | 0x1001 | read during `$CANFaults` |

Only expedited (<= 4 byte) SDO transfers are used, which is what CiA301 requires
for these objects.  Segmented transfers are rejected with an SDO abort.

## Control path

1. FluidNC generates its normal step/direction stream; the step ISR only
   accumulates a signed step counter.
2. On the first poll, and after any resync, the CAN worker reads the actual
   position (0x6064) and statusword (0x6041) via SDO, then adopts the actual
   position as the base for the current step count.  A resync adopts the
   **actual** position, not the old commanded target, so lost steps cannot turn
   into a jump.
3. Enable/disable use the CiA402 state machine: shutdown -> switch on -> enable
   operation for `$ME`/enable, shutdown for disable.
4. Every `update_ms` the worker converts the step delta to drive position units
   and writes the absolute target to 0x607A via SDO.
5. Every `status_poll_ms` it reads 0x6064 and 0x6041.  A drive that reports
   FAULT, leaves operation-enabled, or stops answering is not streamed to: the
   client resynchronises and re-runs the enable sequence.  Three consecutive
   failed transactions raise a FluidNC hard-stop alarm (the other closed-loop
   CAN drivers behave the same way).
6. While the step counter is idle, a position mismatch larger than 5% of
   `counts_per_rev` (minimum 100 units) re-anchors the base to the actual
   position.  This is the "lost step" resynchronisation.
7. `set_homing_mode()` returns true: homing is normal FluidNC step-stream
   homing, **not** CiA402 homing mode (0x6060 = 6).

`speed_rpm` (optional) writes 0x6081 as `counts_per_rev * speed_rpm / 60`
position units per second.  That unit assumption is unverified; leave it 0 to
let the drive use its own profile velocity.

`setpoint_toggle` (default false) additionally toggles controlword bit 4 (new
set-point) after each 0x607A write.  Some profile-position implementations only
latch the target on that edge, while others (cyclic synchronous position,
vendor auto-latch) follow 0x607A directly.  The X42S/X28S manuals do not say
which behaviour they use, so this is opt-in.

**The mode of operation is not written by this firmware.**  The drive must
already be configured (vendor tool or its own startup) for a position mode that
accepts target-position updates, e.g. profile position (1) or cyclic synchronous
position (8).  `$CANConfig` prints the supported-drive-modes object (0x6502) and
the current mode display (0x6061) to check this.

## Commands

Supported:

| Command | Behaviour |
| --- | --- |
| `$CANScan` | SDO read of 0x1000 per configured node, heartbeat fallback |
| `$CANStatus` | TWAI state, config, and statusword/state per node |
| `$CANFaults` | statusword state + error register 0x1001 per node |
| `$CANPositions` | actual position, target position, mode, state per node |
| `$CANConfig` | effective COB-ID/object config, device type, supported modes, heartbeat |
| `$CANEnable=id` | CiA402 shutdown -> switch on -> enable operation |
| `$CANMove=id,delta` | read 0x6064, add delta, write 0x607A |
| `$CANStop` / `M157` | CiA402 quick stop (controlword 0x0002), latches control off |
| `$CANReset` | NMT reset-node + NMT start, invalidates all positions, releases the stop latch |
| `$CANClear` | CiA402 fault reset when a node is in FAULT |
| `$CANAutoConfig` | no-op success: CANopen needs no echo/bit-rate auto-config |

Not supported (reported as unsupported, not faked):

| Command | Why |
| --- | --- |
| `$CANZero` | CiA402 has no standard "set actual position to zero" object |
| `$CANGroup` | CiA301 has no group addressing; use NMT broadcast or node ids |
| `$CANEchoOn` | not applicable; a CANopen server always answers SDO |
| `$CANRead` (M159) | CANopen addresses objects by index/sub-index, not a byte function code |
| `$CANScan all` | the bit rate is not negotiated in-band; scans at `can.baud` |

## Limitations / risks

* No hardware verification of any kind; the object dictionary is assumed.
* Node IDs are limited to 1..16 by the shared CAN address bookkeeping, although
  CiA301 allows 1..127.  `$CANEnable`/`$CANMove` also accept only 1..16.
* PDO1/PDO2 codecs exist in `CanopenProtocol` but PDOs are not used: the
  required PDO mapping (0x1400/0x1600/0x1800/0x1A00) is drive specific and has
  not been configured.
* No segmented SDO, no NMT node guarding, no SYNC producer, no EMCY handling.
* The drive's mode of operation is not set; a drive left in velocity mode will
  ignore or misread the target position writes.
* If the drive firmware actually speaks a private "CANopen-like" protocol
  instead of CiA301, nothing here will work; check with `$CANScan`/`$CANConfig`
  before configuring motion.

## Verification order (read-only first, motion only after explicit confirmation)

1. With the machine idle, no motion configured, run `$CANScan`, `$CANStatus`,
   `$CANConfig`.  A node must answer 0x1000; the device type and supported drive
   modes must look plausible.
2. `$CANPositions` must show a stable actual position when the shaft is not
   moved by hand.
3. `$CANFaults` must show no FAULT.
4. Only after the user explicitly confirms a motion test and the
   small-travel rules from `AGENTS.md` (<= 5 mm, F300 first) may
   `$CANMove=id,small_delta` be tried -- one axis, hand clear, ready to stop.
5. If the direction is inverted, stop, fix `invert_direction` (or wiring), and
   re-run the read-only checks.

## Rollback

Revert the motor type in the configuration to the previous driver (`pd42s1` /
`zdt`) and set `can.driver` back to that protocol; no other firmware change is
needed.  The CANopen driver is additive and has no effect unless selected.
