## PD42S1 CAN motor driver

`pd42s1` drives an ALIENTEK (正点原子) PD42S1 integrated closed-loop stepper
driver over CAN instead of with step/direction pulses.  FluidNC keeps its
normal motion engine, homing and limit handling: the step ISR accumulates a
signed step count, and a CAN worker task turns that count into absolute
position commands that the driver's own closed-loop controller follows.

Only the ESP32 targets that expose a TWAI controller (classic ESP32 and
ESP32-S3) can use this driver.  ESP32-S2 and the non-ESP32 targets cannot.

## Wiring

The ESP32 has a CAN controller but no transceiver, so an external 3.3 V
transceiver is required -- a SN65HVD230 is a good match because its logic
levels are 3.3 V too.

| SN65HVD230 | Connect to |
| --- | --- |
| 1 D | ESP32 TX pin (`can: tx_pin`) |
| 2 GND | common ground |
| 3 VCC | 3.3 V |
| 4 R | ESP32 RX pin (`can: rx_pin`) |
| 5 Vref | leave open |
| 6 CANL | bus CAN_L, twisted pair |
| 7 CANH | bus CAN_H, twisted pair |
| 8 Rs | 0 ohm to GND (high speed mode), or 10k..100k to GND for slope control |

Terminate the bus with 120 ohms at each physical end -- the controller end and
the last driver -- and nowhere else.  If the drivers have built-in
termination, leave only the two end nodes' resistors fitted.

## Preparing the drivers

Before connecting the bus, configure each driver with the vendor tool over
its UART port:

* slave address (command `0x60`): 1, 2, 3, ... one different value per motor
* CAN bit rate (command `0x68`): must match `can: baud`; the PD42S1 supports
  125000, 250000, 500000 and 1000000
* optionally the response CAN ID (command `0x6C`); the common command ID is
  0x1000, which is the default and what `can: command_id` expects
* save the parameters (command `0x04`)

## Configuration

```yaml
can:
  tx_pin: gpio.4
  rx_pin: gpio.5
  baud: 500000

axes:
  x:
    steps_per_mm: 3200
    max_rate_mm_per_min: 3000
    acceleration_mm_per_sec2: 500
    motor0:
      pd42s1:
        address: 1
        counts_per_step: 16
        speed_rpm: 600
        accel: 200
```

`counts_per_step` converts FluidNC steps to the driver's position units.
The PD42S1 counts 51200 units per revolution, so

    counts_per_step = 51200 / steps_per_revolution_on_the_axis
                    = 51200 / (steps_per_mm * mm_per_revolution)

With 3200 steps/mm on a 5 mm pitch screw the axis has 16000 steps per
revolution, so `counts_per_step` is 51200 / 16000 = 3.2.  Set it to 1.0 only
when `steps_per_mm` was chosen so that one FluidNC step equals one driver
position unit.

`speed_rpm` and `accel` are the limits the driver uses while following the
streamed positions; they must be at least the axis's real maximum speed and
acceleration, or the motor will lag behind the commanded position.  The
firmware sends a new absolute position every `update_ms` (default 20 ms).

## Limitations

* The driver is commanded in absolute position mode; FluidNC does not use the
  driver's encoder for closed-loop position correction.  `status_poll_ms`
  reads the position back for diagnostics only.
* Homing still uses FluidNC's limit switches.  During homing the driver
  follows the streamed positions, so it can overshoot the switch by the
  distance travelled between two position updates -- keep homing speeds
  modest and use a small `update_ms`.
* CAN 2.0 (classic) only; the PD42S1 does not support CAN FD.
* The protocol has no request IDs, so all transactions are serialized by the
  CAN worker task.  Adding more motors increases the time between position
  updates; keep the total CAN load in mind when using many axes or a low
  `update_ms`.

## Bring-up and troubleshooting

1. Power the driver from its rated motor supply (12..32 V) and connect its
   ground to the ESP32 ground; the SN65HVD230 runs from 3.3 V.
2. Configure each driver over its UART port before putting it on the bus:
   slave address (`0x60`), CAN bit rate (`0x68`, value in kbps: 500 -> 0x01F4,
   1000 -> 0x03E8), then save (`0x04`).  The factory default bit rate is
   **500 kbps**, so either set the drivers to 1000 or set `can: baud` to
   500000 -- a mismatch produces no acknowledgement on the bus.
3. The controller transmits on extended ID `0x1000`; the PD42S1 accepts IDs in
   `0x1000..0x100F` only.  `can: command_id` is validated against that range.
4. Leave command echo (`0x6F`) at 0 (acknowledge).  With `0x6F = 1` the driver
   stops replying.  At boot the firmware sends `0x6F = 0` once (broadcast then
   each address 1..8); after hot-plugging/powering a driver later, run
   `$CANEchoOn` manually.  Normal motor transactions do not carry 0x6F traffic.
5. Bus wiring: twisted CANH/CANL pair, 120 ohm at the two physical ends only,
   common ground.  The SN65HVD230 `Rs` pin must be tied to GND for high-speed
   mode (or through 10k..100k for slope control).
6. Diagnostics:
   * `$CANScan` -- transceiver D/R loopback test, TWAI state/error counters,
     then version reads at addresses 1..8 at the configured bit rate (extended
     frames, accept-all filter, no 0x6F traffic).  Each address is probed up to
     three times (`after retry` in the log).  The reported
     `TWAI tx_err before -> after` tells whether any node ACKed the frames:
     `0 -> 0` means a powered node is present, an increase means nobody ACKed
     (check power, CANH/CANL wiring and bit rate).
   * `$CANScan=all` -- recovery variant: at each rate it first sends
     `0x6F = 0` (broadcast and every address 1..8), then scans 125k, 250k,
     500k and 1 Mbps; restores the configured bit rate and restarts the CAN
     worker afterwards.
   * `$CANEchoOn` -- send `0x6F = 0` once (broadcast + addresses 1..8) without
     scanning; use it after connecting a new driver.
   * `$CANStatus` -- TWAI state, tx/rx error counters, bus error count,
     queue fill, number of bus-off recoveries, configured pins/baud/ID and the
     cached echo state.
   * `$CANSweep` -- brute-force search for the transceiver D/R pins by pulling
     each candidate GPIO low.  Bench/debug only: it briefly drives pins that
     may be limit switches, STEP/DIR or enable lines.

> Retries and recovery: every protocol transaction (version, position,
> enable/disable, target) is tried twice before it counts as a failure.  A
> failed target/readback is retried on the next worker pass; after three
> consecutive failures the motor is re-synchronised (which also re-enables
> echo on its address).  The CAN worker checks the TWAI state every 500 ms and
> recovers automatically from bus-off.
7. If `$CANScan` reports `transceiver detected` but `(no devices)` and the TWAI
   state is `bus-off`, no other node is acknowledging: check driver power,
   CANH/CANL wiring, termination and the driver's bit rate.
8. Direction: the `pd42s1` profile currently sends direction byte 0 in every
   `0xF2` (absolute position) command.  Before running negative moves, jog a
   single motor a small amount in both directions and confirm the sign
   convention; this is a known open item (see the code review report).

