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

With 80 steps/mm on a 2GT 20T pulley (40 mm/rev) the axis has 3200 steps
per revolution, so `counts_per_step` is 51200 / 3200 = 16.  Set it to 1.0 only
when `steps_per_mm` was chosen so that one FluidNC step equals one driver
position unit.

`speed_rpm` and `accel` are the limits the driver uses while following the
streamed positions; they must be at least the axis's real maximum speed and
acceleration, or the motor will lag behind the commanded position.  The
firmware sends a new absolute position every `update_ms`.

Position targets are streamed **without waiting for the driver's reply**
(fire-and-forget), because waiting for an acknowledgement on every update adds
latency and jitter and makes a loaded axis stutter.  When the target has not
changed for 50 ms, the final target is sent once with an acknowledgement so a
dropped frame cannot leave the axis a few millimetres short.  With four motors
at 500 kbps, `update_ms` of 5-10 ms is the practical range (the CAN worker and
bus saturate below that).  This is still absolute-position streaming, not
step/dir: at very high acceleration/speed the motion is less smooth than a
pulse train; reduce `max_rate`/`acceleration`/driver `accel` if it feels jerky.

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

## Position scaling and resync

Driver position units per axis millimetre:

    counts_per_mm = steps_per_mm * counts_per_step

With the reference config (640 steps/mm, `counts_per_step: 16`) that is
**10240 counts/mm**.  Bench measurement (500 kbps, unloaded): 1 mm gave
~10240 counts, 10 mm gave ~102400 counts and 50 mm gave ~512000 counts.

If a target command is lost (CAN checksum error), the firmware keeps the
*desired* target across the following resync: it records the step-counter
value at the last sync and offsets future targets from it, so the axis does
not adopt the lagging driver position and stop short.  In three ±50 mm test
rounds with intermittent 0xE4 errors the axes still returned to their starting
position within about 0.01 mm.

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
   stops replying.  At boot the firmware enables echo, verifies every
   configured driver and then closes echo again; normal motor sync re-enables
   it when acknowledgments are needed.  After hot-plugging/powering a driver
   later, run `$CANEchoOn` manually.
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
   * `$CANClear` -- release the stall protection (0xF9) on addresses 1..8 and
     report how many drivers acknowledged.  0xF9 does not disable the motor.
     Do NOT use a broadcast 0xFB "clear state": on firmware V1.6 bench testing
     it made all four drivers stop answering until they were power cycled.
     A position command answered with 0xE1..0xE6 automatically tries 0xF9,
     re-enables the motor and forces a fresh sync.
   * `$CANFaults` -- read and display every driver's run status (0x2C:
     stopped / done / running / overload / stall / undervoltage), stall flag
     (0x2D), enable state (0x2F) and in-position flag (0x30); prints a warning
     summary when overload/stall/undervoltage is seen.
   * `$CANAutoConfig` -- probe all driver rates, change the drivers to
     `can: baud` (addressed `0x68` + save `0x04`), confirm every configured
     motor and close echo again.  Also runs automatically at boot.
   * `$CANDriverInfo` -- read 0x32 from every answering driver and show the
     **saved** work mode, echo setting and CAN bit rate (verified after a
     driver reset too).
   * `$CANPos` -- read actual position (0x2A), position error (0x2B), speed
     (0x29) and in-position flag (0x30) for every answering driver.
   * `$CANStop` -- safe stop: broadcast motor disable (`0xFA=1`).  The 0xFC
     immediate-brake command is intentionally not used because its brake state
     cannot be cleared safely on firmware V1.6 (broadcast 0xFB locks all
     drivers until power cycle).
   * `$CANRead` -- raw read, e.g. `$CANRead=1,0x2C` (address,function): sends
     one read command and prints the response bytes in hex.
   * `$CANReset` -- broadcast the driver reset command (0x02), wait 2.5 s and
     re-enable command echo.  Bench recovery only: it stops any motion and is
     the last resort before power-cycling the drivers.
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
8. Direction: the `pd42s1` profile sets the `0xF2` direction byte from the
   sign of the absolute target (0 = forward, 1 = reverse).  The vendor example
   only shows positive targets, so verify with a small +X / -X jog on one
   motor before relying on negative moves.


## CAN bit rate auto-configuration

`can: baud` is both the host rate and the target rate for the drivers.  At
boot the firmware:

1. probes every PD42S1 rate (125k/250k/500k/1 Mbps) with echo enabled,
2. if drivers answer at another rate, broadcasts `0x68` (new rate in kbps) at
   that rate, then sends an addressed `0x68` + `0x04` (save) to every detected
   driver and reads 0x32 back to confirm the saved rate.  If 0x32 already
   shows the target rate, the write is skipped so the driver NVRAM is not
   rewritten on every boot,
3. confirms that every configured `pd42s1` motor answers at the configured
   rate; on a mismatch it warns and leaves echo enabled,
4. on full success disables echo again (motor sync re-enables it before
   streaming).

`$CANAutoConfig` / `M160` runs the same procedure on demand; it suspends the
CAN worker while probing and restarts it afterwards.

**Duplicate addresses**: if two configured `pd42s1` motors use the same
`address`, CAN control is inhibited immediately after startup, the drivers are
sent a broadcast disable and `$CANStatus` reports
`CAN control inhibited: duplicate PD42S1 address`.  Fix the addresses and
restart.  (Two physical drivers set to the same address while the config uses
unique addresses cannot be detected reliably by this protocol.)

## G-code commands (M150..M161)

The CAN subsystem is only initialised when at least one `pd42s1` motor is
configured; without one, the bus/worker are not started and these commands
log `no PD42S1 motor configured`.  The same functions are also available as
`$CAN...` report commands.

| M-code | Action |
| --- | --- |
| M150 | CAN scan (transceiver/TWAI + version read of addresses 1..8) |
| M151 | multi-baud recovery scan (`$CANScan=all`) |
| M152 | bus status (`$CANStatus`) |
| M153 | per-driver status/stall/enable/in-position (`$CANFaults`) |
| M154 | per-driver position/error/speed/in-position (`$CANPos`) |
| M155 | release stall protection (0xF9, per address) |
| M156 | broadcast + addressed `0x6F=0` (enable command echo) |
| M157 | safe stop: broadcast motor disable (0xFA=1; 0xFC brake disabled) |
| M158 | restart drivers (0x02 broadcast, bench recovery only) |
| M159 | raw read; `M159 P<address> Q<function>` e.g. `M159 P1 Q44` (0x2C) |
| M160 | run CAN bit-rate/echo auto-configuration |
| M161 | display saved driver parameters (0x32: mode/echo/CAN baud) |

M150..M159 use the reserved M100..M199 user-code modal group.  They return
`ok` immediately after the CAN operation; do not combine them with motion
words in the same block.
