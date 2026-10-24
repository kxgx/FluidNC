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
