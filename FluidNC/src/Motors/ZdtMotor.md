# ZDT (Zhang Datou) CAN motor driver

This motor driver speaks the ZDT free CAN protocol for XS-series closed-loop
steppers.  The CAN transport, framing, checksum and protocol selection are in
`Can/Protocols/Private/Zdt/`; this document describes the motion contract.

## Position stream (default)

```yaml
zdt:
  address: 1
  firmware: Emm          # Emm or X
  checksum: 6B           # 6B, XOR or CRC8
  counts_per_rev: 0       # 0 = derive from axis mechanics
  counts_per_step: 1
  invert_direction: false
  update_ms: 20
  speed_rpm: 0            # 0 = derive from axis max feed
  accel: -1               # -1 = derive from axis acceleration
  mode: position_stream
  position_mode: relative_previous
  speed_source: fixed
  speed_margin_pct: 25
```

The drive receives a `0xFD` position command every `update_ms`.  The position
field is a relative increment from the previous commanded target (`raF=0`);
the driver's own trapezoid and following loop handle each small segment.
`mode: position_stream` and `position_mode: relative_previous` are the only
supported values.  Unknown values fail configuration validation instead of
silently changing the position contract.

`firmware: Emm` uses pulse units and RPM; `firmware: X` uses 0.1 degree and
0.1 RPM.  `checksum` must match the drive setting.  `counts_per_rev` and
`counts_per_step` must match the drive microstep/encoder scaling.

### Human-readable mechanics (preferred)

The axis can declare the mechanics and let the firmware derive both the axis
resolution and this driver's step count:

```yaml
axes:
  x:
    full_steps_per_rev: 200   # 1.8 degree motor
    microsteps: 16
    distance_per_rev: 40      # mm per motor revolution
    max_speed_mm_per_min: 3000
  a:
    full_steps_per_rev: 200
    microsteps: 16            # distance defaults to 360 degrees
    max_speed_deg_per_min: 600
```

`counts_per_rev` may be omitted (or set to 0); the driver derives
`full_steps_per_rev * microsteps`.  `steps_per_mm` is also omitted and derived
by the Axis.

The drive's microstep setting is **not changed by default**.  Set
`write_microsteps: true` to explicitly opt in; on the first sync the firmware
reads the drive's 0x42 configuration, and only if its MStep differs does it
send 0x84 with the NVRAM store bit.  This is an Emm-only feature; for `X` the
position command is 0.1 degree and `microsteps` is only the FluidNC step
mapping, so this driver never overwrites X MStep.

`speed_rpm: 0` derives a fixed-stream value from the axis maximum feed with
10% headroom.  `accel: -1` derives the native driver value from
`acceleration_mm_per_sec2`: Emm gets its 0..255 ramp, X gets RPM/s.  Explicit
non-zero values keep the historical behavior.

## Speed sources

### `fixed` (default, historical bytes)

The `speed_rpm` value is sent unchanged on every position command.  This is
the existing smooth streaming behavior and is the default for compatibility.
Validation requires `speed_rpm` to cover the axis maximum feed even in fixed
mode: too small a value leaves an invisible following error because the
position stream is not corrected by encoder readback.  The required value is
`ceil(max_rate * steps_per_mm * counts_per_step / counts_per_rev)`.

### `planner` (experimental, Emm only)

The speed field is derived from the step delta that FluidNC actually executed
since the previous successful command:

```text
rpm = ceil(|counts| * 60000 / (counts_per_rev * elapsed_ms))
rpm = ceil(rpm * (1 + speed_margin_pct / 100))
speed = clamp(rpm, 1, speed_rpm)
```

This makes the `speed` field follow G-code feed/override, homing and jog ramps
while keeping the relative position stream unchanged.  `speed_rpm` is an upper
clamp, not a fixed command.  `speed_margin_pct` (default 25) restores catch-up
headroom; without it the drive would be asked to run exactly at the average
stream rate and could accumulate invisible following error.  Validation
requires `speed_rpm` to cover the axis maximum feed *after* applying this
margin; otherwise the clamp would silently remove the headroom at maximum
feed.  At the 1 RPM command resolution, a 1 RPM stream becomes 2 RPM with the
default margin.

`planner` is intentionally opt-in and currently limited to `firmware: Emm`.
Validation requires `speed_rpm` to cover the axis maximum feed plus margin:

```text
required_rpm = ceil(max_rate_mm_per_min * steps_per_mm * counts_per_step / counts_per_rev)
required_with_margin = ceil(required_rpm * (100 + speed_margin_pct) / 100)
speed_rpm >= required_with_margin
```

The driver never reads the ZDT's actual position for closed-loop correction;
`planner` follows the step stream, not a measured velocity.  Do not treat
`M400` or program end as proof that the physical axis has settled.

### Derived acceleration

When `accel` is `-1`, the firmware converts axis acceleration to the driver's
native unit:

```text
rpm/s = acceleration_mm_per_sec2 * steps_per_mm * counts_per_step
        / counts_per_rev * 60
Emm:  acc = 256 - 20000 / rpm_per_s  (clamped 1..255)
X:    acc = rpm/s                    (clamped 1..65535)
```

The Emm `acc: 0` "no curve" mode remains available only by setting it
explicitly.  A drive-side `MStep` change is never performed while motion is
enabled; configuration is checked before the first enable.

## Required drive-side prerequisites

- FWType, Checksum, CAN baud and ID must match `firmware`, `checksum`,
  `can.baud` and `address`.
- When `write_microsteps: true`, the drive must accept 0x84 and its NVRAM
  write.  The config is read first; an unchanged value is not rewritten.
  If the read/write fails, motion stays gated and the driver is not reset;
  fix power/wiring/parameter lock, then reboot or reset.  Back up the drive's
  MStep before enabling this option.
- `Response` must be `Receive` or `Both`; `None`/`Reached`/`Other` do not
  provide the command ACK expected by this streaming driver and can trigger
  HardStop after repeated missing replies.
- Emm `S_Vel_IS` must remain disabled (factory default); enabling it changes
  the speed unit to 0.1 RPM and this driver does not compensate.
- The X firmware `S_PosTDP` option must remain disabled; this driver sends 0.1 degree units and does not compensate for the 10x position scaling.
- Stall protection and heartbeat should be reviewed for the intended motion;
  the current driver sends no position frame while stationary (`stepDelta==0`).
- Use a properly terminated CAN bus and keep 500 kbps as the conservative
  default; 1 Mbps requires all drives and wiring to be verified.

## Safety and rollback

Back up `config.yaml` before enabling `planner`.  Use `speed_source: fixed` for production until `planner` has been validated
on the actual machine.  `planner` must first be tested single-axis, unloaded,
with a very small move and a conservative `speed_rpm`; do not combine it with
lasers, pumps or other actuators.

To roll back behavior, change only:

```yaml
speed_source: fixed
```

To roll back automatically derived values, restore explicit
`counts_per_rev`, `speed_rpm` and `accel` values.  Older firmware may reject
`speed_rpm: 0` and `accel: -1`; restore the old config backup before flashing
an older firmware.  A full reset/reload is required because a soft reset does
not re-read `config.yaml`.  If `write_microsteps: true` was used, the drive's
NVRAM MStep was changed and must be restored with the vendor tool or 0x84
before running the old configuration.
