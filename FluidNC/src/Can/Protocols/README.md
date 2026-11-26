# CAN protocol headers

This directory is the single source of truth for vendor/protocol details used
by the CAN motor drivers.  Driver classes own policy, transactions and motion
integration; they must not duplicate protocol numbers, limits or error values.

## Required structure

Copy [PROTOCOL_TEMPLATE.h](PROTOCOL_TEMPLATE.h) and keep its metadata block at
the top of every protocol header:

```text
Vendor
Driver family
Protocol name / revision
Supported firmware model
Hardware variants
Transport
Source documents
Verification status
Notes
```

Then organize the remainder by function, not by numeric order:

```text
1. Framing and identifiers
2. Addressing and identifiers
3. Command / function codes and response lengths
4. Parameter limits and defaults
5. Status, errors and state machine
6. Timing, retries and buffer sizes
7. Builders
8. Parsers
9. Safety notes
```

## Rules

- Protocol headers are transport-independent.  No TWAI, UART, GPIO, RTOS or
  machine state belongs here.
- Every configurable protocol field has a named range in `Limits` and a default
  in `Defaults`; numeric bounds in a driver are a review defect.
- Builders document argument ranges, output length and buffer requirements.
- Parsers document exact response shape, validation and failure conditions.
- Write/erase/NVRAM/address/baud changes are safety-relevant and are not used by
  read-only diagnostics.
- If a vendor document revision changes, update the metadata and the constants
  in one commit.

## Current protocols

| Header | Transport | Vendor / family | Status |
|---|---|---|---|
| `Private/Zdt/ZdtProtocol.h` | CAN extended, split frames | ZDT XS series, Emm/X firmware | implemented, hardware exercised |
| `Private/Pd42s1/Pd42s1Protocol.h` | CAN extended, 0xC5/0x5C byte stream | ALIENTEK PD42S1 | implemented, hardware exercised |
| `Standard/Canopen/CanopenProtocol.h` | CAN standard | CiA301 / CiA402 | standard implementation, not hardware-verified |
