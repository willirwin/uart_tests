# uart_tests

Serial/USB throughput and integrity tests for two devices wired in a loop
(host -> device A -> device B -> host). This repo includes a robust packet
benchmark (`usb_bench.py`) plus several older test scripts used to tune
dwell times and validate raw byte streams.

## Prerequisites

- Linux (Ubuntu/Debian family recommended).
- Python 3.8+.
- `pyserial` installed: `pip install pyserial`
- Two serial devices visible as `/dev/...` (examples below).

## Common port names used here

The scripts in `ch9121_tests/` use device names like:

- `/dev/ch9121_A1`, `/dev/ch9121_A2`
- `/dev/ch9121_B1`, `/dev/ch9121_B2`
- `/dev/ttyUSB2`, `/dev/ttyUSB3`
- `/dev/ttyUSB4`, `/dev/ttyUSB5`

`usb_bench.py` will auto-pick the first pair that exists on the system from
that list. You can always override with `--port1` and `--port2`.

## usb_bench.py (recommended)

This is the main, protocol-based benchmark. It sends framed packets from
one port and parses them back on the other port, verifying CRC32, tracking
sequence gaps/dupes, and reporting throughput/latency.

### Protocol framing

Each packet:

- Sync word: 4 bytes (`0xA5 0x5A 0xC3 0x3C`)
- Header: sequence (u32), payload length (u16), flags (u16)
- Payload: configurable size
- CRC32: over header + payload

The receiver scans for the sync word, validates header and CRC, and resyncs
on errors. This lets you detect corruption, loss, duplicates, and drift.

### Basic usage

Mode A (host -> port1 -> port2 -> host):

```
python usb_bench.py --direction A
```

Mode B (host -> port2 -> port1 -> host):

```
python usb_bench.py --direction B
```

Explicit ports:

```
python usb_bench.py --port1 /dev/ttyUSB0 --port2 /dev/ttyUSB1 --direction A
```

### Useful options

- `--baud` (default 115200)
- `--payload-bytes` (1..65535)
- One of:
  - `--duration-s`
  - `--total-bytes`
  - `--packets`
- `--dwell-us` (inter-packet delay; use to find max stable rate)
- `--random-payload` and `--seed`
- `--timeout-ms`
- `--progress-interval-s`
- `--csv out.csv`

### Output and metrics

The script prints periodic progress and a final summary:

- Throughput (Mbps) based on verified payload bytes
- Packet rate (pkt/s)
- RTT stats (avg, p50, p95, max)
- Error stats (CRC fails, gaps, duplicates, resync count)

## ch9121_tests/ scripts (legacy/tuning)

These scripts are earlier tests that send raw byte streams and compare the
data received. They are useful for dwell tuning and basic sanity checks.

- `ch9121_tests/ch9121_uart_test.py`:
  - One-way and bidirectional tests using random data.
  - Configurable total bytes, chunk size, and dwell.
- `ch9121_tests/soniceclipse_uart_test.py`:
  - Similar to `ch9121_uart_test.py`, includes start/stop markers.
- `ch9121_tests/uart_speed_test.py`:
  - Sweep dwell times to find a stable rate at higher baud.
- `ch9121_tests/soniceclipse_speed_test.py`:
  - Similar sweep logic at 115200 baud.
- `ch9121_tests/encrypt_test.py`:
  - Repeats raw data tests across multiple port pairs.

## Typical workflow

1. Start with `usb_bench.py` to get accurate throughput and error stats.
2. If you need to tune dwell/chunk sizes, use `uart_speed_test.py` or
   `soniceclipse_speed_test.py` to find stable dwell values.
3. Re-run `usb_bench.py` with the tuned dwell to verify integrity at max
   throughput.

## Notes

- Ensure the two devices are wired/firmware-configured so data written to
  one port eventually returns on the other.
- If ports appear as `/dev/ttyACM*`, pass them via `--port1` and `--port2`.
