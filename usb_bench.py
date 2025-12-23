#!/usr/bin/env python3
import argparse
import csv
import os
import random
import struct
import sys
import threading
import time
import zlib
from collections import OrderedDict

import serial

SYNC_WORD = b"\xA5\x5A\xC3\x3C"
HEADER_FORMAT = ">IHH"  # seq (uint32), payload_len (uint16), flags (uint16)
HEADER_LEN = struct.calcsize(HEADER_FORMAT)
CRC_FORMAT = ">I"
CRC_LEN = struct.calcsize(CRC_FORMAT)

DEFAULT_PORT_PAIRS = [
    ("/dev/ch9121_B2", "/dev/ch9121_A2"),
    ("/dev/ch9121_A1", "/dev/ch9121_B1"),
    ("/dev/ttyUSB2", "/dev/ttyUSB3"),
    ("/dev/ttyUSB4", "/dev/ttyUSB5"),
]


def _randbytes(rng, n):
    # Python 3.8 compatibility: random.Random.randbytes is 3.9+
    return bytes(rng.getrandbits(8) for _ in range(n))


def pick_default_ports():
    for port1, port2 in DEFAULT_PORT_PAIRS:
        if os.path.exists(port1) and os.path.exists(port2):
            return port1, port2
    return DEFAULT_PORT_PAIRS[0]


class Stats:
    def __init__(self, max_inflight):
        self.lock = threading.Lock()
        self.tx_packets = 0
        self.tx_bytes = 0
        self.rx_packets = 0
        self.rx_bytes = 0
        self.crc_fail = 0
        self.resync = 0
        self.gaps = 0
        self.dupes = 0
        self.last_seq = None
        self.send_times = OrderedDict()
        self.max_inflight = max_inflight
        self.rtt_sum = 0.0
        self.rtt_count = 0
        self.rtt_max = 0.0
        self.rtt_samples = []
        self.rtt_missed = 0
        self.last_rx_time = None

    def record_tx(self, seq, payload_len, send_time):
        with self.lock:
            self.tx_packets += 1
            self.tx_bytes += payload_len
            self.send_times[seq] = send_time
            if len(self.send_times) > self.max_inflight:
                self.send_times.popitem(last=False)
                self.rtt_missed += 1

    def record_rx(self, seq, payload_len, recv_time):
        with self.lock:
            self.rx_packets += 1
            self.rx_bytes += payload_len
            self.last_rx_time = recv_time
            if self.last_seq is None:
                self.last_seq = seq
            else:
                if seq == self.last_seq:
                    self.dupes += 1
                elif seq < self.last_seq:
                    self.dupes += 1
                elif seq > self.last_seq + 1:
                    self.gaps += seq - self.last_seq - 1
                    self.last_seq = seq
                else:
                    self.last_seq = seq
            if seq in self.send_times:
                sent = self.send_times.pop(seq)
                rtt = recv_time - sent
                self.rtt_sum += rtt
                self.rtt_count += 1
                if rtt > self.rtt_max:
                    self.rtt_max = rtt
                self.rtt_samples.append(rtt)
            else:
                self.rtt_missed += 1

    def record_crc_fail(self):
        with self.lock:
            self.crc_fail += 1

    def record_resync(self):
        with self.lock:
            self.resync += 1

    def snapshot(self):
        with self.lock:
            return {
                "tx_packets": self.tx_packets,
                "tx_bytes": self.tx_bytes,
                "rx_packets": self.rx_packets,
                "rx_bytes": self.rx_bytes,
                "crc_fail": self.crc_fail,
                "resync": self.resync,
                "gaps": self.gaps,
                "dupes": self.dupes,
                "rtt_sum": self.rtt_sum,
                "rtt_count": self.rtt_count,
                "rtt_max": self.rtt_max,
                "rtt_missed": self.rtt_missed,
                "last_rx_time": self.last_rx_time,
            }

    def get_rtt_samples(self):
        with self.lock:
            return list(self.rtt_samples)


class PacketParser:
    def __init__(self, stats, max_payload):
        self.stats = stats
        self.max_payload = max_payload
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        self._parse()

    def _parse(self):
        while True:
            if len(self.buffer) < len(SYNC_WORD):
                return
            idx = self.buffer.find(SYNC_WORD)
            if idx == -1:
                if len(self.buffer) > len(SYNC_WORD) - 1:
                    self.stats.record_resync()
                    del self.buffer[: -(len(SYNC_WORD) - 1)]
                return
            if idx > 0:
                self.stats.record_resync()
                del self.buffer[:idx]
            if len(self.buffer) < len(SYNC_WORD) + HEADER_LEN:
                return
            header_start = len(SYNC_WORD)
            header_end = header_start + HEADER_LEN
            header = self.buffer[header_start:header_end]
            seq, payload_len, _flags = struct.unpack(HEADER_FORMAT, header)
            if payload_len > self.max_payload:
                self.stats.record_resync()
                del self.buffer[1:]
                continue
            total_len = len(SYNC_WORD) + HEADER_LEN + payload_len + CRC_LEN
            if len(self.buffer) < total_len:
                return
            payload_start = header_end
            payload_end = payload_start + payload_len
            payload = self.buffer[payload_start:payload_end]
            crc_bytes = self.buffer[payload_end:total_len]
            (rx_crc,) = struct.unpack(CRC_FORMAT, crc_bytes)
            calc_crc = zlib.crc32(header + payload) & 0xFFFFFFFF
            if rx_crc != calc_crc:
                self.stats.record_crc_fail()
                self.stats.record_resync()
                del self.buffer[1:]
                continue
            del self.buffer[:total_len]
            self.stats.record_rx(seq, payload_len, time.monotonic())


def build_packet(seq, payload):
    header = struct.pack(HEADER_FORMAT, seq, len(payload), 0)
    crc = zlib.crc32(header + payload) & 0xFFFFFFFF
    return SYNC_WORD + header + payload + struct.pack(CRC_FORMAT, crc)


def tx_loop(ser, stats, args, stop_event):
    seq = 0
    tx_packets = 0
    tx_bytes = 0
    start_time = time.monotonic()
    rng = random.Random(args.seed) if args.random_payload else None
    static_payload = None
    if not args.random_payload:
        static_payload = bytes((i & 0xFF) for i in range(args.payload_bytes))

    while not stop_event.is_set():
        now = time.monotonic()
        if args.duration_s is not None and now - start_time >= args.duration_s:
            break
        if args.total_bytes is not None and tx_bytes >= args.total_bytes:
            break
        if args.packets is not None and tx_packets >= args.packets:
            break

        if args.random_payload:
            payload = _randbytes(rng, args.payload_bytes)
        else:
            payload = static_payload

        pkt = build_packet(seq, payload)
        try:
            ser.write(pkt)
        except serial.SerialTimeoutException:
            continue

        stats.record_tx(seq, args.payload_bytes, time.monotonic())
        tx_packets += 1
        tx_bytes += args.payload_bytes
        seq = (seq + 1) & 0xFFFFFFFF

        if args.dwell_us > 0:
            time.sleep(args.dwell_us / 1_000_000.0)


def rx_loop(ser, parser, stop_event):
    while not stop_event.is_set():
        data = ser.read(4096)
        if data:
            parser.feed(data)


def percentile(sorted_values, pct):
    if not sorted_values:
        return None
    k = int(round((len(sorted_values) - 1) * pct))
    return sorted_values[k]


def progress_loop(stats, start_time, interval_s, stop_event, csv_writer):
    last_time = start_time
    last_snap = stats.snapshot()
    while not stop_event.wait(interval_s):
        now = time.monotonic()
        snap = stats.snapshot()
        dt = now - last_time
        delta_bytes = snap["rx_bytes"] - last_snap["rx_bytes"]
        delta_pkts = snap["rx_packets"] - last_snap["rx_packets"]
        delta_rtt_sum = snap["rtt_sum"] - last_snap["rtt_sum"]
        delta_rtt_count = snap["rtt_count"] - last_snap["rtt_count"]
        throughput_mbps = (delta_bytes * 8.0 / dt / 1_000_000.0) if dt > 0 else 0.0
        pkt_rate = (delta_pkts / dt) if dt > 0 else 0.0
        avg_rtt_ms = (
            (delta_rtt_sum / delta_rtt_count) * 1000.0 if delta_rtt_count > 0 else 0.0
        )

        print(
            f"{now - start_time:7.1f}s "
            f"rx {snap['rx_packets']} pkts "
            f"{snap['rx_bytes']} bytes "
            f"{throughput_mbps:7.2f} Mbps "
            f"{pkt_rate:7.1f} pkt/s "
            f"crc {snap['crc_fail']} "
            f"gaps {snap['gaps']} "
            f"dupes {snap['dupes']} "
            f"resync {snap['resync']} "
            f"rtt_avg {avg_rtt_ms:6.2f} ms",
            flush=True,
        )

        if csv_writer:
            csv_writer.writerow(
                [
                    f"{now - start_time:.3f}",
                    f"{dt:.3f}",
                    delta_bytes,
                    delta_pkts,
                    f"{throughput_mbps:.3f}",
                    f"{pkt_rate:.3f}",
                    snap["rx_bytes"],
                    snap["rx_packets"],
                    snap["crc_fail"],
                    snap["gaps"],
                    snap["dupes"],
                    snap["resync"],
                    f"{avg_rtt_ms:.3f}",
                ]
            )
        last_time = now
        last_snap = snap


def parse_args():
    default_port1, default_port2 = pick_default_ports()
    parser = argparse.ArgumentParser(description="USB serial throughput/latency benchmark")
    parser.add_argument(
        "--port1",
        default=default_port1,
        help=f"First serial port (default: {default_port1})",
    )
    parser.add_argument(
        "--port2",
        default=default_port2,
        help=f"Second serial port (default: {default_port2})",
    )
    parser.add_argument("--direction", choices=["A", "B"], default="A")
    parser.add_argument("--port_tx", help="Explicit TX port (overrides --direction)")
    parser.add_argument("--port_rx", help="Explicit RX port (overrides --direction)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--payload-bytes", type=int, default=256)
    parser.add_argument("--duration-s", type=float)
    parser.add_argument("--total-bytes", type=int)
    parser.add_argument("--packets", type=int)
    parser.add_argument("--dwell-us", type=int, default=0)
    parser.add_argument("--random-payload", action="store_true")
    parser.add_argument("--seed", type=int)
    parser.add_argument("--timeout-ms", type=int, default=100)
    parser.add_argument("--csv", dest="csv_path")
    parser.add_argument("--progress-interval-s", type=float, default=1.0)
    return parser.parse_args()


def main():
    args = parse_args()

    if args.port_tx and args.port_rx:
        port_tx = args.port_tx
        port_rx = args.port_rx
    else:
        if not args.port1 or not args.port2:
            print("error: use --port_tx/--port_rx or --port1/--port2", file=sys.stderr)
            return 2
        if args.direction == "A":
            port_tx = args.port1
            port_rx = args.port2
        else:
            port_tx = args.port2
            port_rx = args.port1

    if args.duration_s is None and args.total_bytes is None and args.packets is None:
        args.duration_s = 10.0

    if args.payload_bytes < 1 or args.payload_bytes > 65535:
        print("error: --payload-bytes must be 1..65535", file=sys.stderr)
        return 2

    timeout_s = args.timeout_ms / 1000.0
    max_inflight = max(1000, int(2_000_000 / max(1, args.payload_bytes)))

    try:
        ser_tx = serial.Serial(port_tx, args.baud, timeout=timeout_s, write_timeout=timeout_s)
        ser_rx = serial.Serial(port_rx, args.baud, timeout=timeout_s, write_timeout=timeout_s)
    except serial.SerialException as exc:
        print(f"error opening serial ports: {exc}", file=sys.stderr)
        return 2

    ser_tx.reset_input_buffer()
    ser_tx.reset_output_buffer()
    ser_rx.reset_input_buffer()
    ser_rx.reset_output_buffer()

    stats = Stats(max_inflight=max_inflight)
    parser = PacketParser(stats, args.payload_bytes)
    stop_event = threading.Event()

    csv_file = None
    csv_writer = None
    if args.csv_path:
        csv_file = open(args.csv_path, "w", newline="")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(
            [
                "time_s",
                "interval_s",
                "interval_rx_bytes",
                "interval_rx_packets",
                "throughput_mbps",
                "pkt_rate",
                "total_rx_bytes",
                "total_rx_packets",
                "crc_fail",
                "gaps",
                "dupes",
                "resync",
                "avg_rtt_ms",
            ]
        )

    start_time = time.monotonic()

    rx_thread = threading.Thread(target=rx_loop, args=(ser_rx, parser, stop_event), daemon=True)
    tx_thread = threading.Thread(target=tx_loop, args=(ser_tx, stats, args, stop_event), daemon=True)
    progress_thread = threading.Thread(
        target=progress_loop,
        args=(stats, start_time, args.progress_interval_s, stop_event, csv_writer),
        daemon=True,
    )

    rx_thread.start()
    tx_thread.start()
    progress_thread.start()

    try:
        tx_thread.join()
        drain_s = max(1.0, timeout_s * 2.0)
        drain_end = time.monotonic() + drain_s
        while time.monotonic() < drain_end:
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        rx_thread.join()
        progress_thread.join()
        ser_tx.close()
        ser_rx.close()
        if csv_file:
            csv_file.close()

    end_time = time.monotonic()
    snap = stats.snapshot()
    duration = end_time - start_time
    if snap["last_rx_time"]:
        duration = max(0.001, snap["last_rx_time"] - start_time)

    throughput_mbps = (snap["rx_bytes"] * 8.0 / duration / 1_000_000.0) if duration > 0 else 0.0
    pkt_rate = (snap["rx_packets"] / duration) if duration > 0 else 0.0
    rtt_samples = stats.get_rtt_samples()
    rtt_samples.sort()
    rtt_avg_ms = (snap["rtt_sum"] / snap["rtt_count"] * 1000.0) if snap["rtt_count"] else 0.0
    rtt_p50_ms = (percentile(rtt_samples, 0.50) or 0.0) * 1000.0
    rtt_p95_ms = (percentile(rtt_samples, 0.95) or 0.0) * 1000.0
    rtt_max_ms = snap["rtt_max"] * 1000.0

    print("\nFinal Summary")
    print(f"Duration (s): {duration:.3f}")
    print(f"Good payload bytes: {snap['rx_bytes']}")
    print(f"Good packets: {snap['rx_packets']}")
    print(f"Throughput (Mbps): {throughput_mbps:.3f}")
    print(f"Packet rate (pkt/s): {pkt_rate:.3f}")
    print(
        "RTT ms: avg {0:.3f}  p50 {1:.3f}  p95 {2:.3f}  max {3:.3f}  "
        "samples {4}".format(
            rtt_avg_ms, rtt_p50_ms, rtt_p95_ms, rtt_max_ms, snap["rtt_count"]
        )
    )
    print(
        "Errors: crc_fail {0}  gaps {1}  dupes {2}  resync {3}  rtt_missed {4}".format(
            snap["crc_fail"], snap["gaps"], snap["dupes"], snap["resync"], snap["rtt_missed"]
        )
    )


if __name__ == "__main__":
    sys.exit(main() or 0)
