import serial
import threading
import time
import os
import argparse
from tqdm import tqdm
import sys

# USAGE
# Open terminal, navigate to this Python file's directory, and input:
#   python3 ch0121_uart_test.py -h
#   → Lists all command line options.

# EXAMPLE
#   python3 ch0121_uart_test.py --port-a /dev/ttyACM2 --port-b /dev/ttyACM4 --baudrate 115200 ...
#   ...   --total-bytes 1024 --chunk-size 8 --dwell-sec 0.005 --direction both --repeat 5
#   → Runs bidirectional test 5 times using 8-byte chunks and 5ms delay between writes.

# Thread function for reading incoming serial data
def reader_thread(ser, storage, stop_event, transmit_done_event):
    try:
        while not stop_event.is_set():
            data = ser.read(1024)
            if data:
                storage.extend(data)  # Accumulate received bytes
            elif transmit_done_event.is_set():
                # If sender finished and no more data is coming, wait briefly then exit
                time.sleep(0.1)
                break
            else:
                time.sleep(0.0005)  # Light polling delay
    except serial.SerialException as e:
        tqdm.write(f"[Reader] SerialException: {e}")

# Transmit random data in chunks, simulating traffic load
def send_data(ser, total_bytes, chunk_size, dwell_sec, progress_desc, position=0):
    sent_bytes = 0
    full_payload = bytearray()

    with tqdm(
        total=total_bytes,
        desc=progress_desc,
        position=position,
        unit='B',
        unit_scale=True,
        unit_divisor=1024,
        miniters=10,
        mininterval=0.5,
        leave=True
    ) as pbar:
        while sent_bytes < total_bytes:
            remaining = total_bytes - sent_bytes
            this_chunk = min(chunk_size, remaining)
            chunk = os.urandom(this_chunk)  # Random data
            written = ser.write(chunk)
            if written != len(chunk):
                tqdm.write(f"[{progress_desc}] Write short! Expected {len(chunk)}, wrote {written}")
            ser.flush()
            full_payload.extend(chunk)
            sent_bytes += this_chunk
            pbar.update(this_chunk)
            time.sleep(dwell_sec)  # Delay between chunks

    return full_payload

# Run a one-way data transmission test and validate correctness
def run_oneway_test(sender_port, receiver_port, baudrate, total_bytes, chunk_size, dwell_sec, label="", position=0):
    try:
        # Open serial connections
        ser_tx = serial.Serial(sender_port, baudrate=baudrate, timeout=5, write_timeout=5)
        ser_rx = serial.Serial(receiver_port, baudrate=baudrate, timeout=5)

        received_data = bytearray()
        ser_rx.reset_input_buffer()
        time.sleep(0.05)

        # Events to coordinate threading
        stop_event = threading.Event()
        transmit_done_event = threading.Event()

        # Launch receiver thread
        reader = threading.Thread(target=reader_thread, args=(ser_rx, received_data, stop_event, transmit_done_event))
        reader.start()

        # Transmit random data
        sent_payload = send_data(ser_tx, total_bytes, chunk_size, dwell_sec, f"{label}", position=position)

        # Signal transmission complete
        transmit_done_event.set()

        # Wait for reader to capture remaining data
        reader.join(timeout=2)
        stop_event.set()
        reader.join()

        # Check data integrity
        exact_match = (received_data == sent_payload)
        tqdm.write(f"\n[{label}] Sent     : {len(sent_payload)} bytes")
        tqdm.write(f"[{label}] Received : {len(received_data)} bytes")

        if exact_match:
            tqdm.write(f"[{label}] All data received correctly.")
        else:
            tqdm.write(f"[{label}] Data mismatch detected.")
            if len(received_data) != len(sent_payload):
                tqdm.write(f"[{label}] Size mismatch: expected {len(sent_payload)}, got {len(received_data)} |")
                tqdm.write(f"{len(sent_payload)-len(received_data)} Bytes missing.")
            else:
                # Byte-by-byte comparison to find mismatch region
                min_len = min(len(sent_payload), len(received_data))
                mismatch_start = None
                mismatch_end = None
                realign_threshold = 16  # Require N correct bytes to consider realignment
                match_count = 0

                for i in range(min_len):
                    if sent_payload[i] != received_data[i]:
                        if mismatch_start is None:
                            mismatch_start = i
                        match_count = 0
                    else:
                        if mismatch_start is not None:
                            match_count += 1
                            if match_count >= realign_threshold:
                                mismatch_end = i - realign_threshold
                                break

                if mismatch_start is not None:
                    if mismatch_end is None:
                        tqdm.write(f"[{label}] Data mismatch starting at byte {mismatch_start} and continues to end.")
                    else:
                        tqdm.write(f"[{label}] Data mismatch from byte {mismatch_start} to {mismatch_end}.")

        # Close serial connections
        ser_tx.close()
        ser_rx.close()

        return exact_match

    except serial.SerialException as e:
        tqdm.write(f"[{label}] SerialException: {e}")
        return False

# Runs both directions concurrently and collects results
def run_bidirectional_test(port_a, port_b, baudrate, total_bytes, chunk_size, dwell_sec):
    results = {"A ➝ B": 0, "B ➝ A": 0}

    def test_a2b():
        if run_oneway_test(port_a, port_b, baudrate, total_bytes, chunk_size, dwell_sec, "A ➝ B", 0):
            results["A ➝ B"] += 1

    def test_b2a():
        if run_oneway_test(port_b, port_a, baudrate, total_bytes, chunk_size, dwell_sec, "B ➝ A", 1):
            results["B ➝ A"] += 1

    # Run both directions in parallel threads
    t1 = threading.Thread(target=test_a2b)
    t2 = threading.Thread(target=test_b2a)
    t1.start()
    t2.start()
    t1.join()
    t2.join()

    return results

# Parses CLI arguments for test configuration
def parse_args():

    # Default device paths for this test setup (varies per machine)
    port_a1 = "/dev/ch9121_A1"
    port_a2 = "/dev/ch9121_A2"
    port_b1 = "/dev/ch9121_B1"
    port_b2 = "/dev/ch9121_B2"

    # Suggested working defaults
    t_size = 1024 * 128          # Default: 128 kB total transmission size
    c_size = 8                 # Default: 8-byte chunk size
    dwell = 0.0007               # Default: .5 ms pause between chunks
                                # Default configuration results in ~1.4 kB/s (6/18/2025)

    parser = argparse.ArgumentParser(description="CH9121 UART <-> ETH Communication Tester")
    parser.add_argument('--port-a', type=str, default=port_a1, help='First serial port (e.g., /dev/ttyACM2)')
    parser.add_argument('--port-b', type=str, default=port_b1, help='Second serial port (e.g., /dev/ttyACM4)')
    parser.add_argument('--baudrate', type=int, default=115200, help='Baud rate (e.g., 9600, 115200)')
    parser.add_argument('--total-bytes', type=int, default=t_size, help='Total bytes to send per direction')
    parser.add_argument('--chunk-size', type=int, default=c_size, help='Bytes per transmission chunk')
    parser.add_argument('--dwell-sec', type=float, default=dwell, help='Delay between chunk writes (in seconds)')
    parser.add_argument('--direction', type=str, choices=['a2b', 'b2a', 'both'], default='both', help='Test direction')
    parser.add_argument('--repeat', type=int, default=1, help='Number of times to repeat the test')
    return parser.parse_args()

# Entry point
if __name__ == "__main__":
    args = parse_args()
    summary = {"A ➝ B": 0, "B ➝ A": 0}

    for i in range(args.repeat):
        tqdm.write(f"\n\n=================== Test Iteration {i + 1}/{args.repeat} ===================\n")
        if args.direction == "a2b":
            if run_oneway_test(args.port_a, args.port_b, args.baudrate, args.total_bytes, args.chunk_size, args.dwell_sec, "A ➝ B", 0):
                summary["A ➝ B"] += 1
        elif args.direction == "b2a":
            if run_oneway_test(args.port_b, args.port_a, args.baudrate, args.total_bytes, args.chunk_size, args.dwell_sec, "B ➝ A", 0):
                summary["B ➝ A"] += 1
        elif args.direction == "both":
            result = run_bidirectional_test(args.port_a, args.port_b, args.baudrate, args.total_bytes, args.chunk_size, args.dwell_sec)
            summary["A ➝ B"] += result["A ➝ B"]
            summary["B ➝ A"] += result["B ➝ A"]

    # Print test summary
    tqdm.write("\n=================== Summary ===================")
    if args.direction in ["a2b", "both"]:
        tqdm.write(f"A ➝ B Successes: {summary['A ➝ B']} / {args.repeat}")
    if args.direction in ["b2a", "both"]:
        tqdm.write(f"B ➝ A Successes: {summary['B ➝ A']} / {args.repeat}")
