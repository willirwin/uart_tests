import serial
import threading
import time
import os
import argparse
from tqdm import tqdm

START_MSG = b"<<BEGIN>>"
STOP_MSG = b"<<END>>"

def read_from_uart(ser, buffer, stop_event):
    while not stop_event.is_set():
        try:
            if ser.in_waiting:
                buffer.extend(ser.read(ser.in_waiting))
            else:
                time.sleep(0.001)
        except Exception as e:
            print(f"[!] Receiver thread error: {e}")
            break

def send_uart_data(ser, total_bytes, chunk_size, dwell_sec, pbar):
    sent = bytearray()
    bytes_sent = 0

    while bytes_sent < total_bytes:
        chunk = os.urandom(min(chunk_size, total_bytes - bytes_sent))
        try:
            ser.write(chunk)
            ser.flush()
        except Exception as e:
            print(f"[!] Error during write: {e}")
            break

        sent.extend(chunk)
        bytes_sent += len(chunk)

        pbar.update(len(chunk))
        time.sleep(dwell_sec)

    return sent

def run_test(send_port, recv_port, baudrate, total_bytes, chunk_size, dwell, iteration):
    print(f"\n========== Test Iteration {iteration} ==========")
    print(f"[INFO] Sending on {send_port}, Receiving on {recv_port}")

    recv_buffer = bytearray()
    stop_event = threading.Event()

    try:
        recv_ser = serial.Serial(recv_port, baudrate, timeout=0)
        recv_thread = threading.Thread(target=read_from_uart, args=(recv_ser, recv_buffer, stop_event))
        recv_thread.start()
    except Exception as e:
        print(f"[ERROR] Could not open receiving port: {e}")
        return False

    try:
        send_ser = serial.Serial(send_port, baudrate, timeout=1)
    except Exception as e:
        print(f"[ERROR] Could not open sending port: {e}")
        stop_event.set()
        recv_thread.join()
        recv_ser.close()
        return False

    time.sleep(1)

    try:
        send_ser.write(START_MSG)
        send_ser.flush()
        time.sleep(0.1)

        with tqdm(total=total_bytes, desc="Transmitting", unit='B', unit_scale=True, unit_divisor=1024, miniters=10, mininterval=0.5, leave=True) as pbar:
            sent_data = send_uart_data(send_ser, total_bytes, chunk_size, dwell, pbar)

        send_ser.write(STOP_MSG)
        send_ser.flush()
        send_ser.close()

        time.sleep(2)
        stop_event.set()
        recv_thread.join()
        recv_ser.close()

        try:
            start_index = recv_buffer.index(START_MSG) + len(START_MSG)
            end_index = recv_buffer.index(STOP_MSG)
            received_data = recv_buffer[start_index:end_index]
        except ValueError:
            print("[ERROR] START or STOP message not found in received data.")
            return False

        print(f"Sent     : {len(sent_data)} bytes")
        print(f"Received : {len(received_data)} bytes")

        if sent_data == received_data:
            print("Data integrity verified.")
            return True
        else:
            mismatch = sum(1 for a, b in zip(sent_data, received_data) if a != b)
            print(f"Data mismatch: {mismatch} bytes differ.")
            return False

    except Exception as e:
        print(f"[ERROR] Unexpected exception during test: {e}")
        stop_event.set()
        recv_thread.join()
        recv_ser.close()
        return False

def main():
    # Default port names
    default_a1 = "/dev/pts/6"
    default_a2 = "/dev/pts/4"
    default_b1 = "/dev/ch9121_B1"
    default_b2 = "/dev/ch9121_B2"

    default_d_size = 1024 * 128
    default_c_size = 4
    default_dwell = 0.0004

    parser = argparse.ArgumentParser(description="UART Communication Tester")
    parser.add_argument("--a1", type=str, default=default_a1, help="Port A1 (default: /dev/ch9121_A1)")
    parser.add_argument("--a2", type=str, default=default_a2, help="Port A2 (default: /dev/ch9121_A2)")
    parser.add_argument("--b1", type=str, default=default_b1, help="Port B1 (default: /dev/ch9121_B1)")
    parser.add_argument("--b2", type=str, default=default_b2, help="Port B2 (default: /dev/ch9121_B2)")
    parser.add_argument("--baudrate", type=int, default=115200, help="Baudrate")
    parser.add_argument("--total-bytes", type=int, default=default_d_size, help="Total bytes to send")
    parser.add_argument("--chunk-size", type=int, default=default_c_size, help="Bytes per chunk")
    parser.add_argument("--dwell", type=float, default=default_dwell, help="Pause between chunks (s)")
    parser.add_argument("--direction", choices=["a2b", "b2a", "both"], default="both", help="Direction of test")
    parser.add_argument("--repeat", type=int, default=1, help="Number of times to repeat test")

    args = parser.parse_args()

    print("\n--- UART Bidirectional Communication Test ---")
    print(f"Total bytes: {args.total_bytes} | Chunk: {args.chunk_size} | Dwell: {args.dwell}s | Baudrate: {args.baudrate}")
    print(f"Direction: {args.direction} | Repeats: {args.repeat}")
    print(f"A1: {args.a1}, B1: {args.b1}")
    print(f"A2: {args.a2}, B2: {args.b2}")

    # Use provided ports in tests
    port_pairs = [
        (args.a1, args.b1),
        (args.a2, args.b2)
    ]

    success_count = 0
    total_tests = args.repeat * len(port_pairs) * (2 if args.direction == "both" else 1)

    for i in range(1, args.repeat + 1):
        for a_port, b_port in port_pairs:
            if args.direction in ["a2b", "both"]:
                result = run_test(a_port, b_port, args.baudrate, args.total_bytes, args.chunk_size, args.dwell, f"{i} ({a_port} ➝ {b_port})")
                success_count += result

            if args.direction in ["b2a", "both"]:
                result = run_test(b_port, a_port, args.baudrate, args.total_bytes, args.chunk_size, args.dwell, f"{i} ({b_port} ➝ {a_port})")
                success_count += result

    print(f"\n=== Test Summary ===")
    print(f"Total Tests Run: {total_tests}")
    print(f"Passed: {success_count}")
    print(f"Failed: {total_tests - success_count}")


if __name__ == "__main__":
    main()