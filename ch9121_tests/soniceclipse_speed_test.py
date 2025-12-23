import serial
import time
import threading
import numpy as np

def reader_thread(ser, buffer, stop_event):
    while not stop_event.is_set():
        if ser.in_waiting:
            buffer.extend(ser.read(ser.in_waiting))
        else:
            time.sleep(0.0001)

def send_data(ser, total_bytes, chunk_size, dwell_time):
    sent = 0
    data = bytes(np.random.bytes(total_bytes))
    start_time = time.time()
    for i in range(0, total_bytes, chunk_size):
        ser.write(data[i:i+chunk_size])
        time.sleep(dwell_time)
    ser.flush()
    end_time = time.time()
    return data, end_time - start_time

def test_transfer(port_a, port_b, dwell_time, total_bytes, chunk_size):
    try:
        ser_a = serial.Serial(port_a, baudrate=115200, timeout=1)
        ser_b = serial.Serial(port_b, baudrate=115200, timeout=1)
    except Exception as e:
        print(f"Error opening ports: {e}")
        return None, False

    recv_buffer = bytearray()
    stop_event = threading.Event()
    reader = threading.Thread(target=reader_thread, args=(ser_b, recv_buffer, stop_event))
    reader.start()

    data_sent, duration = send_data(ser_a, total_bytes, chunk_size, dwell_time)

    time.sleep(1)
    stop_event.set()
    reader.join()

    ser_a.close()
    ser_b.close()

    if len(recv_buffer) != len(data_sent):
        return 0, False

    if recv_buffer != data_sent:
        return 0, False

    rate = len(data_sent) / duration / 1024  # KB/s
    return rate, True

def format_dwell(dwell):
    s = f"{dwell:.12f}".rstrip("0")
    if s[-1] == ".":
        s = s[:-1]
    digits = s.split(".")[-1]
    trimmed = len(digits.rstrip("0")) + 1
    return f"{dwell:.{trimmed}f}"

def verify_stability(port_a, port_b, dwell, total_bytes, chunk_size):
    print(f"\n--- Verifying Stability at {format_dwell(dwell)}s ---")
    rates = []
    for i in range(10):
        print(f"Verification run {i+1}/10...")
        rate, success = test_transfer(port_a, port_b, dwell, total_bytes, chunk_size)
        if not success:
            print(f"Run {i+1}: FAIL")
            return False, rate
        print(f"Run {i+1}: PASS")
        rates.append(rate)
    avg_rate = sum(rates) / len(rates)
    return True, avg_rate

def find_optimal_dwell(port_a, port_b):
    total_bytes = 1024 * 16
    summary = []

    for chunk_size in [8, 16, 32, 64]:
        print(f"\n==================== Testing Chunk Size: {chunk_size} ====================")
        max_dwell = 0.002 * (chunk_size / 8)
        min_dwell = 0.000000005 * (chunk_size / 8)
        step_dwell = 0.0001

        num_steps = int((max_dwell - min_dwell) / step_dwell) + 1
        initial_dwell_times = [max_dwell - i * step_dwell for i in range(num_steps)]

        print("--- Initial Sweep ---")
        best = None
        last_success = None
        pre_last_success = None

        for dwell in initial_dwell_times:
            print(f"\nTesting dwell time: {format_dwell(dwell)} seconds...")
            rate, success = test_transfer(port_a, port_b, dwell, total_bytes, chunk_size)
            status = "PASS" if success else "FAIL"
            print(f"Dwell={format_dwell(dwell)}s | {status} | Rate={rate:.2f} KB/s")
            if success:
                pre_last_success = last_success
                last_success = dwell
                if best is None or rate > best[1]:
                    best = (dwell, rate)
            else:
                break

        if not best:
            print("No successful transfer found.")
            continue

        candidate_dwell = last_success

        while True:
            stable, avg_rate = verify_stability(port_a, port_b, candidate_dwell, total_bytes, chunk_size)
            if stable:
                print(f"\nOptimal Dwell Time for chunk size {chunk_size}: {format_dwell(candidate_dwell)}s at {avg_rate:.2f} KB/s")
                summary.append((chunk_size, format_dwell(candidate_dwell), avg_rate))
                break

            if pre_last_success:
                candidate_dwell = pre_last_success
                pre_last_success = None
            else:
                print("\nNo reliably stable dwell time found.")
                break

    print("\n================== Summary Table =====================")
    print(" Chunk Size | Optimal Dwell Time (s) | Avg Rate (KB/s)")
    print("------------|------------------------|----------------")
    for chunk, dwell, rate in summary:
        print(f"{chunk:11} | {dwell:22} | {rate:14.2f}")

if __name__ == "__main__":
    import argparse

    port_a1 = "/dev/ch9121_A1"
    port_a2 = "/dev/ch9121_A2"
    port_b1 = "/dev/ch9121_B1"
    port_b2 = "/dev/ch9121_B2"

    parser = argparse.ArgumentParser(description="Find optimal dwell time for COM port transfer")
    parser.add_argument('--port-a', type=str, default=port_b2, help='First serial port (e.g., /dev/ttyACM2)')
    parser.add_argument('--port-b', type=str, default=port_a2, help='Second serial port (e.g., /dev/ttyACM4)')
    args = parser.parse_args()

    find_optimal_dwell(args.port_a, args.port_b)



