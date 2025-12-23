import serial
import threading
import time
import os
from tqdm import tqdm 

# USAGE
# Open terminal, navigate to this Python file's directory, and input:
#   python3 uart_test.py -h
#   See line 75 for configurable parameters.
#   Previously used to test FTDI dongle speed/reliability.

# Thread to read incoming data from serial port and store it in a shared bytearray
def reader_thread(ser, storage, stop_event):
    while not stop_event.is_set():
        if ser.in_waiting:
            storage.extend(ser.read(ser.in_waiting))
        else:
            time.sleep(0.001)

# Sends random data over serial in chunks, updating progress visually
def send_data(ser, total_bytes, chunk_size, dwell_sec, progress_desc):
    sent_bytes = 0
    with tqdm(total=total_bytes, desc=progress_desc, unit='B', unit_scale=True, unit_divisor=1024) as pbar:
        while sent_bytes < total_bytes:
            remaining = total_bytes - sent_bytes
            this_chunk = min(chunk_size, remaining)
            chunk = os.urandom(this_chunk)  # Generate random data
            written = ser.write(chunk)
            if written != len(chunk):
                print(f"Write short! Expected {len(chunk)}, wrote {written}")
            ser.flush()
            sent_bytes += this_chunk
            pbar.update(this_chunk)
            time.sleep(dwell_sec)  # Optional delay to throttle sending

# Orchestrates the test: sets up serial ports, runs transmission, receives data, and reports results
def run_test(port_a, port_b, baudrate, total_bytes, chunk_size, dwell_sec):
    try:
        # Open serial ports for both devices
        ser_a = serial.Serial(port_a, baudrate=baudrate, timeout=2)
        ser_b = serial.Serial(port_b, baudrate=baudrate, timeout=2)

        # Start a background thread to read from the receiver port
        received_data = bytearray()
        stop_event = threading.Event()
        reader = threading.Thread(target=reader_thread, args=(ser_b, received_data, stop_event))
        reader.start()

        print("Baudrate confirmed:", ser_a.baudrate)
        print("\n--- Starting Test: A to B ---")
        send_data(ser_a, total_bytes, chunk_size, dwell_sec, "A to B")

        # Wait for any remaining incoming data
        time.sleep(2)
        stop_event.set()
        reader.join()

        # Summary report
        print(f"\nSent     : {total_bytes} bytes")
        print(f"Received : {len(received_data)} bytes")
        if len(received_data) == total_bytes:
            print("All data received correctly.")
        else:
            print(f"Data loss detected. Missing {total_bytes - len(received_data)} bytes.")
            print(f" {len(received_data)/total_bytes}% received.")

        # Close serial connections
        ser_a.close()
        ser_b.close()

    except serial.SerialException as e:
        print(f"SerialException: {e}")

if __name__ == "__main__":
    # === CONFIGURABLE PARAMETERS ===
    port_a = "/dev/ttyUSB4"
    port_b = "/dev/ttyUSB5"
    baudrate = 115200                # Baud rate
    total_bytes = 1024 * 128        # Total bytes to send (1 MB)
    chunk_size = 16284                # Bytes per write
    dwell_sec = .0000000000000000000000000001                 # Delay between writes (in seconds)
    # ===============================

    run_test(port_a, port_b, baudrate, total_bytes, chunk_size, dwell_sec)
