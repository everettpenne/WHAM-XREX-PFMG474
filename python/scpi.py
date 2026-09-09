#!/usr/bin/env python3
import sys
import threading
import time

import serial
from serial.tools import list_ports

SERIAL_PORT = "/dev/cu.usbserial-130"
BAUD = 115200
# BAUD = 9600  # pre-2026-09-04 project baud; sibling PFM-STM32G474 still uses this
TIMEOUT = 0.1  # Short timeout for continuous reading


def list_available_ports():
    for p in list_ports.comports():
        print(f"  {p.device}  {p.description}")


def reader_thread(ser, stop_event):
    """Background thread: print anything the STM32 sends at any time."""
    while not stop_event.is_set():
        try:
            line = ser.readline()
            if line:
                decoded = line.decode("ascii", errors="replace").strip()
                print(f"\n← {decoded}\nSCPI> ", end="", flush=True)
        except serial.SerialException:
            break


def send_command(ser, command):
    if not command.endswith("\n"):
        command += "\n"
    ser.write(command.encode("ascii"))
    ser.flush()
    print(f"→ Sent: {repr(command)}")


def main():
    print("=" * 60)
    print("Everett's Serial Terminal for STM32")
    print("=" * 60)
    print("\nAvailable ports:")
    list_available_ports()
    print()

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD, timeout=TIMEOUT)
        print(f"✓ Connected to {SERIAL_PORT} at {BAUD} baud\n")
        time.sleep(0.2)
    except serial.SerialException as e:
        print(f"✗ Could not open {SERIAL_PORT}: {e}")
        sys.exit(1)

    stop_event = threading.Event()
    t = threading.Thread(target=reader_thread, args=(ser, stop_event), daemon=True)
    t.start()

    print("Type a command and press Enter. 'quit' to exit.\n")
    print("=" * 60 + "\n")

    try:
        while True:
            try:
                user_input = input("STM32Serial> ").strip()
            except EOFError:
                break

            if not user_input:
                continue
            if user_input.lower() in ["quit", "exit", "q"]:
                break
            if user_input.lower() == "clear":
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                print("✓ Buffers cleared")
                continue

            send_command(ser, user_input)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        stop_event.set()
        ser.close()
        print("Serial port closed.")


if __name__ == "__main__":
    main()
