"""
Real-time EV & ADAS Dashboard - Python Companion App
------------------------------------------------------
Connects to the STM32F103 (via the PiximLab virtual COM port / USB-UART
bridge) at 115200 baud and:
  1. Parses the 1 Hz telemetry frames sent by uart_shell.c
  2. Prints a live-updating dashboard in the terminal
  3. Lets the operator send shell commands: status, fault clear,
     set speed <n>, set mode <eco|normal|sport>

Requires: pip install pyserial
"""

import sys
import time
import threading
import serial

BAUD_RATE = 115200
DEFAULT_PORT = "COM3"       # Windows example; use "/dev/ttyUSB0" on Linux/PiximLab export


def parse_telemetry(line: str) -> dict:
    """Parses a 'KEY:VALUE KEY:VALUE ...' telemetry frame into a dict."""
    data = {}
    for token in line.strip().split():
        if ":" in token:
            key, val = token.split(":", 1)
            try:
                data[key] = int(val)
            except ValueError:
                data[key] = val
    return data


STATE_NAMES = {0: "PARKED", 1: "READY", 2: "DRIVING", 3: "REGEN", 4: "FAULT"}
FAULT_NAMES = {0x00: "NONE", 0x01: "OVERHEAT", 0x02: "LOW_BATTERY", 0x04: "COLLISION"}


def render_dashboard(d: dict):
    ttc = d.get("TTC", 999) / 10.0
    alarm = d.get("ALARM", 0)
    state = STATE_NAMES.get(d.get("STATE", -1), "UNKNOWN")
    fault = d.get("FAULT", 0)

    alarms = []
    if alarm & 0x01: alarms.append("CRITICAL-FRONT")
    if alarm & 0x02: alarms.append("WARNING-FRONT")
    if alarm & 0x04: alarms.append("BLIND-SPOT-LEFT")
    if alarm & 0x08: alarms.append("BLIND-SPOT-RIGHT")
    alarm_str = ", ".join(alarms) if alarms else "clear"

    print("\033[2J\033[H", end="")  # clear terminal
    print("=" * 55)
    print("      REAL-TIME EV & ADAS DASHBOARD (Blue Pill)")
    print("=" * 55)
    print(f" Vehicle State : {state}   |  Fault Flags: {fault:#04x}")
    print(f" Speed         : {d.get('SPEED', 0):>4} km/h")
    print(f" SOC           : {d.get('SOC', 0):>4} %")
    print(f" Motor Torque  : {d.get('TORQUE', 0):>4} Nm")
    print(f" Motor Temp    : {d.get('TEMP', 0):>4} C")
    print(f" Est. Range    : {d.get('RANGE', 0):>4} km")
    print("-" * 55)
    print(f" Front Distance: {d.get('FRONT', 0):>4} cm   TTC: {ttc:>5.1f} s")
    print(f" Left Distance : {d.get('LEFT', 0):>4} cm")
    print(f" Right Distance: {d.get('RIGHT', 0):>4} cm")
    print("-" * 55)
    print(f" Alarms        : {alarm_str}")
    print("=" * 55)
    print(" Commands: status | fault clear | set speed <0-20> | set mode <eco|normal|sport> | quit")


def reader_thread(ser: serial.Serial):
    while True:
        try:
            line = ser.readline().decode(errors="ignore")
        except serial.SerialException:
            break
        if not line:
            continue
        if line.startswith("SPEED:"):
            render_dashboard(parse_telemetry(line))
        else:
            # Shell command responses / status replies
            print(f"[MCU] {line.strip()}")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
    except serial.SerialException as e:
        print(f"Could not open {port}: {e}")
        return

    t = threading.Thread(target=reader_thread, args=(ser,), daemon=True)
    t.start()

    try:
        while True:
            cmd = input()
            if cmd.strip().lower() in ("quit", "exit"):
                break
            ser.write((cmd.strip() + "\n").encode())
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()
