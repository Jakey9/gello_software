#!/usr/bin/env python3
"""Scan the Zhonglin serial bus to discover connected servo IDs and read positions.

Usage:
    python -m gello.zhonglin.servo_scan                  # scan IDs 0-15
    python -m gello.zhonglin.servo_scan --max-id 20      # scan IDs 0-20
    python -m gello.zhonglin.servo_scan --port /dev/ttyUSB1
    python -m gello.zhonglin.servo_scan --loop            # continuous read of found servos
"""

import argparse
import re
import time

import serial

PWM_PATTERN = re.compile(r"P(\d{4})")


def send_command(ser, cmd, delay=0.01):
    ser.write(cmd.encode("ascii"))
    time.sleep(delay)
    response = ser.read_all()
    return response.decode("ascii", errors="ignore")


def pwm_to_angle(response_str, pwm_min=500, pwm_max=2500, angle_range=270):
    match = PWM_PATTERN.search(response_str)
    if not match:
        return None
    pwm_val = int(match.group(1))
    return (pwm_val - pwm_min) / (pwm_max - pwm_min) * angle_range


def scan_servos(ser, max_id=15, delay=0.01):
    """Probe each ID with PRAD and return list of (id, angle_deg, raw_response)."""
    found = []
    for sid in range(max_id + 1):
        cmd = f"#{sid:03d}PRAD!"
        raw = send_command(ser, cmd, delay=delay)
        angle = pwm_to_angle(raw.strip())
        if angle is not None:
            found.append((sid, angle, raw.strip()))
    return found


def main():
    parser = argparse.ArgumentParser(description="Scan Zhonglin servo bus")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--max-id", type=int, default=15, help="highest ID to scan")
    parser.add_argument("--delay", type=float, default=0.01, help="seconds to wait per command")
    parser.add_argument("--loop", action="store_true", help="continuously read found servos")
    args = parser.parse_args()

    with serial.Serial(args.port, args.baudrate, timeout=0.01) as ser:
        print(f"Serial port opened: {args.port}")
        print(f"Scanning IDs 0 .. {args.max_id} (delay={args.delay}s) ...\n")

        found = scan_servos(ser, max_id=args.max_id, delay=args.delay)

        if not found:
            print("No servos detected!")
            return

        print(f"Found {len(found)} servo(s):\n")
        print(f"  {'ID':>4}  {'Angle (deg)':>12}  {'Raw response'}")
        print(f"  {'----':>4}  {'------------':>12}  {'------------'}")
        for sid, angle, raw in found:
            print(f"  {sid:4d}  {angle:12.1f}  {raw}")

        found_ids = [sid for sid, _, _ in found]
        print(f"\nServo IDs to use in driver.py: {found_ids}")

        if not args.loop:
            return

        print("\n--- Continuous read (Ctrl+C to stop) ---\n")
        try:
            while True:
                t0 = time.time()
                angles = []
                for sid in found_ids:
                    cmd = f"#{sid:03d}PRAD!"
                    raw = send_command(ser, cmd, delay=args.delay)
                    angle = pwm_to_angle(raw.strip())
                    angles.append(angle)
                dt = time.time() - t0
                angle_strs = [f"{a:6.1f}" if a is not None else "  None" for a in angles]
                hz = 1.0 / dt if dt > 0 else 0
                print(f"IDs {found_ids} | angles(deg): [{', '.join(angle_strs)}] | {dt*1000:.1f}ms ({hz:.1f} Hz)")
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
