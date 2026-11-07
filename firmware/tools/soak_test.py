#!/usr/bin/env python3
"""Deploy and stow the arm over and over and check it lands where it should.

    pip install pyserial
    python firmware/tools/soak_test.py              # auto-finds the Pico, 50 cycles
    python firmware/tools/soak_test.py --port COM5 --cycles 10

Passes when every cycle reaches LIVE and STOWED without a FAULT, and the final
encoder angles of each are within --tol degrees of the first cycle's.
"""

import argparse
import re
import sys
import time

import serial
from serial.tools import list_ports

STATE_RE = re.compile(r"STATE (\w+) yaw=(-?[\d.]+) sh=(-?[\d.]+) el=(-?[\d.]+)(?: wr=(-?[\d.]+))?")


def find_pico():
    for p in list_ports.comports():
        if p.vid == 0x2E8A:
            return p.device
    sys.exit("No Pico found - pass --port")


def wait_for(ser, want, timeout):
    end = time.time() + timeout
    while time.time() < end:
        line = ser.readline().decode(errors="replace").strip()
        if not line:
            continue
        m = STATE_RE.match(line)
        if m:
            if m.group(1) == "FAULT":
                raise RuntimeError(line)
            if m.group(1) == want:
                return tuple(float(v) for v in m.groups()[1:] if v is not None)
        elif line.startswith("ERR"):
            raise RuntimeError(line)
    raise TimeoutError(f"no {want} within {timeout} s")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--cycles", type=int, default=50)
    ap.add_argument("--tol", type=float, default=1.0, help="allowed spread in degrees")
    ap.add_argument("--timeout", type=float, default=40)
    args = ap.parse_args()

    ser = serial.Serial(args.port or find_pico(), 115200, timeout=0.5)
    time.sleep(0.5)
    ser.reset_input_buffer()
    ser.write(b"STOW\n")
    wait_for(ser, "STOWED", args.timeout)

    first = {}
    worst = 0.0
    for i in range(1, args.cycles + 1):
        for cmd, state in ((b"DEPLOY\n", "LIVE"), (b"STOW\n", "STOWED")):
            t0 = time.time()
            ser.write(cmd)
            pose = wait_for(ser, state, args.timeout)
            ser.write(b"STATUS?\n")  # the STATE line sent on arrival can be read mid-settle; ask again
            pose = wait_for(ser, state, 5)
            ref = first.setdefault(state, pose)
            err = max(abs(a - b) for a, b in zip(pose, ref))
            worst = max(worst, err)
            print(f"cycle {i:3d} {state:<7} {time.time() - t0:5.1f} s  pose {pose}  off by {err:.2f} deg")
            if err > args.tol:
                sys.exit(f"FAIL: {state} pose drifted {err:.2f} deg (tolerance {args.tol})")
    print(f"PASS: {args.cycles} cycles, worst drift {worst:.2f} deg")


if __name__ == "__main__":
    main()
