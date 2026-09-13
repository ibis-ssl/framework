"""Measure how far a robot travels after each of the three ways it can stop.

The simulator reproduces the robot's two hardware stop paths. The wheel PID brakes
when told to (mode 3 with r=0). Drive is cut entirely when the stop condition fires
-- the real G474 answers that with omniStopAll(), which writes duty 0, and the CAN
frame has no brake bit, so hardware coasts. A third case, commands simply not
arriving, keeps executing the last command for 0.1 s and then coasts as well.
See docs/robot-side-position-control.md.

Two things make a naive measurement of this wrong in a way that looks right:

  * Measure in +y only, and on both teams. Robots start near y=-2.8, so -y is the
    wall and both x directions are blocked by other robots. A blocked run still
    produces a plausible number -- 0.12 to 0.40 m, varying run to run -- and there
    is nothing in the reading that says it was blocked. Both teams agreeing to a
    few mm is the signal that the coast was free.

  * One simulator process per measurement. The feedback velocity field freezes when
    commands stop arriving (see the fidelity gaps in the doc above), so a previous
    run's frozen value is read as "already at speed" and the next run never
    accelerates -- reporting a full-speed start and a stopping distance of zero.

Usage: python3 data/scripts/ibis-stop-distance.py [kinds] [base-port] [binary]
       kinds: comma separated, any of brake,estop,drop (default: all three)
"""

import math
import socket
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

# Commands are sent at this speed; the run starts measuring once the robot reaches
# TARGET_V. Stopping distance goes with speed, so the three paths are only
# comparable if they all start from the same one. A fixed-duration acceleration
# phase does not do this: the speed reached varies with the robot's start pose.
COMMAND_SPEED = 1.5
TARGET_V = 1.45

MODE_POLAR_VELOCITY = 3
HEADING_PLUS_Y = math.pi / 2


def encode_two_byte(value, value_range):
    raw = int(32767.0 * (value / value_range) + 32767.0)
    raw = max(0, min(65535, raw))
    return bytes([(raw >> 8) & 0xFF, raw & 0xFF])


def build_packet(counter, pos, speed, theta, stop_emergency=False):
    """715-byte packet driving robot 0 at `speed` along global direction `theta`."""
    d = bytearray(64)
    d[1] = counter & 0xFF
    d[2:4] = encode_two_byte(pos[0], 32.767)       # VISION_GLOBAL_X
    d[4:6] = encode_two_byte(pos[1], 32.767)       # VISION_GLOBAL_Y
    d[6:8] = encode_two_byte(0.0, math.pi)         # VISION_GLOBAL_THETA
    d[8:10] = encode_two_byte(0.0, math.pi)        # TARGET_GLOBAL_THETA
    d[12:14] = encode_two_byte(4.0, 32.767)        # ACCELERATION_LIMIT
    d[14:16] = encode_two_byte(4.0, 32.767)        # LINEAR_VELOCITY_LIMIT
    d[16:18] = encode_two_byte(5.0, 32.767)        # ANGULAR_VELOCITY_LIMIT
    d[22] = 0x01 | (0x08 if stop_emergency else 0x00)   # IS_VISION_AVAILABLE | STOP_EMERGENCY
    d[23] = MODE_POLAR_VELOCITY
    d[24:26] = encode_two_byte(speed, 32.767)      # CONTROL_MODE_ARGS: r
    # theta is in the same +-32.767 range as r, not +-pi (ibis_protocol.h). Packing
    # it as a pi-range angle silently drives the robot in a different direction.
    d[26:28] = encode_two_byte(theta, 32.767)
    packet = bytearray()
    for slot in range(11):
        packet += (bytes([0]) + bytes(d)) if slot == 0 else (bytes([0xFF]) + bytes(64))
    return bytes(packet)


def parse_feedback(data):
    """(x, y, vx) in metres and m/s, or None if this is not a feedback packet."""
    if len(data) != 128 or data[0] != 0xAB or data[1] != 0xEA:
        return None
    return (struct.unpack_from("<f", data, 44)[0],
            struct.unpack_from("<f", data, 48)[0],
            struct.unpack_from("<f", data, 52)[0])


def measure(binary, kind, team, port):
    """Accelerate to TARGET_V along +y, stop by `kind`, return distance travelled."""
    feedback_port = port + 40000
    log_dir = Path(tempfile.mkdtemp(prefix="ibis-stop-"))
    log = open(log_dir / "simulator-cli.log", "w")
    proc = subprocess.Popen(
        [str(binary), "-g", "2020B", "--realism", "None", "--localhost",
         "--ibis-port", str(port), "--ibis-feedback-port-base", str(feedback_port),
         "--ibis-team-color", team],
        stdout=log, stderr=subprocess.STDOUT)
    try:
        rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        rx.bind(("127.0.0.1", feedback_port))
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        time.sleep(2.0)

        def latest(state):
            rx.settimeout(0.001)
            try:
                while True:
                    got = parse_feedback(rx.recv(256))
                    if got:
                        state = got
            except socket.timeout:
                pass
            return state

        state = None
        for _ in range(80):
            rx.settimeout(3.0)
            try:
                got = parse_feedback(rx.recv(256))
            except socket.timeout:
                return None, f"{log_dir}: no feedback"
            if got:
                state = got
                break
        if state is None:
            return None, f"{log_dir}: no feedback"

        counter = 1
        deadline = time.time() + 4.0
        while time.time() < deadline and abs(state[2]) < TARGET_V:
            tx.sendto(build_packet(counter, state, COMMAND_SPEED, HEADING_PLUS_Y),
                      ("127.0.0.1", port))
            counter += 1
            time.sleep(1 / 125)
            state = latest(state)
        if abs(state[2]) < TARGET_V:
            return None, f"only reached {state[2]:+.3f} m/s; see {log_dir}"

        origin = (state[0], state[1])
        deadline = time.time() + 2.0
        while time.time() < deadline:
            if kind == "estop":
                tx.sendto(build_packet(counter, state, COMMAND_SPEED, HEADING_PLUS_Y,
                                       stop_emergency=True), ("127.0.0.1", port))
            elif kind == "brake":
                tx.sendto(build_packet(counter, state, 0.0, HEADING_PLUS_Y),
                          ("127.0.0.1", port))
            # "drop": send nothing at all
            counter += 1
            time.sleep(1 / 125)
            state = latest(state)
        return math.hypot(state[0] - origin[0], state[1] - origin[1]), None
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        log.close()


def main():
    kinds = sys.argv[1].split(",") if len(sys.argv) > 1 else ["brake", "estop", "drop"]
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 15600
    repo = Path(__file__).resolve().parents[2]
    binary = Path(sys.argv[3]) if len(sys.argv) > 3 else repo / "build" / "bin" / "simulator-cli"
    if not binary.exists():
        print(f"simulator-cli not found at {binary}", file=sys.stderr)
        return 2

    results = {}
    for team in ("yellow", "blue"):
        row = []
        for kind in kinds:
            distance, error = measure(binary, kind, team, port)
            port += 2
            results[(team, kind)] = distance
            row.append(f"{kind}={'FAILED' if distance is None else format(distance, '.3f')}"
                       + (f" ({error})" if error else ""))
        print(f"{team:6s} +y: " + "  ".join(row))

    # Both teams are different robots on different parts of the field. Agreeing means
    # neither hit anything; disagreeing means at least one run was blocked and the
    # numbers cannot be compared against each other or against the doc.
    print()
    ok = True
    for kind in kinds:
        a, b = results[("yellow", kind)], results[("blue", kind)]
        if a is None or b is None:
            print(f"[FAIL] {kind}: a run did not complete")
            ok = False
            continue
        spread = abs(a - b)
        agree = spread < 0.02
        ok = ok and agree
        print(f"[{'PASS' if agree else 'FAIL'}] {kind}: teams agree to {spread:.3f} m "
              f"(want < 0.020; larger means something was in the way)")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
