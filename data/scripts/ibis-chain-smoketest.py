"""Smoke test for the ibis command path of simulator-cli.

The simulator emulates the robot's STM32 (G474) main board: it accepts
POLAR_VELOCITY_TARGET (control mode 3) only and never closes a position loop.
POSITION_TARGET (mode 4) belongs to the robot-side CM4 controller that runs
between crane and the simulator. See docs/robot-side-position-control.md.

Checks, all against a real simulator-cli process over UDP:
  1. a mode 3 command drives the robot
  2. a mode 4 command stops it and logs a rate-limited warning
  3. a command whose vision_global_pos disagrees with the simulator is dropped,
     and says so -- this drop used to be silent, which is indistinguishable from
     "commanded to hold still" while packets keep arriving

Usage: python3 data/scripts/ibis-chain-smoketest.py [path/to/simulator-cli]
"""

import socket
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

CMD_SIZE = 64
SLOTS = 11
FEEDBACK_SIZE = 128

MODE_POLAR_VELOCITY = 3
MODE_POSITION_TARGET = 4

# The bound ports are offset well away from the defaults so a running match is
# untouched. Vision output has no port option on this branch, so it goes to the
# default address; that is send-only (no bind, no conflict), but it does inject
# extra frames into anything listening there -- do not run this during a match.
IBIS_PORT = 12397
FEEDBACK_BASE = 50700


def encode_two_byte(value, value_range):
    raw = int(32767.0 * (value / value_range) + 32767.0)
    raw = max(0, min(65535, raw))
    return bytes([(raw >> 8) & 0xFF, raw & 0xFF])


def build_command(counter, pos, yaw, mode, args=(0.0, 0.0), target=(0.0, 0.0)):
    """One 64-byte RobotCommandSerializedV2, laid out as in crane's robot_packet.h."""
    d = bytearray(CMD_SIZE)
    d[1] = counter & 0xFF
    d[2:4] = encode_two_byte(pos[0], 32.767)      # VISION_GLOBAL_X
    d[4:6] = encode_two_byte(pos[1], 32.767)      # VISION_GLOBAL_Y
    d[6:8] = encode_two_byte(yaw, 3.14159265)     # VISION_GLOBAL_THETA
    d[8:10] = encode_two_byte(yaw, 3.14159265)    # TARGET_GLOBAL_THETA -> omega ~ 0
    d[12:14] = encode_two_byte(4.0, 32.767)       # ACCELERATION_LIMIT
    d[14:16] = encode_two_byte(3.0, 32.767)       # LINEAR_VELOCITY_LIMIT
    d[16:18] = encode_two_byte(5.0, 32.767)       # ANGULAR_VELOCITY_LIMIT
    d[22] = 0x01                                  # FLAGS: IS_VISION_AVAILABLE
    d[23] = mode                                  # CONTROL_MODE
    d[24:26] = encode_two_byte(args[0], 32.767)   # CONTROL_MODE_ARGS
    d[26:28] = encode_two_byte(args[1], 32.767)
    d[32:34] = encode_two_byte(target[0], 32.767)  # TARGET_GLOBAL_POS_X
    d[34:36] = encode_two_byte(target[1], 32.767)  # TARGET_GLOBAL_POS_Y
    return bytes(d)


def build_packet(robot_id, command):
    """715-byte packet: 11 slots of (robot_id, 64-byte command), others zero-filled."""
    packet = bytearray()
    for slot in range(SLOTS):
        if slot == robot_id:
            packet += bytes([robot_id]) + command
        else:
            packet += bytes([0xFF]) + bytes(CMD_SIZE)
    return bytes(packet)


def parse_feedback(data):
    if len(data) != FEEDBACK_SIZE or data[0] != 0xAB or data[1] != 0xEA:
        return None
    return {
        "counter": data[3],
        "yaw": struct.unpack_from("<f", data, 4)[0],
        "x": struct.unpack_from("<f", data, 44)[0],
        "y": struct.unpack_from("<f", data, 48)[0],
        "vx": struct.unpack_from("<f", data, 52)[0],
        "vy": struct.unpack_from("<f", data, 56)[0],
    }


def distance(a, b):
    return ((a["x"] - b["x"]) ** 2 + (a["y"] - b["y"]) ** 2) ** 0.5


class Simulator:
    def __init__(self, binary, log_path=None):
        args = [
            str(binary), "-g", "2020B", "--realism", "None", "--localhost",
            "--ibis-port", str(IBIS_PORT),
            "--ibis-feedback-port-base", str(FEEDBACK_BASE),
        ]
        self.log_path = log_path
        self.log = open(log_path, "w") if log_path else subprocess.DEVNULL
        # stdbuf: log() in simulator.cpp does not flush, and SIGTERM would drop
        # a block-buffered pipe.
        self.proc = subprocess.Popen(["stdbuf", "-oL", "-eL"] + args,
                                     stdout=self.log, stderr=subprocess.STDOUT)
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx.bind(("127.0.0.1", FEEDBACK_BASE))
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        time.sleep(1.0)

    def send(self, packet):
        self.tx.sendto(packet, ("127.0.0.1", IBIS_PORT))

    def alive(self):
        return self.proc.poll() is None

    def recv(self, timeout=3.0):
        self.rx.settimeout(timeout)
        try:
            return parse_feedback(self.rx.recv(256))
        except socket.timeout:
            return None

    def drain(self):
        self.rx.settimeout(0.05)
        try:
            while True:
                self.rx.recv(256)
        except socket.timeout:
            pass

    def close(self):
        self.proc.terminate()
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait()
        if self.log is not subprocess.DEVNULL:
            self.log.close()


def run_commands(binary, log_path):
    """Mode 3 drives the robot; mode 4 stops it and warns."""
    sim = Simulator(binary, log_path=log_path)
    try:
        state = None
        for _ in range(20):
            if not sim.alive():
                return False, (f"simulator-cli exited (rc={sim.proc.returncode}); "
                               f"see {log_path}")
            state = sim.recv()
            if state:
                break
        if state is None:
            return False, f"no feedback from simulator; see {log_path}"
        start = state

        def drive(mode, args, seconds, counter):
            nonlocal state
            deadline = time.time() + seconds
            while time.time() < deadline:
                cmd = build_command(counter, (state["x"], state["y"]), state["yaw"],
                                    mode, args)
                sim.send(build_packet(0, cmd))
                counter += 1
                time.sleep(1 / 60)
                sim.rx.settimeout(0.001)
                try:
                    while True:
                        got = parse_feedback(sim.rx.recv(256))
                        if got:
                            state = got
                except socket.timeout:
                    pass
            return counter

        counter = drive(MODE_POLAR_VELOCITY, (1.5, 0.0), 2.0, 1)
        after_mode3 = dict(state)
        moved3 = distance(after_mode3, start)

        drive(MODE_POSITION_TARGET, (0.0, 0.0), 2.0, counter)
        after_mode4 = dict(state)
        moved4 = distance(after_mode4, after_mode3)

        ok = moved3 > 0.3 and abs(after_mode4["vx"]) < 0.05 and abs(after_mode4["vy"]) < 0.05
        return ok, (f"mode3 moved {moved3:.3f} m (want > 0.3), "
                    f"mode4 moved {moved4:.3f} m, "
                    f"final vel ({after_mode4['vx']:+.3f}, {after_mode4['vy']:+.3f}) "
                    f"(want ~0)")
    finally:
        sim.close()


def run_position_mismatch(binary, log_path):
    """A command claiming the wrong robot position is dropped, and says so.

    The simulator identifies the robot by matching the command's vision_global_pos
    against the robots on the field. A sender whose own position estimate has
    drifted past the threshold gets its commands dropped -- which looks exactly
    like a robot commanded to hold still, so the log line is the only way to tell.
    """
    sim = Simulator(binary, log_path=log_path)
    try:
        state = None
        for _ in range(20):
            if not sim.alive():
                return False, (f"simulator-cli exited (rc={sim.proc.returncode}); "
                               f"see {log_path}")
            state = sim.recv()
            if state:
                break
        if state is None:
            return False, f"no feedback from simulator; see {log_path}"
        start = dict(state)

        # Far enough past the threshold that feedback lag cannot pull it back under.
        offset = 1.0
        deadline = time.time() + 2.0
        counter = 1
        while time.time() < deadline:
            claimed = (state["x"] + offset, state["y"])
            cmd = build_command(counter, claimed, state["yaw"],
                                MODE_POLAR_VELOCITY, (1.5, 0.0))
            sim.send(build_packet(0, cmd))
            counter += 1
            time.sleep(1 / 60)
            sim.rx.settimeout(0.001)
            try:
                while True:
                    got = parse_feedback(sim.rx.recv(256))
                    if got:
                        state = got
            except socket.timeout:
                pass

        moved = distance(state, start)
        return moved < 0.05, (f"claimed a position {offset:.1f} m off, robot moved "
                              f"{moved:.3f} m (want ~0)")
    finally:
        sim.close()


def main():
    repo = Path(__file__).resolve().parents[2]
    binary = Path(sys.argv[1]) if len(sys.argv) > 1 else repo / "build" / "bin" / "simulator-cli"
    if not binary.exists():
        print(f"simulator-cli not found at {binary}", file=sys.stderr)
        return 2

    tmp = Path(tempfile.mkdtemp(prefix="ibis-smoketest-"))
    log = tmp / "simulator-cli.log"
    mismatch_log = tmp / "simulator-cli-mismatch.log"

    failures = 0
    ok, detail = run_commands(binary, log)
    print(f"[{'PASS' if ok else 'FAIL'}] commands: {detail}")
    failures += 0 if ok else 1

    text = log.read_text()
    warnings = [line for line in text.splitlines() if "POSITION_TARGET" in line]
    ok = len(warnings) > 0
    print(f"[{'PASS' if ok else 'FAIL'}] warning: {len(warnings)} POSITION_TARGET "
          f"warning(s) logged (rate limited to 1/s per robot)")
    failures += 0 if ok else 1

    # The commands above always claim the position the feedback just reported, so a
    # drop here would mean the matching rejects agreeing positions.
    stray = [line for line in text.splitlines() if "command dropped" in line]
    ok = not stray
    print(f"[{'PASS' if ok else 'FAIL'}] no false drops: {len(stray)} drop warning(s) "
          f"while the claimed position agreed (want 0)")
    failures += 0 if ok else 1

    ok, detail = run_position_mismatch(binary, mismatch_log)
    print(f"[{'PASS' if ok else 'FAIL'}] position mismatch: {detail}")
    failures += 0 if ok else 1

    drops = [line for line in mismatch_log.read_text().splitlines()
             if "command dropped" in line]
    ok = len(drops) > 0
    print(f"[{'PASS' if ok else 'FAIL'}] drop warning: {len(drops)} warning(s) logged "
          f"(rate limited to 1/s per robot)")
    failures += 0 if ok else 1

    print(f"\nlogs: {tmp}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
