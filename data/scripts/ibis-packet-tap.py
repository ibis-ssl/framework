"""Inline tap for the ibis 715-byte command stream.

Binds a UDP port, decodes every robot slot, and optionally forwards the packet
on unchanged. Put it between two hops to see what is actually on the wire
without a packet capture -- useful where loopback capture needs root but the
chain's own ports are already held by listeners:

    crane --> cm4_sim --out-port 12399 --> [tap 12399] --> simulator-cli 12346

Usage:
    python3 data/scripts/ibis-packet-tap.py --port 12399 \
        [--forward 127.0.0.1:12346] [--robot-ids 0,1] [--every 60] [--count 0]

--every N   print one line per N packets for the robot (default 1; 0 = summary only)
--count N   stop after N datagrams (default 0 = run until Ctrl-C)

Prints, per slot: control mode, check counter, vision pose, target pose,
mode args, and the limits, so a mode 4 -> mode 3 conversion can be read
directly off both sides of a hop.
"""

import argparse
import socket
import struct
import sys

CMD_SIZE = 64
SLOTS = 11
PACKET_SIZE = SLOTS * (CMD_SIZE + 1)

MODE_NAMES = {3: "POLAR_VELOCITY", 4: "POSITION_TARGET"}


def two_byte(d, i, rng):
    raw = (d[i] << 8) | d[i + 1]
    return (raw - 32767.0) / 32767.0 * rng


def decode(d):
    """Offsets follow crane_sender/include/crane_sender/robot_packet.h."""
    mode = d[23]
    out = {
        "counter": d[1],
        "mode": mode,
        "vision": (two_byte(d, 2, 32.767), two_byte(d, 4, 32.767)),
        "vision_theta": two_byte(d, 6, 3.14159265),
        "target_theta": two_byte(d, 8, 3.14159265),
        "accel_limit": two_byte(d, 12, 32.767),
        "vel_limit": two_byte(d, 14, 32.767),
        "flags": d[22],
        "target_pos": (two_byte(d, 32, 32.767), two_byte(d, 34, 32.767)),
        "terminal_velocity": two_byte(d, 36, 32.767),
    }
    if mode == 3:
        out["args"] = ("r", two_byte(d, 24, 32.767), "theta", two_byte(d, 26, 32.767))
    elif mode == 4:
        out["args"] = ("tv_x", two_byte(d, 24, 32.767), "tv_y", two_byte(d, 26, 32.767))
    else:
        out["args"] = ("raw", d[24:32].hex())
    return out


def is_empty(d):
    return not any(d)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, required=True)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--forward", default="", help="host:port to pass the packet on to")
    ap.add_argument("--robot-ids", default="", help="only print these ids (default: all)")
    ap.add_argument("--every", type=int, default=1, help="print 1 line per N packets, 0 = summary only")
    ap.add_argument("--count", type=int, default=0, help="stop after N datagrams (0 = forever)")
    args = ap.parse_args()

    wanted = None
    if args.robot_ids:
        wanted = {int(x) for x in args.robot_ids.split(",") if x.strip()}

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind((args.bind, args.port))

    tx = fwd = None
    if args.forward:
        host, _, port = args.forward.partition(":")
        fwd = (host, int(port))
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"tap listening on {args.bind}:{args.port}"
          + (f", forwarding to {fwd[0]}:{fwd[1]}" if fwd else ", not forwarding"),
          flush=True)

    seen = 0
    mode_counts = {}
    try:
        while True:
            data, _ = rx.recvfrom(2048)
            if fwd:
                tx.sendto(data, fwd)          # 先に転送してチェーンを止めない
            if len(data) != PACKET_SIZE:
                print(f"  !! size {len(data)} (expected {PACKET_SIZE})", flush=True)
                continue
            seen += 1
            for slot in range(SLOTS):
                off = slot * (CMD_SIZE + 1)
                rid = data[off]
                d = data[off + 1: off + 1 + CMD_SIZE]
                if is_empty(d) or rid >= SLOTS:
                    continue
                if wanted is not None and rid not in wanted:
                    continue
                info = decode(d)
                mode_counts[info["mode"]] = mode_counts.get(info["mode"], 0) + 1
                if args.every and seen % args.every == 0:
                    name = MODE_NAMES.get(info["mode"], f"UNKNOWN({info['mode']})")
                    a = info["args"]
                    print(
                        f"#{seen:6d} id={rid} cnt={info['counter']:3d} mode={info['mode']}({name}) "
                        f"vision=({info['vision'][0]:+.3f},{info['vision'][1]:+.3f}) "
                        f"{a[0]}={a[1]:+.3f} {a[2]}={a[3]:+.3f} "
                        f"target=({info['target_pos'][0]:+.3f},{info['target_pos'][1]:+.3f}) "
                        f"term={info['terminal_velocity']:+.3f} "
                        f"vlim={info['vel_limit']:.2f} flags=0x{info['flags']:02x}",
                        flush=True)
            if args.count and seen >= args.count:
                break
    except KeyboardInterrupt:
        pass
    finally:
        print(f"\n--- {seen} datagrams ---", flush=True)
        for m in sorted(mode_counts):
            print(f"  mode {m} ({MODE_NAMES.get(m, 'UNKNOWN')}): {mode_counts[m]} slots", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
