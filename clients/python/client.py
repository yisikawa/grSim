#!/usr/bin/env python3
"""Minimal grSim Python client.

Sends velocity commands to grSim over UDP, equivalent to the Qt sample client.

Usage examples:
    python3 client.py --vx 1.0                     # move yellow robot 0 forward for 3 s
    python3 client.py --team blue --id 2 --w 3.14  # spin blue robot 2
    python3 client.py --vx 0.5 --kick 4 --duration 5

Regenerate the *_pb2.py modules after changing the .proto files with:
    protoc -I src/proto --python_out=clients/python \
        src/proto/grSim_Packet.proto src/proto/grSim_Commands.proto src/proto/grSim_Replacement.proto
"""

import argparse
import socket
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import grSim_Packet_pb2  # noqa: E402


def build_packet(args, stop=False):
    packet = grSim_Packet_pb2.grSim_Packet()
    packet.commands.timestamp = time.time()
    packet.commands.isteamyellow = args.team == "yellow"
    cmd = packet.commands.robot_commands.add()
    cmd.id = args.id
    cmd.veltangent = 0.0 if stop else args.vx
    cmd.velnormal = 0.0 if stop else args.vy
    cmd.velangular = 0.0 if stop else args.w
    cmd.kickspeedx = 0.0 if stop else args.kick
    cmd.kickspeedz = 0.0 if stop else args.chip
    cmd.spinner = False if stop else args.spin
    cmd.wheelsspeed = False
    return packet.SerializeToString()


def main():
    parser = argparse.ArgumentParser(description="Minimal grSim Python client")
    parser.add_argument("--ip", default="127.0.0.1", help="grSim address (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=20011, help="command listen port (default: 20011)")
    parser.add_argument("--team", choices=["yellow", "blue"], default="yellow")
    parser.add_argument("--id", type=int, default=0, help="robot id (default: 0)")
    parser.add_argument("--vx", type=float, default=0.0, help="velocity x m/s")
    parser.add_argument("--vy", type=float, default=0.0, help="velocity y m/s")
    parser.add_argument("--w", type=float, default=0.0, help="angular velocity rad/s")
    parser.add_argument("--kick", type=float, default=0.0, help="kick speed m/s")
    parser.add_argument("--chip", type=float, default=0.0, help="chip kick speed m/s")
    parser.add_argument("--spin", action="store_true", help="enable dribbler spinner")
    parser.add_argument("--duration", type=float, default=3.0, help="seconds to keep sending (default: 3)")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (args.ip, args.port)
    data = build_packet(args)

    print(f"Sending to {dest[0]}:{dest[1]} team={args.team} id={args.id} "
          f"vx={args.vx} vy={args.vy} w={args.w} for {args.duration}s ...")
    end = time.time() + args.duration
    count = 0
    while time.time() < end:
        sock.sendto(data, dest)
        count += 1
        time.sleep(0.02)  # 50 Hz, same rate as the Qt sample client

    sock.sendto(build_packet(args, stop=True), dest)
    print(f"Done. Sent {count} packets, then a stop command.")


if __name__ == "__main__":
    main()
