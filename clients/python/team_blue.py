#!/usr/bin/env python3
import argparse
import threading
import time

import ssl_client  # noqa: F401  (runs the pb/ sys.path + env var bootstrap)
from ssl_client.commands import CommandSender
from ssl_client.referee import RefereeReceiver
from ssl_client.strategy import TeamConfig, TeamStrategy
from ssl_client.vision import VisionReceiver
from ssl_client.world import WorldModel

TEAM_IS_YELLOW = False
TEAM_NAME = "blue"


def main():
    parser = argparse.ArgumentParser(description="Simple blue-team SSL client for grSim")
    parser.add_argument(
        "--defend-positive-x",
        action="store_true",
        help="This team's own goal is on the +X side of the field (default: -X side)",
    )
    parser.add_argument("--grsim-host", default="127.0.0.1")
    parser.add_argument("--grsim-port", type=int, default=20011)
    parser.add_argument("--vision-group", default="224.5.23.2")
    parser.add_argument("--vision-port", type=int, default=10020)
    parser.add_argument("--referee-group", default="224.5.23.1")
    parser.add_argument("--referee-port", type=int, default=10003)
    parser.add_argument("--tick-hz", type=float, default=30.0)
    args = parser.parse_args()

    world = WorldModel()
    vision = VisionReceiver(world, group=args.vision_group, port=args.vision_port)
    referee = RefereeReceiver(group=args.referee_group, port=args.referee_port)
    sender = CommandSender(host=args.grsim_host, port=args.grsim_port)
    strategy = TeamStrategy(
        TeamConfig(is_team_yellow=TEAM_IS_YELLOW, defend_positive_x=args.defend_positive_x)
    )

    threading.Thread(target=vision.run_forever, daemon=True).start()
    threading.Thread(target=referee.run_forever, daemon=True).start()

    period = 1.0 / args.tick_hz
    print(f"[{TEAM_NAME}] sending commands to {args.grsim_host}:{args.grsim_port}", flush=True)
    try:
        while True:
            commands = strategy.compute_commands(world, referee.is_running())
            if commands:
                sender.send(TEAM_IS_YELLOW, commands)
            time.sleep(period)
    except KeyboardInterrupt:
        print(f"[{TEAM_NAME}] stopped", flush=True)


if __name__ == "__main__":
    main()
