import math
from dataclasses import dataclass
from typing import Dict, Optional

from .commands import RobotCommand
from .roles import (
    apply_separation,
    face,
    formation_target,
    goalkeeper_target,
    own_goal_x,
    seek,
)
from .world import BallObservation, WorldModel

_KICK_RANGE_M = 0.15
_KICK_SPEED_MPS = 4.0
_KICK_ANGLE_TOLERANCE = 0.35  # radians (~20 degrees)


@dataclass
class TeamConfig:
    is_team_yellow: bool
    defend_positive_x: bool  # True: this team's own goal is on the +X side


class TeamStrategy:
    def __init__(self, config: TeamConfig):
        self._config = config
        self._goalkeeper_id: Optional[int] = None
        self._formation_slots: Dict[int, int] = {}

    def compute_commands(self, world: WorldModel, referee_running: bool):
        # Snapshot: the Vision receiver runs on its own thread and mutates
        # world.*_robots concurrently with this method's iteration. Inserting a
        # newly-seen robot id mid-iteration would raise "dictionary changed size
        # during iteration".
        own_robots = dict(world.yellow_robots if self._config.is_team_yellow else world.blue_robots)
        if not own_robots or world.ball is None or world.geometry is None:
            return []

        if self._goalkeeper_id is None or self._goalkeeper_id not in own_robots:
            self._goalkeeper_id = min(own_robots.keys())

        non_keeper_ids = sorted(rid for rid in own_robots if rid != self._goalkeeper_id)
        for slot_index, rid in enumerate(non_keeper_ids):
            self._formation_slots.setdefault(rid, slot_index)

        attacker_id = None
        if non_keeper_ids:
            attacker_id = min(
                non_keeper_ids,
                key=lambda rid: math.hypot(
                    own_robots[rid].x - world.ball.x, own_robots[rid].y - world.ball.y
                ),
            )

        commands = []
        for rid, robot in own_robots.items():
            if not referee_running:
                commands.append(RobotCommand(rid, 0.0, 0.0, 0.0, robot.orientation))
                continue

            if rid == self._goalkeeper_id:
                target_x, target_y = goalkeeper_target(world, self._config.defend_positive_x)
                vx, vy = seek(robot.x, robot.y, target_x, target_y)
                vx, vy = apply_separation(rid, vx, vy, robot, own_robots)
                commands.append(RobotCommand(rid, vx, vy, 0.0, robot.orientation))
            elif rid == attacker_id:
                vx, vy = seek(robot.x, robot.y, world.ball.x, world.ball.y)
                vx, vy = apply_separation(rid, vx, vy, robot, own_robots)
                opponent_goal_x = -own_goal_x(world, self._config.defend_positive_x)
                target_orientation = math.atan2(0.0 - robot.y, opponent_goal_x - robot.x)
                vel_angular, angle_error = face(robot.orientation, target_orientation)
                dist = math.hypot(robot.x - world.ball.x, robot.y - world.ball.y)
                facing_goal = abs(angle_error) < _KICK_ANGLE_TOLERANCE
                kick = _KICK_SPEED_MPS if (dist < _KICK_RANGE_M and facing_goal) else 0.0
                commands.append(
                    RobotCommand(
                        rid, vx, vy, vel_angular, robot.orientation, kick_speed=kick, dribble=True
                    )
                )
            else:
                slot_index = self._formation_slots[rid]
                target_x, target_y = formation_target(
                    world, self._config.defend_positive_x, slot_index, world.ball.x
                )
                vx, vy = seek(robot.x, robot.y, target_x, target_y)
                vx, vy = apply_separation(rid, vx, vy, robot, own_robots)
                commands.append(RobotCommand(rid, vx, vy, 0.0, robot.orientation))

        return commands
