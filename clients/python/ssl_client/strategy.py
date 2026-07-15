import math
from dataclasses import dataclass
from typing import Dict, Optional

from .commands import RobotCommand
from .world import BallObservation, WorldModel

_SEEK_GAIN = 2.0
_MAX_SPEED = 2.0  # m/s, conservative vs. grSim's configured VelAbsoluteMax=5
_KICK_RANGE_M = 0.15
_KICK_SPEED_MPS = 4.0
_SEPARATION_DISTANCE = 0.4  # meters; own robots closer than this get pushed apart
_SEPARATION_GAIN = 1.5
_ANGLE_GAIN = 3.0
_MAX_ANGULAR_SPEED = 4.0  # rad/s, conservative vs. grSim's configured VelAngularMax=20
_KICK_ANGLE_TOLERANCE = 0.35  # radians (~20 degrees)

_FORMATION_PUSH_FORWARD = 0.5  # meters; extra forward shift when the ball is in the attacking half

# (forward_offset, lateral_offset) in meters, relative to the team's own goal
# line, for each non-keeper robot's fallback formation slot (assigned in
# ascending robot-id order, cycling if there are more robots than slots).
_FORMATION_SLOTS = [
    (1.0, 0.0),
    (1.0, 1.2),
    (1.0, -1.2),
    (2.5, 0.8),
    (2.5, -0.8),
]


@dataclass
class TeamConfig:
    is_team_yellow: bool
    defend_positive_x: bool  # True: this team's own goal is on the +X side


def _seek(current_x: float, current_y: float, target_x: float, target_y: float):
    vx = (target_x - current_x) * _SEEK_GAIN
    vy = (target_y - current_y) * _SEEK_GAIN
    speed = math.hypot(vx, vy)
    if speed > _MAX_SPEED:
        scale = _MAX_SPEED / speed
        vx *= scale
        vy *= scale
    return vx, vy


def _own_goal_x(world: WorldModel, defend_positive_x: bool) -> float:
    half_length = world.geometry.field_length / 2.0
    return half_length if defend_positive_x else -half_length


def _goalkeeper_target(world: WorldModel, defend_positive_x: bool):
    half_goal = world.geometry.goal_width / 2.0
    target_y = max(-half_goal, min(half_goal, world.ball.y))
    return _own_goal_x(world, defend_positive_x), target_y


def _formation_target(world: WorldModel, defend_positive_x: bool, slot_index: int, ball_x: float):
    """Fixed formation slot, relative to this team's own goal line, pushed
    _FORMATION_PUSH_FORWARD further forward when the ball is in the
    opponent's half of the field (a fixed, symmetric field split around
    field-center x=0 - not the same axis as the goalkeeper's lateral
    tracking, which follows the ball's Y)."""
    forward_offset, lateral_offset = _FORMATION_SLOTS[slot_index % len(_FORMATION_SLOTS)]
    forward_sign = -1.0 if defend_positive_x else 1.0
    own_goal_x = _own_goal_x(world, defend_positive_x)
    ball_in_attacking_half = (ball_x * forward_sign) > 0.0
    push_forward = _FORMATION_PUSH_FORWARD if ball_in_attacking_half else 0.0
    target_x = own_goal_x + forward_sign * (forward_offset + push_forward)
    return target_x, lateral_offset


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def _face(current_orientation: float, target_orientation: float):
    """Returns (vel_angular, angle_error) to rotate from current_orientation
    toward target_orientation, clamped to _MAX_ANGULAR_SPEED."""
    error = _normalize_angle(target_orientation - current_orientation)
    vel_angular = error * _ANGLE_GAIN
    if abs(vel_angular) > _MAX_ANGULAR_SPEED:
        vel_angular = math.copysign(_MAX_ANGULAR_SPEED, vel_angular)
    return vel_angular, error


def _apply_separation(rid, vx: float, vy: float, robot, own_robots: dict):
    """Push a robot's commanded velocity away from own teammates that are
    closer than _SEPARATION_DISTANCE, to avoid them stacking on top of each
    other. Not real path planning — just a simple repulsion term."""
    for other_id, other in own_robots.items():
        if other_id == rid:
            continue
        dx = robot.x - other.x
        dy = robot.y - other.y
        dist = math.hypot(dx, dy)
        if 0 < dist < _SEPARATION_DISTANCE:
            push = (_SEPARATION_DISTANCE - dist) * _SEPARATION_GAIN
            vx += (dx / dist) * push
            vy += (dy / dist) * push
    speed = math.hypot(vx, vy)
    if speed > _MAX_SPEED:
        scale = _MAX_SPEED / speed
        vx *= scale
        vy *= scale
    return vx, vy


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
                target_x, target_y = _goalkeeper_target(world, self._config.defend_positive_x)
                vx, vy = _seek(robot.x, robot.y, target_x, target_y)
                vx, vy = _apply_separation(rid, vx, vy, robot, own_robots)
                commands.append(RobotCommand(rid, vx, vy, 0.0, robot.orientation))
            elif rid == attacker_id:
                vx, vy = _seek(robot.x, robot.y, world.ball.x, world.ball.y)
                vx, vy = _apply_separation(rid, vx, vy, robot, own_robots)
                opponent_goal_x = -_own_goal_x(world, self._config.defend_positive_x)
                target_orientation = math.atan2(0.0 - robot.y, opponent_goal_x - robot.x)
                vel_angular, angle_error = _face(robot.orientation, target_orientation)
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
                target_x, target_y = _formation_target(
                    world, self._config.defend_positive_x, slot_index, world.ball.x
                )
                vx, vy = _seek(robot.x, robot.y, target_x, target_y)
                vx, vy = _apply_separation(rid, vx, vy, robot, own_robots)
                commands.append(RobotCommand(rid, vx, vy, 0.0, robot.orientation))

        return commands
