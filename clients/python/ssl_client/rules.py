"""Rule-constraint filter applied to strategy output as a safety net.

The strategy is expected to pick rule-compliant targets already; this
filter force-corrects whatever still violates the rules, so a strategy
bug degrades into conservative motion instead of a foul.
"""
import math
from typing import Optional

from .commands import RobotCommand
from .game_state import GameState, Phase

STOP_SPEED_CAP_MPS = 1.5
BALL_KEEP_OUT_M = 0.5
RETREAT_SPEED_MPS = 1.0

_SPEED_CAP_PHASES = (
    Phase.STOP,
    Phase.BALL_PLACEMENT_OURS,
    Phase.BALL_PLACEMENT_THEIRS,
)

# Phases where non-exempt robots must stay away from the ball (or, for
# ball placement, from the ball->designated_position corridor).
_BALL_KEEP_OUT_PHASES = (
    Phase.STOP,
    Phase.KICKOFF_OURS,
    Phase.KICKOFF_THEIRS,
    Phase.FREE_KICK_OURS,
    Phase.FREE_KICK_THEIRS,
    Phase.PENALTY_OURS,
    Phase.PENALTY_THEIRS,
    Phase.BALL_PLACEMENT_OURS,
    Phase.BALL_PLACEMENT_THEIRS,
)

_PLACEMENT_PHASES = (Phase.BALL_PLACEMENT_OURS, Phase.BALL_PLACEMENT_THEIRS)


def _nearest_point_on_segment(px, py, ax, ay, bx, by):
    abx, aby = bx - ax, by - ay
    ab_len_sq = abx * abx + aby * aby
    if ab_len_sq == 0.0:
        return ax, ay
    t = ((px - ax) * abx + (py - ay) * aby) / ab_len_sq
    t = max(0.0, min(1.0, t))
    return ax + t * abx, ay + t * aby


def _away_velocity(from_x, from_y, robot_x, robot_y):
    dx, dy = robot_x - from_x, robot_y - from_y
    dist = math.hypot(dx, dy)
    if dist < 1e-9:
        return RETREAT_SPEED_MPS, 0.0  # arbitrary but deterministic direction
    return dx / dist * RETREAT_SPEED_MPS, dy / dist * RETREAT_SPEED_MPS


def apply_rule_constraints(
    commands,
    game_state: GameState,
    *,
    own_robots,
    ball,
    geometry,
    defend_positive_x: bool,
    keeper_id: Optional[int],
    exempt_ids: frozenset = frozenset(),
):
    phase = game_state.phase
    out = []
    for cmd in commands:
        robot = own_robots.get(cmd.robot_id)
        if robot is None:
            out.append(cmd)
            continue
        vx, vy = cmd.vel_x, cmd.vel_y
        vel_angular = cmd.vel_angular
        kick, chip, dribble = cmd.kick_speed, cmd.chip_speed, cmd.dribble
        exempt = cmd.robot_id in exempt_ids

        if phase is Phase.HALT:
            vx = vy = vel_angular = 0.0
            kick = chip = 0.0
            dribble = False
        else:
            if ball is not None and phase in _BALL_KEEP_OUT_PHASES and not exempt:
                if phase in _PLACEMENT_PHASES and game_state.designated_position is not None:
                    nx, ny = _nearest_point_on_segment(
                        robot.x, robot.y, ball.x, ball.y,
                        game_state.designated_position[0], game_state.designated_position[1],
                    )
                else:
                    nx, ny = ball.x, ball.y
                if math.hypot(robot.x - nx, robot.y - ny) < BALL_KEEP_OUT_M:
                    vx, vy = _away_velocity(nx, ny, robot.x, robot.y)
            # Kicks are only legal in open play, or for the designated
            # kicker once the set piece is armed.
            if phase is not Phase.RUNNING and not (game_state.may_kick and exempt):
                kick = chip = 0.0
            if phase in _SPEED_CAP_PHASES:
                speed = math.hypot(vx, vy)
                if speed > STOP_SPEED_CAP_MPS:
                    scale = STOP_SPEED_CAP_MPS / speed
                    vx *= scale
                    vy *= scale

        out.append(RobotCommand(
            cmd.robot_id, vx, vy, vel_angular, cmd.robot_orientation,
            kick_speed=kick, chip_speed=chip, dribble=dribble,
        ))
    return out
