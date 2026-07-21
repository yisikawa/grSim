"""Rule-constraint filter applied to strategy output as a safety net.

The strategy is expected to pick rule-compliant targets already; this
filter force-corrects whatever still violates the rules, so a strategy
bug degrades into conservative motion instead of a foul.
"""
import math
from typing import Dict, List, Optional

from .commands import RobotCommand
from .game_state import GameState, Phase
from .world import BallObservation, FieldGeometry, RobotObservation

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


def _defense_area_bounds(geometry, positive_x: bool):
    half_len = geometry.field_length / 2.0
    half_w = geometry.penalty_area_width / 2.0
    if positive_x:
        return half_len - geometry.penalty_area_depth, half_len, -half_w, half_w
    return -half_len, -half_len + geometry.penalty_area_depth, -half_w, half_w


def _push_out_of_rect(x, y, x_min, x_max, y_min, y_max, goal_line_at_max_x: bool):
    """Push toward the nearest legal exit face of the rect. The face on the
    goal-line side is never a legal exit (it leads out of the field)."""
    if not (x_min <= x <= x_max and y_min <= y <= y_max):
        return None
    exits = []
    if not goal_line_at_max_x:
        exits.append((x_max - x, (1.0, 0.0)))
    else:
        exits.append((x - x_min, (-1.0, 0.0)))
    exits.append((y - y_min, (0.0, -1.0)))
    exits.append((y_max - y, (0.0, 1.0)))
    _, (dx, dy) = min(exits, key=lambda e: e[0])
    return dx * RETREAT_SPEED_MPS, dy * RETREAT_SPEED_MPS


def _keep_out_push(robot, geometry, defend_positive_x: bool, is_keeper: bool):
    """Always-on constraints: return an override velocity, or None."""
    half_len = geometry.field_length / 2.0
    half_w = geometry.field_width / 2.0
    if abs(robot.x) > half_len or abs(robot.y) > half_w:
        tx = max(-half_len, min(half_len, robot.x))
        ty = max(-half_w, min(half_w, robot.y))
        return _away_velocity(robot.x, robot.y, tx, ty)  # toward clamped point
    if not is_keeper:
        push = _push_out_of_rect(
            robot.x, robot.y,
            *_defense_area_bounds(geometry, defend_positive_x),
            goal_line_at_max_x=defend_positive_x,
        )
        if push is not None:
            return push
    return _push_out_of_rect(
        robot.x, robot.y,
        *_defense_area_bounds(geometry, not defend_positive_x),
        goal_line_at_max_x=not defend_positive_x,
    )


def apply_rule_constraints(
    commands: List[RobotCommand],
    game_state: GameState,
    *,
    own_robots: Dict[int, RobotObservation],
    ball: Optional[BallObservation],
    geometry: FieldGeometry,
    defend_positive_x: bool,
    keeper_id: Optional[int],
    exempt_ids: frozenset = frozenset(),
) -> List[RobotCommand]:
    phase = game_state.phase
    out = []
    for cmd in commands:
        # A robot missing from own_robots (e.g. vision dropout) still gets
        # every phase-based constraint that doesn't require a position
        # (HALT zeroing, kick suppression, speed cap); only position-based
        # constraints (ball keep-out, and Task 5's area checks) are skipped
        # for it since there is no position to test.
        robot = own_robots.get(cmd.robot_id)
        vx, vy = cmd.vel_x, cmd.vel_y
        vel_angular = cmd.vel_angular
        kick, chip, dribble = cmd.kick_speed, cmd.chip_speed, cmd.dribble
        exempt = cmd.robot_id in exempt_ids

        if phase is Phase.HALT:
            vx = vy = vel_angular = 0.0
            kick = chip = 0.0
            dribble = False
        else:
            push = (
                _keep_out_push(robot, geometry, defend_positive_x, cmd.robot_id == keeper_id)
                if robot is not None
                else None
            )
            if push is not None:
                vx, vy = push
            elif (
                robot is not None
                and ball is not None
                and phase in _BALL_KEEP_OUT_PHASES
                and not exempt
            ):
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
