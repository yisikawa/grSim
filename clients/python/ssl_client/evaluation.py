"""Pure-function lane/shot evaluation for the passing strategy.

All coordinates are meters in the field (global) frame. All functions are
side-effect free so they can be unit tested with synthetic positions.
"""
import math
from dataclasses import dataclass
from typing import Dict, Iterable, Optional, Tuple

LANE_CAP_M = 0.5  # opponent distance at which a lane counts as fully open
W_DIST = 0.3  # weight of the pass-distance penalty
W_FWD = 0.3  # weight of the forward-progress bonus
PASS_DIST_MIN_M = 0.8
PASS_DIST_MAX_M = 4.0
FWD_NORM_M = 3.0  # forward progress saturates at this many meters gained
SHOOT_SCORE_MIN = 0.8
SHOOT_RANGE_M = 2.5
SHOOT_KICK_SPEED_MPS = 4.0
PASS_SCORE_MIN = 0.3
PASS_KICK_GAIN = 1.5  # kick speed per meter of pass distance
PASS_KICK_MIN_MPS = 1.5
PASS_KICK_MAX_MPS = 3.5


@dataclass
class Action:
    kind: str  # "shoot" | "pass" | "dribble"
    target_x: float  # aim point: goal center (shoot/dribble) or mate (pass)
    target_y: float
    kick_speed: float  # m/s; 0.0 for dribble
    receiver_id: Optional[int] = None  # set only for kind == "pass"


def point_to_segment_distance(
    px: float, py: float, ax: float, ay: float, bx: float, by: float
) -> float:
    abx, aby = bx - ax, by - ay
    ab_len_sq = abx * abx + aby * aby
    if ab_len_sq == 0.0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * abx + (py - ay) * aby) / ab_len_sq
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (ax + t * abx), py - (ay + t * aby))


def lane_safety(from_xy: Tuple[float, float], to_xy: Tuple[float, float], opponents: Iterable) -> float:
    """0..1: how clear the from->to segment is of opponents. 1.0 = wide open."""
    min_dist = None
    for opp in opponents:
        d = point_to_segment_distance(opp.x, opp.y, from_xy[0], from_xy[1], to_xy[0], to_xy[1])
        if min_dist is None or d < min_dist:
            min_dist = d
    if min_dist is None:
        return 1.0
    return min(min_dist, LANE_CAP_M) / LANE_CAP_M


def _dist_penalty(distance: float) -> float:
    if distance < PASS_DIST_MIN_M:
        return PASS_DIST_MIN_M - distance
    if distance > PASS_DIST_MAX_M:
        return distance - PASS_DIST_MAX_M
    return 0.0


def _forward_progress(
    holder_xy: Tuple[float, float], mate_xy: Tuple[float, float], goal_xy: Tuple[float, float]
) -> float:
    holder_to_goal = math.hypot(goal_xy[0] - holder_xy[0], goal_xy[1] - holder_xy[1])
    mate_to_goal = math.hypot(goal_xy[0] - mate_xy[0], goal_xy[1] - mate_xy[1])
    return max(0.0, min(1.0, (holder_to_goal - mate_to_goal) / FWD_NORM_M))


def pass_score(
    holder_xy: Tuple[float, float],
    mate_xy: Tuple[float, float],
    opponents: Iterable,
    goal_xy: Tuple[float, float],
) -> float:
    distance = math.hypot(mate_xy[0] - holder_xy[0], mate_xy[1] - holder_xy[1])
    return (
        lane_safety(holder_xy, mate_xy, opponents)
        - W_DIST * _dist_penalty(distance)
        + W_FWD * _forward_progress(holder_xy, mate_xy, goal_xy)
    )


def choose_action(
    ball_xy: Tuple[float, float],
    teammates: Dict[int, object],
    opponents: Iterable,
    goal_xy: Tuple[float, float],
) -> Action:
    """Pass-first action selection for the ball holder.

    `teammates` must already exclude the goalkeeper and the holder itself.
    `opponents` may be any iterable of objects with .x/.y (re-iterated, so
    pass a list, not a generator).
    """
    opponents = list(opponents)
    goal_dist = math.hypot(goal_xy[0] - ball_xy[0], goal_xy[1] - ball_xy[1])
    if lane_safety(ball_xy, goal_xy, opponents) > SHOOT_SCORE_MIN and goal_dist < SHOOT_RANGE_M:
        return Action("shoot", goal_xy[0], goal_xy[1], SHOOT_KICK_SPEED_MPS)

    best_id = None
    best_score = -math.inf
    for rid, mate in teammates.items():
        score = pass_score(ball_xy, (mate.x, mate.y), opponents, goal_xy)
        if score > best_score:
            best_id, best_score = rid, score
    if best_id is not None and best_score > PASS_SCORE_MIN:
        mate = teammates[best_id]
        distance = math.hypot(mate.x - ball_xy[0], mate.y - ball_xy[1])
        kick = max(PASS_KICK_MIN_MPS, min(PASS_KICK_MAX_MPS, distance * PASS_KICK_GAIN))
        return Action("pass", mate.x, mate.y, kick, receiver_id=best_id)

    return Action("dribble", goal_xy[0], goal_xy[1], 0.0)
