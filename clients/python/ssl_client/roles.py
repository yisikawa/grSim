"""Per-role motion primitives and target computation.

Everything here is a pure function of observations -> velocities/targets,
so it can be unit tested without a running simulator.
"""
import math

from .evaluation import lane_safety

SEEK_GAIN = 2.0
MAX_SPEED = 2.0  # m/s, conservative vs. grSim's configured VelAbsoluteMax=5
SEPARATION_DISTANCE = 0.4  # meters; own robots closer than this get pushed apart
SEPARATION_GAIN = 1.5
ANGLE_GAIN = 3.0
MAX_ANGULAR_SPEED = 4.0  # rad/s, conservative vs. grSim's configured VelAngularMax=20

FORMATION_PUSH_FORWARD = 0.5  # meters; extra forward shift when the ball is in the attacking half

# (forward_offset, lateral_offset) in meters, relative to the team's own goal
# line, for each non-keeper robot's fallback formation slot (assigned in
# ascending robot-id order, cycling if there are more robots than slots).
FORMATION_SLOTS = [
    (1.0, 0.0),
    (1.0, 1.2),
    (1.0, -1.2),
    (2.5, 0.8),
    (2.5, -0.8),
]


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def seek(current_x: float, current_y: float, target_x: float, target_y: float):
    vx = (target_x - current_x) * SEEK_GAIN
    vy = (target_y - current_y) * SEEK_GAIN
    speed = math.hypot(vx, vy)
    if speed > MAX_SPEED:
        scale = MAX_SPEED / speed
        vx *= scale
        vy *= scale
    return vx, vy


def face(current_orientation: float, target_orientation: float):
    """Returns (vel_angular, angle_error) to rotate from current_orientation
    toward target_orientation, clamped to MAX_ANGULAR_SPEED."""
    error = normalize_angle(target_orientation - current_orientation)
    vel_angular = error * ANGLE_GAIN
    if abs(vel_angular) > MAX_ANGULAR_SPEED:
        vel_angular = math.copysign(MAX_ANGULAR_SPEED, vel_angular)
    return vel_angular, error


def apply_separation(rid, vx: float, vy: float, robot, own_robots: dict):
    """Push a robot's commanded velocity away from own teammates that are
    closer than SEPARATION_DISTANCE, to avoid them stacking on top of each
    other. Not real path planning — just a simple repulsion term."""
    for other_id, other in own_robots.items():
        if other_id == rid:
            continue
        dx = robot.x - other.x
        dy = robot.y - other.y
        dist = math.hypot(dx, dy)
        if 0 < dist < SEPARATION_DISTANCE:
            push = (SEPARATION_DISTANCE - dist) * SEPARATION_GAIN
            vx += (dx / dist) * push
            vy += (dy / dist) * push
    speed = math.hypot(vx, vy)
    if speed > MAX_SPEED:
        scale = MAX_SPEED / speed
        vx *= scale
        vy *= scale
    return vx, vy


def own_goal_x(world, defend_positive_x: bool) -> float:
    half_length = world.geometry.field_length / 2.0
    return half_length if defend_positive_x else -half_length


def goalkeeper_target(world, defend_positive_x: bool):
    half_goal = world.geometry.goal_width / 2.0
    target_y = max(-half_goal, min(half_goal, world.ball.y))
    return own_goal_x(world, defend_positive_x), target_y


def formation_target(world, defend_positive_x: bool, slot_index: int, ball_x: float):
    """Fixed formation slot, relative to this team's own goal line, pushed
    FORMATION_PUSH_FORWARD further forward when the ball is in the
    opponent's half of the field (a fixed, symmetric field split around
    field-center x=0 - not the same axis as the goalkeeper's lateral
    tracking, which follows the ball's Y)."""
    forward_offset, lateral_offset = FORMATION_SLOTS[slot_index % len(FORMATION_SLOTS)]
    forward_sign = -1.0 if defend_positive_x else 1.0
    goal_x = own_goal_x(world, defend_positive_x)
    ball_in_attacking_half = (ball_x * forward_sign) > 0.0
    push_forward = FORMATION_PUSH_FORWARD if ball_in_attacking_half else 0.0
    target_x = goal_x + forward_sign * (forward_offset + push_forward)
    return target_x, lateral_offset


BALL_MOVING_MIN_SPEED_MPS = 0.1  # below this the ball counts as stationary
SUPPORT_CANDIDATE_LATERAL_M = (-0.8, 0.0, 0.8)
SUPPORT_CANDIDATE_FORWARD_M = (0.0, 0.5)


def receiver_target(robot_x: float, robot_y: float, ball):
    """Intercept point for a pass receiver: the closest point to the robot
    on the ray from the ball along its velocity. Falls back to the ball
    position itself when the ball is (nearly) stationary."""
    speed = math.hypot(ball.vx, ball.vy)
    if speed < BALL_MOVING_MIN_SPEED_MPS:
        return ball.x, ball.y
    ux, uy = ball.vx / speed, ball.vy / speed
    t = (robot_x - ball.x) * ux + (robot_y - ball.y) * uy
    t = max(0.0, t)
    return ball.x + t * ux, ball.y + t * uy


def support_target(base_xy, forward_sign: float, ball_xy, opponents):
    """Off-ball repositioning: sample candidate points around the base
    formation slot and pick the one with the most open pass lane from the
    ball. Ties go to the candidate closest to the base slot."""
    opponents = list(opponents)
    best_key = None
    best_candidate = base_xy
    for forward in SUPPORT_CANDIDATE_FORWARD_M:
        for lateral in SUPPORT_CANDIDATE_LATERAL_M:
            candidate = (base_xy[0] + forward_sign * forward, base_xy[1] + lateral)
            safety = lane_safety(ball_xy, candidate, opponents)
            dist_to_base = math.hypot(candidate[0] - base_xy[0], candidate[1] - base_xy[1])
            key = (safety, -dist_to_base)  # maximize safety, then prefer near-base
            if best_key is None or key > best_key:
                best_key = key
                best_candidate = candidate
    return best_candidate
