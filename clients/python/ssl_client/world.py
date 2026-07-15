from dataclasses import dataclass, field
from typing import Dict, Optional

_MM_TO_M = 1.0 / 1000.0

_VELOCITY_EMA_ALPHA = 0.5
_MAX_FRAME_GAP_S = 0.5  # beyond this, treat as a dropped-frame gap and hold velocity


def estimate_ball_velocity(prev, x: float, y: float, t_capture: float):
    """Finite-difference ball velocity from the previous observation,
    EMA-smoothed. Holds the previous velocity across bad dt (<= 0 or
    dropped frames). Returns (0.0, 0.0) for the first observation."""
    if prev is None:
        return 0.0, 0.0
    dt = t_capture - prev.t_capture
    if dt <= 0.0 or dt > _MAX_FRAME_GAP_S:
        return prev.vx, prev.vy
    raw_vx = (x - prev.x) / dt
    raw_vy = (y - prev.y) / dt
    vx = _VELOCITY_EMA_ALPHA * raw_vx + (1.0 - _VELOCITY_EMA_ALPHA) * prev.vx
    vy = _VELOCITY_EMA_ALPHA * raw_vy + (1.0 - _VELOCITY_EMA_ALPHA) * prev.vy
    return vx, vy


@dataclass
class BallObservation:
    x: float  # meters, field frame
    y: float  # meters, field frame
    t_capture: float
    vx: float = 0.0  # m/s, field frame, EMA-smoothed finite difference
    vy: float = 0.0  # m/s


@dataclass
class RobotObservation:
    robot_id: int
    x: float  # meters
    y: float  # meters
    orientation: float  # radians
    t_capture: float


@dataclass
class FieldGeometry:
    field_length: float  # meters
    field_width: float  # meters
    goal_width: float  # meters
    goal_depth: float  # meters
    boundary_width: float  # meters


@dataclass
class WorldModel:
    ball: Optional[BallObservation] = None
    blue_robots: Dict[int, RobotObservation] = field(default_factory=dict)
    yellow_robots: Dict[int, RobotObservation] = field(default_factory=dict)
    geometry: Optional[FieldGeometry] = None


def update_from_detection_frame(world: WorldModel, detection) -> None:
    if detection.balls:
        ball = max(detection.balls, key=lambda b: b.confidence)
        x = ball.x * _MM_TO_M
        y = ball.y * _MM_TO_M
        vx, vy = estimate_ball_velocity(world.ball, x, y, detection.t_capture)
        world.ball = BallObservation(
            x=x, y=y, t_capture=detection.t_capture, vx=vx, vy=vy
        )

    for robot in detection.robots_blue:
        if not robot.HasField("robot_id"):
            continue
        world.blue_robots[robot.robot_id] = RobotObservation(
            robot_id=robot.robot_id,
            x=robot.x * _MM_TO_M,
            y=robot.y * _MM_TO_M,
            orientation=robot.orientation if robot.HasField("orientation") else 0.0,
            t_capture=detection.t_capture,
        )

    for robot in detection.robots_yellow:
        if not robot.HasField("robot_id"):
            continue
        world.yellow_robots[robot.robot_id] = RobotObservation(
            robot_id=robot.robot_id,
            x=robot.x * _MM_TO_M,
            y=robot.y * _MM_TO_M,
            orientation=robot.orientation if robot.HasField("orientation") else 0.0,
            t_capture=detection.t_capture,
        )


def update_from_geometry_data(world: WorldModel, geometry) -> None:
    f = geometry.field
    world.geometry = FieldGeometry(
        field_length=f.field_length * _MM_TO_M,
        field_width=f.field_width * _MM_TO_M,
        goal_width=f.goal_width * _MM_TO_M,
        goal_depth=f.goal_depth * _MM_TO_M,
        boundary_width=f.boundary_width * _MM_TO_M,
    )
