from dataclasses import dataclass, field
from typing import Dict, Optional

_MM_TO_M = 1.0 / 1000.0


@dataclass
class BallObservation:
    x: float  # meters, field frame
    y: float  # meters, field frame
    t_capture: float


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
        world.ball = BallObservation(
            x=ball.x * _MM_TO_M,
            y=ball.y * _MM_TO_M,
            t_capture=detection.t_capture,
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
