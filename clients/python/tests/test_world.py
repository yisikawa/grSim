import pytest
import ssl_vision_detection_pb2
import ssl_vision_geometry_pb2

from ssl_client.world import WorldModel, update_from_detection_frame, update_from_geometry_data


def _make_detection_frame(ball_xy=None, blue=(), yellow=(), t_capture=123.456):
    frame = ssl_vision_detection_pb2.SSL_DetectionFrame()
    frame.frame_number = 1
    frame.t_capture = t_capture
    frame.t_sent = t_capture
    frame.camera_id = 0
    if ball_xy is not None:
        ball = frame.balls.add()
        ball.confidence = 1.0
        ball.x, ball.y = ball_xy
        ball.pixel_x = 0.0
        ball.pixel_y = 0.0
    for robot_id, x, y, orientation in blue:
        robot = frame.robots_blue.add()
        robot.confidence = 1.0
        robot.robot_id = robot_id
        robot.x = x
        robot.y = y
        robot.orientation = orientation
        robot.pixel_x = 0.0
        robot.pixel_y = 0.0
    for robot_id, x, y, orientation in yellow:
        robot = frame.robots_yellow.add()
        robot.confidence = 1.0
        robot.robot_id = robot_id
        robot.x = x
        robot.y = y
        robot.orientation = orientation
        robot.pixel_x = 0.0
        robot.pixel_y = 0.0
    return frame


def test_update_from_detection_frame_converts_mm_to_meters_and_stores_ball():
    world = WorldModel()
    frame = _make_detection_frame(ball_xy=(1000.0, -500.0))

    update_from_detection_frame(world, frame)

    assert world.ball is not None
    assert world.ball.x == 1.0
    assert world.ball.y == -0.5


def test_update_from_detection_frame_stores_robots_by_id_and_color():
    world = WorldModel()
    frame = _make_detection_frame(
        blue=[(0, 2000.0, 0.0, 1.5707963267948966)],
        yellow=[(3, -2000.0, 0.0, 0.0)],
    )

    update_from_detection_frame(world, frame)

    assert 0 in world.blue_robots
    assert world.blue_robots[0].x == 2.0
    assert world.blue_robots[0].orientation == pytest.approx(1.5707963267948966, abs=1e-7)
    assert 3 in world.yellow_robots
    assert world.yellow_robots[3].x == -2.0


def test_update_from_detection_frame_overwrites_previous_observation_for_same_id():
    world = WorldModel()
    update_from_detection_frame(world, _make_detection_frame(blue=[(0, 0.0, 0.0, 0.0)]))
    update_from_detection_frame(world, _make_detection_frame(blue=[(0, 1000.0, 0.0, 0.0)]))

    assert world.blue_robots[0].x == 1.0


def test_update_from_geometry_data_converts_mm_to_meters():
    world = WorldModel()
    geometry = ssl_vision_geometry_pb2.SSL_GeometryData()
    geometry.field.field_length = 9000
    geometry.field.field_width = 6000
    geometry.field.goal_width = 1000
    geometry.field.goal_depth = 180
    geometry.field.boundary_width = 300

    update_from_geometry_data(world, geometry)

    assert world.geometry is not None
    assert world.geometry.field_length == 9.0
    assert world.geometry.field_width == 6.0
    assert world.geometry.goal_width == 1.0
    assert world.geometry.goal_depth == 0.18
    assert world.geometry.boundary_width == 0.3


def test_first_ball_observation_has_zero_velocity():
    world = WorldModel()
    update_from_detection_frame(world, _make_detection_frame(ball_xy=(0.0, 0.0), t_capture=0.0))

    assert world.ball.vx == 0.0
    assert world.ball.vy == 0.0


def test_ball_velocity_is_ema_smoothed_finite_difference():
    world = WorldModel()
    # 0.1 s ごとに +x へ 0.1 m 移動 => 生の速度 1.0 m/s
    update_from_detection_frame(world, _make_detection_frame(ball_xy=(0.0, 0.0), t_capture=0.0))
    update_from_detection_frame(world, _make_detection_frame(ball_xy=(100.0, 0.0), t_capture=0.1))
    # EMA(alpha=0.5): 0.5 * 1.0 + 0.5 * 0.0 = 0.5
    assert world.ball.vx == pytest.approx(0.5)
    assert world.ball.vy == pytest.approx(0.0)

    update_from_detection_frame(world, _make_detection_frame(ball_xy=(200.0, 0.0), t_capture=0.2))
    # 0.5 * 1.0 + 0.5 * 0.5 = 0.75
    assert world.ball.vx == pytest.approx(0.75)


def test_ball_velocity_held_when_dt_is_zero_or_frame_gap_too_large():
    from ssl_client.world import BallObservation, estimate_ball_velocity

    prev = BallObservation(x=0.0, y=0.0, t_capture=1.0, vx=0.5, vy=-0.2)
    # dt = 0
    assert estimate_ball_velocity(prev, 1.0, 1.0, 1.0) == (0.5, -0.2)
    # dt < 0(順序が乱れたフレーム)
    assert estimate_ball_velocity(prev, 1.0, 1.0, 0.5) == (0.5, -0.2)
    # dt > 0.5 s(フレーム落ち)
    assert estimate_ball_velocity(prev, 1.0, 1.0, 2.0) == (0.5, -0.2)


def test_estimate_ball_velocity_none_prev_returns_zero():
    from ssl_client.world import estimate_ball_velocity

    assert estimate_ball_velocity(None, 1.0, 2.0, 0.0) == (0.0, 0.0)


class _FakeFieldWithPenalty:
    def __init__(self):
        self.field_length = 12000
        self.field_width = 9000
        self.goal_width = 1800
        self.goal_depth = 180
        self.boundary_width = 300
        self._has_penalty = True
        self.penalty_area_depth = 1800
        self.penalty_area_width = 3600

    def HasField(self, name):
        return self._has_penalty


class _FakeGeometryPacket:
    def __init__(self, field):
        self.field = field


def test_geometry_uses_penalty_area_from_packet():
    from ssl_client.world import WorldModel, update_from_geometry_data

    world = WorldModel()
    update_from_geometry_data(world, _FakeGeometryPacket(_FakeFieldWithPenalty()))
    assert world.geometry.penalty_area_depth == 1.8
    assert world.geometry.penalty_area_width == 3.6


def test_geometry_falls_back_to_division_b_defaults():
    from ssl_client.world import WorldModel, update_from_geometry_data

    field = _FakeFieldWithPenalty()
    field._has_penalty = False
    world = WorldModel()
    update_from_geometry_data(world, _FakeGeometryPacket(field))
    assert world.geometry.penalty_area_depth == 1.0
    assert world.geometry.penalty_area_width == 2.0
