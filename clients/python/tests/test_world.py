import pytest
import ssl_vision_detection_pb2
import ssl_vision_geometry_pb2

from ssl_client.world import WorldModel, update_from_detection_frame, update_from_geometry_data


def _make_detection_frame(ball_xy=None, blue=(), yellow=()):
    frame = ssl_vision_detection_pb2.SSL_DetectionFrame()
    frame.frame_number = 1
    frame.t_capture = 123.456
    frame.t_sent = 123.456
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
