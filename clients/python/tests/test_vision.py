import ssl_vision_wrapper_pb2

from ssl_client.vision import VisionReceiver, apply_wrapper_packet, parse_wrapper_packet
from ssl_client.world import WorldModel


def _make_wrapper_with_ball(x, y):
    wrapper = ssl_vision_wrapper_pb2.SSL_WrapperPacket()
    wrapper.detection.frame_number = 1
    wrapper.detection.t_capture = 1.0
    wrapper.detection.t_sent = 1.0
    wrapper.detection.camera_id = 0
    ball = wrapper.detection.balls.add()
    ball.confidence = 1.0
    ball.x = x
    ball.y = y
    ball.pixel_x = 0.0
    ball.pixel_y = 0.0
    return wrapper


def test_parse_wrapper_packet_round_trips():
    original = _make_wrapper_with_ball(500.0, 250.0)
    data = original.SerializeToString()

    parsed = parse_wrapper_packet(data)

    assert parsed.detection.balls[0].x == 500.0


def test_apply_wrapper_packet_updates_world_ball():
    world = WorldModel()
    wrapper = _make_wrapper_with_ball(500.0, 250.0)

    apply_wrapper_packet(world, wrapper)

    assert world.ball is not None
    assert world.ball.x == 0.5
    assert world.ball.y == 0.25


def test_apply_wrapper_packet_ignores_absent_geometry():
    world = WorldModel()
    wrapper = _make_wrapper_with_ball(0.0, 0.0)

    apply_wrapper_packet(world, wrapper)

    assert world.geometry is None


def test_vision_receiver_handle_packet_updates_its_world():
    world = WorldModel()
    receiver = VisionReceiver(world)
    data = _make_wrapper_with_ball(100.0, 0.0).SerializeToString()

    receiver.handle_packet(data)

    assert world.ball.x == 0.1
