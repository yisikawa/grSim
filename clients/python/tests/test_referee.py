from state import ssl_gc_referee_message_pb2 as referee_pb2

from ssl_client.referee import RefereeReceiver, parse_referee_packet


def _make_referee(command):
    msg = referee_pb2.Referee()
    msg.packet_timestamp = 0
    msg.stage = referee_pb2.Referee.NORMAL_FIRST_HALF
    msg.command = command
    msg.command_counter = 0
    msg.command_timestamp = 0
    for team in (msg.yellow, msg.blue):
        team.name = ""
        team.score = 0
        team.red_cards = 0
        team.yellow_cards = 0
        team.timeouts = 0
        team.timeout_time = 0
        team.goalkeeper = 0
    return msg


def test_parse_referee_packet_round_trips():
    data = _make_referee(referee_pb2.Referee.FORCE_START).SerializeToString()

    parsed = parse_referee_packet(data)

    assert parsed.command == referee_pb2.Referee.FORCE_START


def test_receiver_starts_with_no_message():
    receiver = RefereeReceiver()
    assert receiver.latest is None


def test_handle_packet_updates_latest():
    msg = _make_referee(referee_pb2.Referee.STOP)
    msg.command_counter = 7
    raw = msg.SerializeToString()

    receiver = RefereeReceiver()
    receiver.handle_packet(raw)

    assert receiver.latest.command == referee_pb2.Referee.STOP
    assert receiver.latest.command_counter == 7
