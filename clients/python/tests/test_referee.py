from state import ssl_gc_referee_message_pb2 as referee_pb2

from ssl_client.referee import RefereeReceiver, is_match_running, parse_referee_packet


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


def test_is_match_running_false_during_halt_and_stop():
    assert is_match_running(_make_referee(referee_pb2.Referee.HALT)) is False
    assert is_match_running(_make_referee(referee_pb2.Referee.STOP)) is False


def test_is_match_running_true_during_normal_start_and_force_start():
    assert is_match_running(_make_referee(referee_pb2.Referee.NORMAL_START)) is True
    assert is_match_running(_make_referee(referee_pb2.Referee.FORCE_START)) is True


def test_parse_referee_packet_round_trips():
    data = _make_referee(referee_pb2.Referee.FORCE_START).SerializeToString()

    parsed = parse_referee_packet(data)

    assert parsed.command == referee_pb2.Referee.FORCE_START


def test_referee_receiver_defaults_to_running_before_any_packet_seen():
    receiver = RefereeReceiver()
    assert receiver.is_running() is True


def test_referee_receiver_reflects_latest_packet():
    receiver = RefereeReceiver()
    receiver.handle_packet(_make_referee(referee_pb2.Referee.HALT).SerializeToString())
    assert receiver.is_running() is False

    receiver.handle_packet(_make_referee(referee_pb2.Referee.FORCE_START).SerializeToString())
    assert receiver.is_running() is True
