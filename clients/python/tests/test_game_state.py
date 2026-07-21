import ssl_client  # noqa: F401  (pb/ sys.path bootstrap)
from state import ssl_gc_referee_message_pb2 as referee_pb2

from ssl_client.game_state import Phase, phase_for_command

Cmd = referee_pb2.Referee


def test_halt_maps_to_halt():
    assert phase_for_command(Cmd.HALT, True) is Phase.HALT


def test_stop_timeout_goal_and_unknown_map_to_stop():
    for cmd in (Cmd.STOP, Cmd.TIMEOUT_YELLOW, Cmd.TIMEOUT_BLUE,
                Cmd.GOAL_YELLOW, Cmd.GOAL_BLUE, 999):
        assert phase_for_command(cmd, True) is Phase.STOP


def test_force_start_maps_to_running():
    assert phase_for_command(Cmd.FORCE_START, False) is Phase.RUNNING


def test_team_commands_resolve_ours_theirs_for_yellow():
    assert phase_for_command(Cmd.PREPARE_KICKOFF_YELLOW, True) is Phase.KICKOFF_OURS
    assert phase_for_command(Cmd.PREPARE_KICKOFF_BLUE, True) is Phase.KICKOFF_THEIRS
    assert phase_for_command(Cmd.PREPARE_PENALTY_YELLOW, True) is Phase.PENALTY_OURS
    assert phase_for_command(Cmd.DIRECT_FREE_YELLOW, True) is Phase.FREE_KICK_OURS
    assert phase_for_command(Cmd.INDIRECT_FREE_BLUE, True) is Phase.FREE_KICK_THEIRS
    assert phase_for_command(Cmd.BALL_PLACEMENT_BLUE, True) is Phase.BALL_PLACEMENT_THEIRS


def test_team_commands_resolve_ours_theirs_for_blue():
    assert phase_for_command(Cmd.PREPARE_KICKOFF_YELLOW, False) is Phase.KICKOFF_THEIRS
    assert phase_for_command(Cmd.BALL_PLACEMENT_BLUE, False) is Phase.BALL_PLACEMENT_OURS


def test_normal_start_inherits_prepare_phase():
    assert phase_for_command(Cmd.NORMAL_START, True, Phase.KICKOFF_OURS) is Phase.KICKOFF_OURS
    assert phase_for_command(Cmd.NORMAL_START, True, Phase.PENALTY_THEIRS) is Phase.PENALTY_THEIRS


def test_normal_start_without_prepare_falls_back_to_running():
    assert phase_for_command(Cmd.NORMAL_START, True, Phase.STOP) is Phase.RUNNING


from ssl_client.game_state import GameState, GameStateTracker
from ssl_client.world import BallObservation


def _msg(command, counter, designated=None):
    m = referee_pb2.Referee()
    m.command = command
    m.command_counter = counter
    # Required fields must be set for a valid message object.
    m.packet_timestamp = 0
    m.stage = referee_pb2.Referee.NORMAL_FIRST_HALF
    m.command_timestamp = 0
    m.yellow.name = "y"
    m.yellow.score = 0
    m.yellow.red_cards = 0
    m.yellow.yellow_cards = 0
    m.yellow.timeouts = 0
    m.yellow.timeout_time = 0
    m.yellow.goalkeeper = 0
    m.blue.CopyFrom(m.yellow)
    m.blue.name = "b"
    if designated is not None:
        m.designated_position.x = designated[0]
        m.designated_position.y = designated[1]
    return m


def _ball(x, y):
    return BallObservation(x=x, y=y, t_capture=0.0)


def test_no_referee_message_means_free_play():
    tracker = GameStateTracker(is_team_yellow=True)
    state = tracker.update(None, _ball(0, 0))
    assert state == GameState(Phase.RUNNING, may_kick=True)


def test_normal_start_arms_pending_kickoff():
    tracker = GameStateTracker(is_team_yellow=True)
    s1 = tracker.update(_msg(Cmd.PREPARE_KICKOFF_YELLOW, 1), _ball(0, 0))
    assert s1.phase is Phase.KICKOFF_OURS and s1.may_kick is False
    s2 = tracker.update(_msg(Cmd.NORMAL_START, 2), _ball(0, 0))
    assert s2.phase is Phase.KICKOFF_OURS and s2.may_kick is True


def test_free_kick_is_armed_immediately():
    tracker = GameStateTracker(is_team_yellow=True)
    state = tracker.update(_msg(Cmd.DIRECT_FREE_YELLOW, 5), _ball(1, 1))
    assert state.phase is Phase.FREE_KICK_OURS and state.may_kick is True


def test_set_piece_transitions_to_running_once_ball_moves():
    tracker = GameStateTracker(is_team_yellow=True)
    tracker.update(_msg(Cmd.PREPARE_KICKOFF_YELLOW, 1), _ball(0, 0))
    tracker.update(_msg(Cmd.NORMAL_START, 2), _ball(0, 0))
    # ball still within 5 cm: stay in kickoff
    s = tracker.update(_msg(Cmd.NORMAL_START, 2), _ball(0.03, 0))
    assert s.phase is Phase.KICKOFF_OURS
    s = tracker.update(_msg(Cmd.NORMAL_START, 2), _ball(0.2, 0))
    assert s.phase is Phase.RUNNING and s.may_kick is True


def test_ball_placement_carries_designated_position_in_meters():
    tracker = GameStateTracker(is_team_yellow=True)
    state = tracker.update(_msg(Cmd.BALL_PLACEMENT_BLUE, 3, designated=(1500, -500)), _ball(0, 0))
    assert state.phase is Phase.BALL_PLACEMENT_THEIRS
    assert state.designated_position == (1.5, -0.5)


def test_stop_command_clears_set_piece():
    tracker = GameStateTracker(is_team_yellow=True)
    tracker.update(_msg(Cmd.DIRECT_FREE_YELLOW, 1), _ball(0, 0))
    state = tracker.update(_msg(Cmd.STOP, 2), _ball(0, 0))
    assert state.phase is Phase.STOP and state.may_kick is False
