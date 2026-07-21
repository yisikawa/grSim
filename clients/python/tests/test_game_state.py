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
