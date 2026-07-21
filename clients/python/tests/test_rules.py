import math

import ssl_client  # noqa: F401
from ssl_client.commands import RobotCommand
from ssl_client.game_state import GameState, Phase
from ssl_client.rules import (
    BALL_KEEP_OUT_M,
    STOP_SPEED_CAP_MPS,
    apply_rule_constraints,
)
from ssl_client.world import BallObservation, FieldGeometry, RobotObservation


GEOMETRY = FieldGeometry(
    field_length=9.0, field_width=6.0, goal_width=1.0, goal_depth=0.18,
    boundary_width=0.3, penalty_area_depth=1.0, penalty_area_width=2.0,
)


def _robot(rid, x, y):
    return RobotObservation(robot_id=rid, x=x, y=y, orientation=0.0, t_capture=0.0)


def _cmd(rid, vx, vy, kick=0.0):
    return RobotCommand(rid, vx, vy, 0.0, 0.0, kick_speed=kick)


def _apply(commands, state, robots, ball, exempt=frozenset()):
    return apply_rule_constraints(
        commands, state,
        own_robots=robots, ball=ball, geometry=GEOMETRY,
        defend_positive_x=False, keeper_id=0, exempt_ids=exempt,
    )


def test_stop_caps_speed():
    robots = {1: _robot(1, 2.0, 2.0)}
    out = _apply([_cmd(1, 3.0, 0.0)], GameState(Phase.STOP), robots,
                 BallObservation(x=-2.0, y=-2.0, t_capture=0.0))
    assert math.hypot(out[0].vel_x, out[0].vel_y) <= STOP_SPEED_CAP_MPS + 1e-9


def test_stop_pushes_robot_away_from_ball():
    ball = BallObservation(x=0.0, y=0.0, t_capture=0.0)
    robots = {1: _robot(1, 0.2, 0.0)}  # inside 0.5 m keep-out
    out = _apply([_cmd(1, -1.0, 0.0)], GameState(Phase.STOP), robots, ball)
    assert out[0].vel_x > 0  # pushed away (+x), not toward the ball


def test_exempt_kicker_may_approach_ball():
    ball = BallObservation(x=0.0, y=0.0, t_capture=0.0)
    robots = {2: _robot(2, 0.2, 0.0)}
    state = GameState(Phase.FREE_KICK_OURS, may_kick=True)
    out = _apply([_cmd(2, -1.0, 0.0, kick=3.0)], state, robots, ball,
                 exempt=frozenset({2}))
    assert out[0].vel_x == -1.0 and out[0].kick_speed == 3.0


def test_non_exempt_robot_cannot_kick_during_set_piece():
    ball = BallObservation(x=0.0, y=0.0, t_capture=0.0)
    robots = {3: _robot(3, 2.0, 2.0)}
    state = GameState(Phase.FREE_KICK_OURS, may_kick=True)
    out = _apply([_cmd(3, 0.5, 0.0, kick=4.0)], state, robots, ball)
    assert out[0].kick_speed == 0.0


def test_ball_placement_clears_the_corridor():
    ball = BallObservation(x=0.0, y=0.0, t_capture=0.0)
    robots = {1: _robot(1, 1.0, 0.1)}  # 0.1 m from segment (0,0)-(2,0)
    state = GameState(Phase.BALL_PLACEMENT_THEIRS, designated_position=(2.0, 0.0))
    out = _apply([_cmd(1, 0.0, -1.0)], state, robots, ball)
    assert out[0].vel_y > 0  # pushed perpendicular, away from the corridor


def test_halt_zeroes_everything():
    robots = {1: _robot(1, 1.0, 1.0)}
    out = _apply([_cmd(1, 2.0, 2.0, kick=4.0)], GameState(Phase.HALT), robots, None)
    assert (out[0].vel_x, out[0].vel_y, out[0].kick_speed) == (0.0, 0.0, 0.0)


def test_running_passes_commands_through():
    robots = {1: _robot(1, 0.2, 0.0)}
    ball = BallObservation(x=0.0, y=0.0, t_capture=0.0)
    out = _apply([_cmd(1, -1.0, 0.0, kick=4.0)], GameState(Phase.RUNNING, may_kick=True), robots, ball)
    assert out[0].vel_x == -1.0 and out[0].kick_speed == 4.0


def test_untracked_robot_is_still_zeroed_on_halt():
    # robot_id 9 is not in own_robots (e.g. vision dropout); HALT must still
    # zero it out rather than passing the raw command through.
    out = _apply([_cmd(9, 2.0, 2.0, kick=4.0)], GameState(Phase.HALT), {}, None)
    assert (out[0].vel_x, out[0].vel_y, out[0].kick_speed) == (0.0, 0.0, 0.0)


def test_untracked_robot_cannot_kick_during_set_piece():
    ball = BallObservation(x=0.0, y=0.0, t_capture=0.0)
    state = GameState(Phase.FREE_KICK_OURS, may_kick=True)
    out = _apply([_cmd(9, 0.5, 0.0, kick=4.0)], state, {}, ball)
    assert out[0].kick_speed == 0.0


def test_untracked_robot_speed_is_capped_during_stop():
    # Speed capping only needs velocity, not position, so it should still
    # apply to a robot missing from own_robots.
    out = _apply([_cmd(9, 3.0, 0.0)], GameState(Phase.STOP), {}, None)
    assert math.hypot(out[0].vel_x, out[0].vel_y) <= STOP_SPEED_CAP_MPS + 1e-9


def test_non_keeper_is_pushed_out_of_own_defense_area():
    # defend_positive_x=False -> own goal at x=-4.5; own area x in [-4.5, -3.5], y in [-1, 1]
    robots = {1: _robot(1, -4.0, 0.0)}
    out = _apply([_cmd(1, -1.0, 0.0)], GameState(Phase.RUNNING, may_kick=True), robots,
                 BallObservation(x=2.0, y=2.0, t_capture=0.0))
    assert out[0].vel_x > 0  # pushed toward field center (+x)


def test_keeper_may_stay_in_own_defense_area():
    robots = {0: _robot(0, -4.0, 0.0)}  # keeper_id=0 in _apply
    out = _apply([_cmd(0, -0.5, 0.0)], GameState(Phase.RUNNING, may_kick=True), robots,
                 BallObservation(x=2.0, y=2.0, t_capture=0.0))
    assert out[0].vel_x == -0.5  # untouched


def test_everyone_is_pushed_out_of_opponent_defense_area():
    # opponent area: x in [3.5, 4.5], y in [-1, 1]
    robots = {0: _robot(0, 4.0, 0.0)}
    out = _apply([_cmd(0, 1.0, 0.0)], GameState(Phase.RUNNING, may_kick=True), robots,
                 BallObservation(x=2.0, y=2.0, t_capture=0.0))
    assert out[0].vel_x < 0  # keeper exemption does NOT apply to opponent area


def test_robot_outside_field_is_sent_back():
    robots = {1: _robot(1, 5.0, 0.0)}  # beyond +x field edge (4.5)
    out = _apply([_cmd(1, 1.0, 0.0)], GameState(Phase.RUNNING, may_kick=True), robots,
                 BallObservation(x=0.0, y=0.0, t_capture=0.0))
    assert out[0].vel_x < 0
