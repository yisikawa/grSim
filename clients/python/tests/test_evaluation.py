import pytest

from ssl_client.evaluation import Action, choose_action, lane_safety, pass_score
from ssl_client.world import RobotObservation


def _robot(rid, x, y):
    return RobotObservation(robot_id=rid, x=x, y=y, orientation=0.0, t_capture=0.0)


# --- lane_safety ---

def test_lane_safety_is_one_with_no_opponents():
    assert lane_safety((0.0, 0.0), (2.0, 0.0), []) == 1.0


def test_lane_safety_is_zero_with_opponent_on_the_line():
    opponents = [_robot(0, 1.0, 0.0)]
    assert lane_safety((0.0, 0.0), (2.0, 0.0), opponents) == 0.0


def test_lane_safety_scales_with_perpendicular_distance():
    # 敵が線分の脇 0.25 m => 0.25 / 0.5(CAP) = 0.5
    opponents = [_robot(0, 1.0, 0.25)]
    assert lane_safety((0.0, 0.0), (2.0, 0.0), opponents) == pytest.approx(0.5)


def test_lane_safety_saturates_at_cap_distance():
    opponents = [_robot(0, 1.0, 3.0)]  # 0.5 m より十分遠い
    assert lane_safety((0.0, 0.0), (2.0, 0.0), opponents) == 1.0


def test_lane_safety_measures_distance_to_segment_not_infinite_line():
    # 敵は線分の延長線上(to の 2 m 先)。無限直線なら距離 0 だが、
    # 線分としては端点から 2 m 離れている => safety 1.0。
    opponents = [_robot(0, 4.0, 0.0)]
    assert lane_safety((0.0, 0.0), (2.0, 0.0), opponents) == 1.0


# --- pass_score ---

def test_pass_score_prefers_open_lane():
    goal = (4.5, 0.0)
    holder = (0.0, 0.0)
    blocked_mate = (2.0, 0.0)
    open_mate = (2.0, 2.0)
    opponents = [_robot(0, 1.0, 0.0)]  # blocked_mate へのラインを遮る

    assert pass_score(holder, open_mate, opponents, goal) > pass_score(
        holder, blocked_mate, opponents, goal
    )


def test_pass_score_penalizes_too_short_and_too_long_passes():
    goal = (4.5, 0.0)
    holder = (0.0, 0.0)
    good = (2.0, 0.0)      # 2.0 m: [0.8, 4.0] の範囲内
    too_short = (0.3, 0.0)  # 0.3 m
    too_long = (0.0, 5.0)   # 5.0 m(前進ボーナスの影響を避けるため真横方向)

    assert pass_score(holder, good, [], goal) > pass_score(holder, too_short, [], goal)
    # 前進ボーナスの差を除いて比較するため、good も真横に置いた版で比べる
    good_lateral = (0.0, 2.0)
    assert pass_score(holder, good_lateral, [], goal) > pass_score(holder, too_long, [], goal)


def test_pass_score_rewards_forward_progress():
    goal = (4.5, 0.0)
    holder = (0.0, 0.0)
    forward_mate = (2.0, 0.0)   # ゴールへ 2.0 m 近づく
    backward_mate = (-2.0, 0.0)  # ゴールから 2.0 m 遠ざかる(同じパス距離)

    assert pass_score(holder, forward_mate, [], goal) > pass_score(
        holder, backward_mate, [], goal
    )


# --- choose_action ---

def test_choose_action_shoots_when_close_to_goal_and_lane_open():
    goal = (4.5, 0.0)
    ball = (3.0, 0.0)  # ゴールまで 1.5 m < 2.5 m、敵なし => shoot
    teammates = {1: _robot(1, 2.0, 1.0)}

    action = choose_action(ball, teammates, [], goal)

    assert action.kind == "shoot"
    assert action.kick_speed == 4.0
    assert (action.target_x, action.target_y) == goal


def test_choose_action_passes_when_shot_is_blocked():
    goal = (4.5, 0.0)
    ball = (3.0, 0.0)
    teammates = {1: _robot(1, 2.0, 1.5)}
    opponents = [_robot(0, 3.7, 0.0)]  # シュートラインを遮る

    action = choose_action(ball, teammates, opponents, goal)

    assert action.kind == "pass"
    assert action.receiver_id == 1


def test_choose_action_passes_when_goal_is_far():
    goal = (4.5, 0.0)
    ball = (0.0, 0.0)  # ゴールまで 4.5 m > 2.5 m => シュート不可
    teammates = {7: _robot(7, 2.0, 0.5)}

    action = choose_action(ball, teammates, [], goal)

    assert action.kind == "pass"
    assert action.receiver_id == 7
    # kick_speed = clamp(パス距離 * 1.5, 1.5, 3.5); 距離 ≈ 2.06 => ≈ 3.09
    assert 1.5 <= action.kick_speed <= 3.5


def test_choose_action_dribbles_when_no_safe_option():
    goal = (4.5, 0.0)
    ball = (0.0, 0.0)
    teammates = {1: _robot(1, 2.0, 0.0)}
    # パスラインもシュートラインも敵だらけ
    opponents = [_robot(0, 1.0, 0.0), _robot(2, 2.5, 0.0), _robot(3, 3.5, 0.0)]

    action = choose_action(ball, teammates, opponents, goal)

    assert action.kind == "dribble"
    assert action.kick_speed == 0.0
    assert (action.target_x, action.target_y) == goal


def test_choose_action_dribbles_when_no_teammates():
    action = choose_action((0.0, 0.0), {}, [], (4.5, 0.0))
    assert action.kind == "dribble"
