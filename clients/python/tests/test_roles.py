import pytest

from ssl_client.roles import receiver_target, support_target
from ssl_client.world import BallObservation, RobotObservation


def _robot(rid, x, y):
    return RobotObservation(robot_id=rid, x=x, y=y, orientation=0.0, t_capture=0.0)


# --- receiver_target ---

def test_receiver_target_is_closest_point_on_ball_velocity_ray():
    # ボールは原点から +x へ 2 m/s。ロボットは (1.0, 1.0)。
    # レイ上の最近傍点は (1.0, 0.0)。
    ball = BallObservation(x=0.0, y=0.0, t_capture=0.0, vx=2.0, vy=0.0)

    assert receiver_target(1.0, 1.0, ball) == pytest.approx((1.0, 0.0))


def test_receiver_target_clamps_to_ball_position_behind_the_ray():
    # ロボットがボールの進行方向の後ろにいる場合、レイのパラメータ t は
    # 負になるので 0 にクランプ => ボール位置そのものへ向かう。
    ball = BallObservation(x=0.0, y=0.0, t_capture=0.0, vx=2.0, vy=0.0)

    assert receiver_target(-3.0, 0.5, ball) == pytest.approx((0.0, 0.0))


def test_receiver_target_is_ball_position_when_ball_is_slow():
    ball = BallObservation(x=1.5, y=-0.5, t_capture=0.0, vx=0.05, vy=0.0)

    assert receiver_target(0.0, 0.0, ball) == pytest.approx((1.5, -0.5))


# --- support_target ---

def test_support_target_returns_base_slot_when_all_candidates_open():
    # 敵がいなければ全候補 safety 1.0 => 基準スロットに最も近い候補
    # (オフセット (0, 0) = 基準そのもの)が選ばれる。
    base = (-3.0, 0.0)

    assert support_target(base, 1.0, (0.0, 0.0), []) == pytest.approx(base)


def test_support_target_moves_off_blocked_lane():
    # 敵がボール(原点)と基準スロットの間に立ってラインを遮る =>
    # 基準以外の候補(横か前へずれた点)が選ばれる。
    base = (-3.0, 0.0)
    opponents = [_robot(0, -1.5, 0.0)]

    target = support_target(base, 1.0, (0.0, 0.0), opponents)

    assert target != pytest.approx(base)


def test_support_target_prefers_most_open_candidate():
    # 基準 (-3.0, 0.0) へのラインは敵0で遮断、(-3.0, -0.8) へのラインも敵1で
    # やや窮屈。(-3.0, +0.8) 方面だけ完全に開いている => +y 側の候補を選ぶ。
    base = (-3.0, 0.0)
    opponents = [_robot(0, -1.5, 0.0), _robot(1, -1.5, -0.45)]

    target = support_target(base, 1.0, (0.0, 0.0), opponents)

    assert target[1] > 0.0
