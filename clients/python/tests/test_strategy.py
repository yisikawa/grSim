from ssl_client.strategy import TeamConfig, TeamStrategy
from ssl_client.world import BallObservation, FieldGeometry, RobotObservation, WorldModel


def _world(ball_xy, blue=(), yellow=(), ball_v=(0.0, 0.0), t_capture=0.0):
    world = WorldModel()
    world.ball = BallObservation(
        x=ball_xy[0], y=ball_xy[1], t_capture=t_capture, vx=ball_v[0], vy=ball_v[1]
    )
    world.geometry = FieldGeometry(
        field_length=9.0, field_width=6.0, goal_width=1.0, goal_depth=0.18, boundary_width=0.3
    )
    for robot_id, x, y, orientation in blue:
        world.blue_robots[robot_id] = RobotObservation(robot_id, x, y, orientation, t_capture)
    for robot_id, x, y, orientation in yellow:
        world.yellow_robots[robot_id] = RobotObservation(robot_id, x, y, orientation, t_capture)
    return world


def test_returns_no_commands_before_ball_or_geometry_is_known():
    config = TeamConfig(is_team_yellow=False, defend_positive_x=False)
    strategy = TeamStrategy(config)

    empty_world = WorldModel()
    empty_world.blue_robots[0] = RobotObservation(0, 0.0, 0.0, 0.0, 0.0)

    assert strategy.compute_commands(empty_world, referee_running=True) == []


def test_halted_state_returns_zero_velocity_for_every_own_robot():
    config = TeamConfig(is_team_yellow=False, defend_positive_x=False)
    strategy = TeamStrategy(config)
    world = _world(ball_xy=(0.0, 0.0), blue=[(0, -4.0, 0.0, 0.0), (1, 0.0, 0.0, 0.0)])

    commands = strategy.compute_commands(world, referee_running=False)

    assert len(commands) == 2
    for cmd in commands:
        assert cmd.vel_x == 0.0
        assert cmd.vel_y == 0.0
        assert cmd.vel_angular == 0.0
        assert cmd.kick_speed == 0.0


def test_lowest_id_robot_is_permanently_the_goalkeeper():
    config = TeamConfig(is_team_yellow=False, defend_positive_x=False)
    strategy = TeamStrategy(config)
    world = _world(
        ball_xy=(0.0, 0.0),
        blue=[(5, -4.0, 0.0, 0.0), (2, -3.5, 0.0, 0.0), (9, 0.0, 0.0, 0.0)],
    )

    strategy.compute_commands(world, referee_running=True)

    assert strategy._goalkeeper_id == 2


def test_attacker_is_nearest_non_keeper_robot_to_the_ball():
    config = TeamConfig(is_team_yellow=False, defend_positive_x=False)
    strategy = TeamStrategy(config)
    # Keeper (id 0) stays near goal; robot 1 is far from the ball, robot 2 is close.
    world = _world(
        ball_xy=(1.0, 0.0),
        blue=[(0, -4.4, 0.0, 0.0), (1, -3.0, 2.0, 0.0), (2, 0.9, 0.0, 0.0)],
    )

    commands = strategy.compute_commands(world, referee_running=True)

    by_id = {c.robot_id: c for c in commands}
    # The attacker (robot 2) should be driving toward the ball: positive vel_x.
    assert by_id[2].vel_x > 0
    # A non-attacking, non-keeper robot should not be trying to kick.
    assert by_id[1].kick_speed == 0.0


def test_attacker_rotates_to_face_the_opponent_goal():
    config = TeamConfig(is_team_yellow=False, defend_positive_x=False)
    strategy = TeamStrategy(config)
    # This team defends -X (field_length=9.0 => own goal at x=-4.5), so the
    # opponent goal is at x=+4.5. Attacker (id 1) is facing +Y (orientation
    # pi/2), which is 90 degrees off from facing the opponent goal from its
    # position - it must rotate, i.e. a nonzero vel_angular.
    world = _world(
        ball_xy=(1.0, 0.0),
        blue=[(0, -4.4, 0.0, 0.0), (1, 0.9, 0.0, 1.5707963267948966)],
    )

    commands = strategy.compute_commands(world, referee_running=True)

    attacker_cmd = next(c for c in commands if c.robot_id == 1)
    # Facing +Y and needing to face +X (toward the opponent goal) means
    # rotating clockwise, i.e. negative angular velocity.
    assert attacker_cmd.vel_angular < 0


def test_attacker_kicks_when_close_to_ball_and_facing_the_opponent_goal():
    config = TeamConfig(is_team_yellow=False, defend_positive_x=False)
    strategy = TeamStrategy(config)
    # Attacker (id 1) at (4.0, 0.0) already faces +X (orientation 0.0), which
    # points straight at the opponent goal (x=+4.5) from this position, and
    # the ball is 5cm away (well within the 0.15m kick range).
    world = _world(
        ball_xy=(4.05, 0.0),
        blue=[(0, -4.4, 0.0, 0.0), (1, 4.0, 0.0, 0.0)],
    )

    commands = strategy.compute_commands(world, referee_running=True)

    attacker_cmd = next(c for c in commands if c.robot_id == 1)
    assert attacker_cmd.kick_speed > 0.0


def test_attacker_does_not_kick_when_close_but_not_facing_the_opponent_goal():
    config = TeamConfig(is_team_yellow=False, defend_positive_x=False)
    strategy = TeamStrategy(config)
    # Same positions as the previous test (ball within kick range), but the
    # attacker faces +Y instead of the opponent goal.
    world = _world(
        ball_xy=(4.05, 0.0),
        blue=[(0, -4.4, 0.0, 0.0), (1, 4.0, 0.0, 1.5707963267948966)],
    )

    commands = strategy.compute_commands(world, referee_running=True)

    attacker_cmd = next(c for c in commands if c.robot_id == 1)
    assert attacker_cmd.kick_speed == 0.0


def test_formation_robot_shifts_forward_when_ball_is_in_attacking_half():
    config = TeamConfig(is_team_yellow=False, defend_positive_x=False)
    strategy = TeamStrategy(config)
    # This team defends -X (own goal at x=-4.5). Formation slot 0 is
    # (forward_offset=1.0, lateral_offset=0.0) => base target_x = -3.5 with
    # no push. Robot 1 sits exactly at that no-push baseline; robot 2 sits
    # right on the ball so it is unambiguously the attacker, leaving robot 1
    # in the formation branch. The ball at x=4.0 is in the attacking half
    # (positive X), so robot 1 should be pushed further forward (target_x
    # becomes -3.0), i.e. positive vel_x.
    world = _world(
        ball_xy=(4.0, 0.0),
        blue=[(0, -4.4, 0.0, 0.0), (1, -3.5, 0.0, 0.0), (2, 3.99, 0.0, 0.0)],
    )

    commands = strategy.compute_commands(world, referee_running=True)

    formation_cmd = next(c for c in commands if c.robot_id == 1)
    assert formation_cmd.vel_x > 0


def test_goalkeeper_targets_own_goal_line_and_tracks_ball_y_within_goal_width():
    config = TeamConfig(is_team_yellow=False, defend_positive_x=False)
    strategy = TeamStrategy(config)
    # Ball is far outside the goal width (goal_width=1.0 => half-width 0.5);
    # keeper (lowest id, robot 0) should move toward +y (goal half-width),
    # not all the way to the ball's y=2.0.
    world = _world(ball_xy=(0.0, 2.0), blue=[(0, -4.4, 0.0, 0.0), (1, 0.0, 0.0, 0.0)])

    commands = strategy.compute_commands(world, referee_running=True)

    keeper_cmd = next(c for c in commands if c.robot_id == 0)
    assert keeper_cmd.vel_y > 0  # moves toward positive y, following the ball
    # but capped: the goalkeeper's own robot.y (0.0) is already within the
    # commanded direction, and it must not aim past the goal half-width.
    assert keeper_cmd.vel_y <= 2.0 * 0.5 + 1e-6  # seek gain (2.0) * max target offset (half goal width)


def test_apply_separation_pushes_robots_apart_when_too_close():
    # Isolated unit test of the repulsion term itself (rather than going
    # through compute_commands' role assignment, where the seek target
    # differences between roles would swamp the separation effect and make
    # the assertion pass even with separation deleted).
    from ssl_client.roles import apply_separation as _apply_separation
    from ssl_client.world import RobotObservation

    robot = RobotObservation(robot_id=1, x=0.0, y=0.0, orientation=0.0, t_capture=0.0)
    other = RobotObservation(robot_id=2, x=0.05, y=0.0, orientation=0.0, t_capture=0.0)
    own_robots = {1: robot, 2: other}

    # Robot 1's own commanded velocity is zero; the other robot is 5cm away
    # (well within the 0.4m separation distance) at higher x, so separation
    # alone should push robot 1 toward -x.
    vx, vy = _apply_separation(1, 0.0, 0.0, robot, own_robots)

    assert vx < 0.0
    assert vy == 0.0


def test_apply_separation_does_nothing_when_robots_are_far_apart():
    from ssl_client.roles import apply_separation as _apply_separation
    from ssl_client.world import RobotObservation

    robot = RobotObservation(robot_id=1, x=0.0, y=0.0, orientation=0.0, t_capture=0.0)
    other = RobotObservation(robot_id=2, x=5.0, y=0.0, orientation=0.0, t_capture=0.0)
    own_robots = {1: robot, 2: other}

    vx, vy = _apply_separation(1, 1.0, 0.5, robot, own_robots)

    assert vx == 1.0
    assert vy == 0.5


# --- passing state machine ---


def _blue_strategy():
    return TeamStrategy(TeamConfig(is_team_yellow=False, defend_positive_x=False))


def test_state_is_chase_when_nobody_holds_the_ball():
    strategy = _blue_strategy()
    world = _world(ball_xy=(2.0, 0.0), blue=[(0, -4.4, 0.0, 0.0), (1, -1.0, 0.0, 0.0)])

    strategy.compute_commands(world, referee_running=True)

    assert strategy._state == "CHASE"


def test_state_becomes_possess_when_a_robot_holds_a_slow_ball():
    strategy = _blue_strategy()
    world = _world(ball_xy=(1.0, 0.0), blue=[(0, -4.4, 0.0, 0.0), (1, 0.95, 0.0, 0.0)])

    strategy.compute_commands(world, referee_running=True)

    assert strategy._state == "POSSESS"


def test_fast_ball_nearby_does_not_count_as_possession():
    strategy = _blue_strategy()
    world = _world(
        ball_xy=(1.0, 0.0),
        blue=[(0, -4.4, 0.0, 0.0), (1, 0.95, 0.0, 0.0)],
        ball_v=(1.5, 0.0),
    )

    strategy.compute_commands(world, referee_running=True)

    assert strategy._state == "CHASE"


def test_holder_kicks_a_pass_toward_open_mate_and_enters_pass_in_flight():
    strategy = _blue_strategy()
    # ホルダー(id 1)はボールを保持し、味方(id 2)は +y 方向 2 m(良い距離、
    # 敵なし => パスコース全開)。ゴール(x=+4.5)までは 4.5 m > 2.5 m なので
    # シュートは選ばれない。ホルダーは既に +y(pi/2)を向いている => 即キック。
    world = _world(
        ball_xy=(0.0, 0.05),
        blue=[(0, -4.4, 0.0, 0.0), (1, 0.0, 0.0, 1.5707963267948966), (2, 0.0, 2.0, 0.0)],
    )

    commands = strategy.compute_commands(world, referee_running=True)

    holder_cmd = next(c for c in commands if c.robot_id == 1)
    assert 1.5 <= holder_cmd.kick_speed <= 3.5  # パス強度(シュートの 4.0 ではない)
    assert strategy._state == "PASS_IN_FLIGHT"
    assert strategy._receiver_id == 2


def test_receiver_plays_support_on_the_kick_tick_itself():
    strategy = _blue_strategy()
    # Same pass-kick scenario as above. The PASS_IN_FLIGHT transition happens
    # at the END of the kick tick, so on that tick the designated receiver
    # (id 2) must still get a plain support command (support commands never
    # set dribble; receiver/chaser commands do) — deterministically,
    # regardless of robot-dict iteration order.
    world = _world(
        ball_xy=(0.0, 0.05),
        blue=[(0, -4.4, 0.0, 0.0), (1, 0.0, 0.0, 1.5707963267948966), (2, 0.0, 2.0, 0.0)],
    )

    commands = strategy.compute_commands(world, referee_running=True)

    assert strategy._state == "PASS_IN_FLIGHT"  # visible after the tick
    receiver_cmd = next(c for c in commands if c.robot_id == 2)
    assert receiver_cmd.dribble is False


def test_holder_does_not_kick_before_facing_the_pass_target():
    strategy = _blue_strategy()
    # 同じ配置だがホルダーは +x(0.0)を向いている: 味方は +y 方向なので
    # 角度誤差 pi/2 > 0.35 => まず旋回、キックしない。
    world = _world(
        ball_xy=(0.0, 0.05),
        blue=[(0, -4.4, 0.0, 0.0), (1, 0.0, 0.0, 0.0), (2, 0.0, 2.0, 0.0)],
    )

    commands = strategy.compute_commands(world, referee_running=True)

    holder_cmd = next(c for c in commands if c.robot_id == 1)
    assert holder_cmd.kick_speed == 0.0
    assert holder_cmd.vel_angular > 0.0  # +x から +y へは反時計回り
    assert strategy._state == "POSSESS"


def test_holder_shoots_when_near_goal_with_open_lane():
    strategy = _blue_strategy()
    # ボール(とホルダー)はゴール(+4.5, 0)まで 1.0 m、敵なし、ゴール正面向き。
    world = _world(
        ball_xy=(3.5, 0.0),
        blue=[(0, -4.4, 0.0, 0.0), (1, 3.45, 0.0, 0.0), (2, 2.0, 1.5, 0.0)],
    )

    commands = strategy.compute_commands(world, referee_running=True)

    holder_cmd = next(c for c in commands if c.robot_id == 1)
    assert holder_cmd.kick_speed == 4.0
    # シュートはパスではないので PASS_IN_FLIGHT に入らない
    assert strategy._state == "POSSESS"


def test_holder_prefers_pass_when_shot_lane_is_blocked():
    strategy = _blue_strategy()
    # 同じくゴールまで 1.0 m だが、敵がシュートラインを塞ぐ。
    # 味方(id 2)へのラインは開いている => パスを選ぶ。
    world = _world(
        ball_xy=(3.5, 0.0),
        blue=[(0, -4.4, 0.0, 0.0), (1, 3.45, 0.0, 0.0), (2, 2.0, 1.5, 0.0)],
        yellow=[(0, 4.0, 0.0, 0.0)],
    )

    commands = strategy.compute_commands(world, referee_running=True)

    holder_cmd = next(c for c in commands if c.robot_id == 1)
    assert holder_cmd.kick_speed != 4.0  # シュートではない(0 か パス強度)
    # 蹴れる向きならパス強度、向きが合うまでは 0 — どちらでもシュートでなければよい


def test_receiver_intercepts_moving_ball_during_pass_in_flight():
    strategy = _blue_strategy()
    strategy._goalkeeper_id = 0
    strategy._state = "PASS_IN_FLIGHT"
    strategy._receiver_id = 2
    strategy._pass_kick_time = 0.0
    # ボールは原点から +x へ 2 m/s。受け手(id 2)は (1.0, 1.0) にいる =>
    # インターセプト点 (1.0, 0.0) へ向かう(-y 方向の速度)。
    world = _world(
        ball_xy=(0.0, 0.0),
        blue=[(0, -4.4, 0.0, 0.0), (1, -1.0, -2.0, 0.0), (2, 1.0, 1.0, 0.0)],
        ball_v=(2.0, 0.0),
        t_capture=0.1,
    )

    commands = strategy.compute_commands(world, referee_running=True)

    receiver_cmd = next(c for c in commands if c.robot_id == 2)
    assert receiver_cmd.vel_y < 0.0
    assert strategy._state == "PASS_IN_FLIGHT"  # まだ飛行中(速いボール、時間内)


def test_pass_in_flight_times_out_back_to_chase():
    strategy = _blue_strategy()
    strategy._goalkeeper_id = 0
    strategy._state = "PASS_IN_FLIGHT"
    strategy._receiver_id = 2
    strategy._pass_kick_time = 0.0
    world = _world(
        ball_xy=(2.0, 0.0),
        blue=[(0, -4.4, 0.0, 0.0), (1, -1.0, -2.0, 0.0), (2, 1.0, 1.0, 0.0)],
        ball_v=(2.0, 0.0),
        t_capture=2.1,  # 2.0 s のタイムアウト超過
    )

    strategy.compute_commands(world, referee_running=True)

    assert strategy._state == "CHASE"


def test_pass_in_flight_ends_when_ball_slows_after_min_flight():
    strategy = _blue_strategy()
    strategy._goalkeeper_id = 0
    strategy._state = "PASS_IN_FLIGHT"
    strategy._receiver_id = 2
    strategy._pass_kick_time = 0.0
    world = _world(
        ball_xy=(2.0, 0.0),
        blue=[(0, -4.4, 0.0, 0.0), (1, -1.0, -2.0, 0.0), (2, 1.0, 1.0, 0.0)],
        ball_v=(0.1, 0.0),  # 減速済み
        t_capture=0.5,  # min-flight 0.3 s は経過済み
    )

    strategy.compute_commands(world, referee_running=True)

    assert strategy._state == "CHASE"


def test_pass_in_flight_survives_slow_ball_within_min_flight_window():
    strategy = _blue_strategy()
    strategy._goalkeeper_id = 0
    strategy._state = "PASS_IN_FLIGHT"
    strategy._receiver_id = 2
    strategy._pass_kick_time = 0.0
    # キック直後(0.1 s < 0.3 s)は Vision がまだ遅いボールを報告していても
    # PASS_IN_FLIGHT を維持する。
    world = _world(
        ball_xy=(0.1, 0.0),
        blue=[(0, -4.4, 0.0, 0.0), (1, -1.0, -2.0, 0.0), (2, 1.0, 1.0, 0.0)],
        ball_v=(0.0, 0.0),
        t_capture=0.1,
    )

    strategy.compute_commands(world, referee_running=True)

    assert strategy._state == "PASS_IN_FLIGHT"


def test_pass_in_flight_ends_when_receiver_disappears():
    strategy = _blue_strategy()
    strategy._goalkeeper_id = 0
    strategy._state = "PASS_IN_FLIGHT"
    strategy._receiver_id = 9  # 視界にいない
    strategy._pass_kick_time = 0.0
    world = _world(
        ball_xy=(2.0, 0.0),
        blue=[(0, -4.4, 0.0, 0.0), (1, -1.0, -2.0, 0.0)],
        ball_v=(2.0, 0.0),
        t_capture=0.1,
    )

    strategy.compute_commands(world, referee_running=True)

    assert strategy._state == "CHASE"


def test_halt_resets_state_machine_to_chase():
    strategy = _blue_strategy()
    strategy._state = "PASS_IN_FLIGHT"
    strategy._receiver_id = 2
    strategy._pass_kick_time = 0.0
    world = _world(ball_xy=(0.0, 0.0), blue=[(0, -4.0, 0.0, 0.0), (2, 0.0, 1.0, 0.0)])

    commands = strategy.compute_commands(world, referee_running=False)

    assert strategy._state == "CHASE"
    assert strategy._receiver_id is None
    assert all(c.vel_x == 0.0 and c.vel_y == 0.0 for c in commands)


def test_holder_dribbles_toward_goal_when_no_pass_or_shot_available():
    strategy = _blue_strategy()
    # 味方はキーパーのみ(パス候補なし)、ゴールまで 4.5 m(シュート不可)
    # => ドリブル前進: ドリブラー ON、キックなし、+x 方向の速度。
    world = _world(
        ball_xy=(0.05, 0.0),
        blue=[(0, -4.4, 0.0, 0.0), (1, 0.0, 0.0, 0.0)],
    )

    commands = strategy.compute_commands(world, referee_running=True)

    holder_cmd = next(c for c in commands if c.robot_id == 1)
    assert holder_cmd.kick_speed == 0.0
    assert holder_cmd.dribble is True
    assert holder_cmd.vel_x > 0.0


def test_chaser_faces_and_seeks_the_ball_in_chase_state():
    strategy = _blue_strategy()
    # ボールは遠い(保持なし)。チェイサー(id 1)はボールと逆(-x)を向いている
    # => 回転が必要。ボール方向(+x)への移動速度も出る。
    world = _world(
        ball_xy=(2.0, 0.0),
        blue=[(0, -4.4, 0.0, 0.0), (1, 0.0, 0.0, 3.14159)],
    )

    commands = strategy.compute_commands(world, referee_running=True)

    chaser_cmd = next(c for c in commands if c.robot_id == 1)
    assert chaser_cmd.vel_x > 0.0
    assert chaser_cmd.vel_angular != 0.0
    assert chaser_cmd.kick_speed == 0.0
