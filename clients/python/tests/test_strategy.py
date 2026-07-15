from ssl_client.strategy import TeamConfig, TeamStrategy
from ssl_client.world import BallObservation, FieldGeometry, RobotObservation, WorldModel


def _world(ball_xy, blue=(), yellow=()):
    world = WorldModel()
    world.ball = BallObservation(x=ball_xy[0], y=ball_xy[1], t_capture=0.0)
    world.geometry = FieldGeometry(
        field_length=9.0, field_width=6.0, goal_width=1.0, goal_depth=0.18, boundary_width=0.3
    )
    for robot_id, x, y, orientation in blue:
        world.blue_robots[robot_id] = RobotObservation(robot_id, x, y, orientation, 0.0)
    for robot_id, x, y, orientation in yellow:
        world.yellow_robots[robot_id] = RobotObservation(robot_id, x, y, orientation, 0.0)
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
    from ssl_client.strategy import _apply_separation
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
    from ssl_client.strategy import _apply_separation
    from ssl_client.world import RobotObservation

    robot = RobotObservation(robot_id=1, x=0.0, y=0.0, orientation=0.0, t_capture=0.0)
    other = RobotObservation(robot_id=2, x=5.0, y=0.0, orientation=0.0, t_capture=0.0)
    own_robots = {1: robot, 2: other}

    vx, vy = _apply_separation(1, 1.0, 0.5, robot, own_robots)

    assert vx == 1.0
    assert vy == 0.5
