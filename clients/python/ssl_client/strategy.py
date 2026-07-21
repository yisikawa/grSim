import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from .commands import RobotCommand
from .evaluation import SHOOT_KICK_SPEED_MPS, choose_action
from .game_state import GameState, Phase
from .roles import (
    apply_separation,
    face,
    formation_target,
    goalkeeper_target,
    own_goal_x,
    receiver_target,
    seek,
    support_target,
)
from .world import WorldModel

POSSESSION_DIST_M = 0.12
SLOW_BALL_SPEED_MPS = 0.5
PASS_TIMEOUT_S = 2.0
PASS_MIN_FLIGHT_S = 0.3  # suppress the slow-ball exit right after the kick
KICK_RANGE_M = 0.15
KICK_ANGLE_TOLERANCE = 0.35  # radians (~20 degrees)
DRIBBLE_ADVANCE_SPEED_MPS = 1.0
KICKOFF_KICKER_STANDOFF_M = 0.2  # kicker waits this far on our side of the ball
PENALTY_RETREAT_M = 1.0  # non-kicker robots stay this far behind the ball
OWN_HALF_MARGIN_M = 0.2  # clamp margin from the halfway line

STATE_CHASE = "CHASE"
STATE_POSSESS = "POSSESS"
STATE_PASS_IN_FLIGHT = "PASS_IN_FLIGHT"


@dataclass
class TeamConfig:
    is_team_yellow: bool
    defend_positive_x: bool  # True: this team's own goal is on the +X side


def _ball_speed(ball) -> float:
    return math.hypot(ball.vx, ball.vy)


def _holds_ball(robot, ball) -> bool:
    close = math.hypot(robot.x - ball.x, robot.y - ball.y) < POSSESSION_DIST_M
    return close and _ball_speed(ball) < SLOW_BALL_SPEED_MPS


class TeamStrategy:
    def __init__(self, config: TeamConfig):
        self._config = config
        self._goalkeeper_id: Optional[int] = None
        self._formation_slots: Dict[int, int] = {}
        self._state: str = STATE_CHASE
        self._receiver_id: Optional[int] = None
        self._pass_kick_time: float = 0.0
        # (receiver_id, kick_time) recorded by the holder mid-loop; applied at
        # the end of the tick so every robot sees the pre-kick state this tick.
        self._pending_pass: Optional[Tuple[int, float]] = None
        self.rule_exempt_ids: frozenset = frozenset()
        self._forbidden_toucher_id: Optional[int] = None

    # --- state machine -------------------------------------------------

    def _update_state(self, own_robots, ball, holder_id):
        if self._state == STATE_PASS_IN_FLIGHT:
            elapsed = ball.t_capture - self._pass_kick_time
            receiver = own_robots.get(self._receiver_id)
            ball_settled = (
                elapsed >= PASS_MIN_FLIGHT_S and _ball_speed(ball) < SLOW_BALL_SPEED_MPS
            )
            if (
                receiver is None
                or _holds_ball(receiver, ball)
                or elapsed >= PASS_TIMEOUT_S
                or ball_settled
            ):
                self._enter_chase()
            # Per spec: a pass always exits to CHASE; POSSESS follows via the
            # normal derivation on the NEXT tick, so return either way here.
            return
        # CHASE <-> POSSESS is derived directly from possession each tick.
        self._state = STATE_POSSESS if holder_id is not None else STATE_CHASE

    def _enter_chase(self):
        self._state = STATE_CHASE
        self._receiver_id = None
        self._pending_pass = None

    # --- main entry -----------------------------------------------------

    @property
    def goalkeeper_id(self) -> Optional[int]:
        return self._goalkeeper_id

    def compute_commands(self, world: WorldModel, game_state: GameState):
        # Snapshot: the Vision receiver runs on its own thread and mutates
        # world.*_robots concurrently with this method's iteration.
        if self._config.is_team_yellow:
            own_robots = dict(world.yellow_robots)
            opponents = list(world.blue_robots.values())
        else:
            own_robots = dict(world.blue_robots)
            opponents = list(world.yellow_robots.values())
        if not own_robots or world.ball is None or world.geometry is None:
            return []

        self.rule_exempt_ids = frozenset()
        phase = game_state.phase

        if phase is Phase.HALT:
            self._enter_chase()
            return [
                RobotCommand(rid, 0.0, 0.0, 0.0, robot.orientation)
                for rid, robot in own_robots.items()
            ]

        if self._goalkeeper_id is None or self._goalkeeper_id not in own_robots:
            self._goalkeeper_id = min(own_robots.keys())

        non_keeper_ids = sorted(rid for rid in own_robots if rid != self._goalkeeper_id)
        for slot_index, rid in enumerate(non_keeper_ids):
            self._formation_slots.setdefault(rid, slot_index)

        ball = world.ball
        geometry = world.geometry

        if phase is Phase.RUNNING:
            return self._run_play(world, own_robots, opponents, non_keeper_ids, ball, geometry)

        if phase in (Phase.KICKOFF_OURS, Phase.PENALTY_OURS):
            return self._set_piece_ours(phase, game_state, world, own_robots,
                                        non_keeper_ids, ball, geometry)
        if phase is Phase.KICKOFF_THEIRS:
            return self._kickoff_theirs(world, own_robots, ball, geometry)
        if phase is Phase.PENALTY_THEIRS:
            return self._penalty_theirs(world, own_robots, ball, geometry)
        if phase is Phase.FREE_KICK_OURS:
            return self._free_kick_ours(game_state, world, own_robots, opponents,
                                        non_keeper_ids, ball, geometry)

        # Any other non-running, non-halt phase (FREE_KICK_THEIRS, BALL_PLACEMENT_*):
        # strategy repositions; rules.py enforces distances/speed on top.
        # Set-piece placement for these is refined in a later task;
        # STOP-style formation is the safe default.
        self._enter_chase()
        return self._stop_commands(world, own_robots, ball, geometry)

    def _run_play(self, world, own_robots, opponents, non_keeper_ids, ball, geometry):
        if self._forbidden_toucher_id is not None:
            others_near_ball = any(
                math.hypot(r.x - ball.x, r.y - ball.y) < POSSESSION_DIST_M
                for rid, r in own_robots.items() if rid != self._forbidden_toucher_id
            ) or any(
                math.hypot(o.x - ball.x, o.y - ball.y) < POSSESSION_DIST_M
                for o in opponents
            )
            if others_near_ball:
                self._forbidden_toucher_id = None

        eligible_ids = [rid for rid in non_keeper_ids if rid != self._forbidden_toucher_id]

        holder_id = None
        holders = [rid for rid in eligible_ids if _holds_ball(own_robots[rid], ball)]
        if holders:
            holder_id = min(
                holders,
                key=lambda rid: math.hypot(
                    own_robots[rid].x - ball.x, own_robots[rid].y - ball.y
                ),
            )

        self._update_state(own_robots, ball, holder_id)

        chaser_id = None
        if self._state == STATE_CHASE and eligible_ids:
            chaser_id = min(
                eligible_ids,
                key=lambda rid: math.hypot(
                    own_robots[rid].x - ball.x, own_robots[rid].y - ball.y
                ),
            )

        goal_x = -own_goal_x(world, self._config.defend_positive_x)
        goal_xy = (goal_x, 0.0)
        forward_sign = -1.0 if self._config.defend_positive_x else 1.0

        commands = []
        for rid, robot in own_robots.items():
            if rid == self._goalkeeper_id:
                commands.append(
                    self._keeper_command(rid, robot, geometry, ball, own_robots)
                )
            elif self._state == STATE_POSSESS and rid == holder_id:
                commands.append(
                    self._holder_command(
                        rid, robot, ball, own_robots, opponents, goal_xy
                    )
                )
            elif self._state == STATE_PASS_IN_FLIGHT and rid == self._receiver_id:
                commands.append(self._receiver_command(rid, robot, ball, own_robots))
            elif self._state == STATE_CHASE and rid == chaser_id:
                commands.append(self._chaser_command(rid, robot, ball, own_robots))
            else:
                commands.append(
                    self._support_command(
                        rid, robot, world, ball, own_robots, opponents, forward_sign
                    )
                )

        if self._pending_pass is not None:
            self._receiver_id, self._pass_kick_time = self._pending_pass
            self._state = STATE_PASS_IN_FLIGHT
            self._pending_pass = None
        return commands

    def _stop_commands(self, world, own_robots, ball, geometry):
        commands = []
        for rid, robot in own_robots.items():
            if rid == self._goalkeeper_id:
                commands.append(self._keeper_command(rid, robot, geometry, ball, own_robots))
                continue
            slot_index = self._formation_slots[rid]
            target_x, target_y = formation_target(
                world, self._config.defend_positive_x, slot_index, ball.x
            )
            vx, vy = seek(robot.x, robot.y, target_x, target_y)
            vx, vy = apply_separation(rid, vx, vy, robot, own_robots)
            commands.append(RobotCommand(rid, vx, vy, 0.0, robot.orientation))
        return commands

    # --- set-piece placement ---------------------------------------------

    def _choose_kicker(self, non_keeper_ids, own_robots, ball):
        if not non_keeper_ids:
            return None
        return min(non_keeper_ids, key=lambda rid: math.hypot(
            own_robots[rid].x - ball.x, own_robots[rid].y - ball.y))

    def _kicker_command(self, rid, robot, ball, own_robots, may_kick, goal_xy):
        if may_kick:
            vx, vy = seek(robot.x, robot.y, ball.x, ball.y)
        else:
            forward_sign = -1.0 if self._config.defend_positive_x else 1.0
            standoff_x = ball.x - forward_sign * KICKOFF_KICKER_STANDOFF_M
            vx, vy = seek(robot.x, robot.y, standoff_x, ball.y)
        vx, vy = apply_separation(rid, vx, vy, robot, own_robots)
        target_orientation = math.atan2(goal_xy[1] - robot.y, goal_xy[0] - robot.x)
        vel_angular, angle_error = face(robot.orientation, target_orientation)
        kick = 0.0
        if may_kick:
            ball_dist = math.hypot(robot.x - ball.x, robot.y - ball.y)
            if abs(angle_error) < KICK_ANGLE_TOLERANCE and ball_dist < KICK_RANGE_M:
                kick = SHOOT_KICK_SPEED_MPS
        return RobotCommand(rid, vx, vy, vel_angular, robot.orientation,
                            kick_speed=kick, dribble=may_kick)

    def _set_piece_ours(self, phase, game_state, world, own_robots,
                        non_keeper_ids, ball, geometry):
        self._enter_chase()
        kicker_id = self._choose_kicker(non_keeper_ids, own_robots, ball)
        if kicker_id is not None:
            self.rule_exempt_ids = frozenset({kicker_id})
        goal_x = -own_goal_x(world, self._config.defend_positive_x)
        goal_xy = (goal_x, 0.0)
        forward_sign = -1.0 if self._config.defend_positive_x else 1.0
        commands = []
        for rid, robot in own_robots.items():
            if rid == self._goalkeeper_id:
                commands.append(self._keeper_command(rid, robot, geometry, ball, own_robots))
            elif rid == kicker_id:
                cmd = self._kicker_command(
                    rid, robot, ball, own_robots, game_state.may_kick, goal_xy)
                if phase is Phase.KICKOFF_OURS and cmd.kick_speed > 0.0:
                    self._forbidden_toucher_id = rid
                commands.append(cmd)
            elif phase is Phase.PENALTY_OURS:
                target_x = ball.x - forward_sign * PENALTY_RETREAT_M
                vx, vy = seek(robot.x, robot.y, target_x, robot.y)
                vx, vy = apply_separation(rid, vx, vy, robot, own_robots)
                commands.append(RobotCommand(rid, vx, vy, 0.0, robot.orientation))
            else:  # KICKOFF_OURS support robots: formation clamped to own half
                commands.append(self._own_half_formation_command(
                    rid, robot, world, ball, own_robots, forward_sign))
        return commands

    def _free_kick_ours(self, game_state, world, own_robots, opponents,
                        non_keeper_ids, ball, geometry):
        self._enter_chase()
        kicker_id = self._choose_kicker(non_keeper_ids, own_robots, ball)
        if kicker_id is not None:
            self.rule_exempt_ids = frozenset({kicker_id})
        goal_x = -own_goal_x(world, self._config.defend_positive_x)
        goal_xy = (goal_x, 0.0)
        forward_sign = -1.0 if self._config.defend_positive_x else 1.0
        commands = []
        for rid, robot in own_robots.items():
            if rid == self._goalkeeper_id:
                commands.append(self._keeper_command(rid, robot, geometry, ball, own_robots))
            elif rid == kicker_id:
                cmd = self._kicker_command(rid, robot, ball, own_robots,
                                           game_state.may_kick, goal_xy)
                if cmd.kick_speed > 0.0:
                    self._forbidden_toucher_id = rid
                commands.append(cmd)
            else:
                commands.append(self._support_command(
                    rid, robot, world, ball, own_robots, opponents, forward_sign))
        return commands

    def _own_half_formation_command(self, rid, robot, world, ball, own_robots, forward_sign):
        slot_index = self._formation_slots[rid]
        target_x, target_y = formation_target(
            world, self._config.defend_positive_x, slot_index, ball.x)
        # own half: x has opposite sign to forward_sign; clamp with margin
        if forward_sign > 0:
            target_x = min(target_x, -OWN_HALF_MARGIN_M)
        else:
            target_x = max(target_x, OWN_HALF_MARGIN_M)
        vx, vy = seek(robot.x, robot.y, target_x, target_y)
        vx, vy = apply_separation(rid, vx, vy, robot, own_robots)
        return RobotCommand(rid, vx, vy, 0.0, robot.orientation)

    def _kickoff_theirs(self, world, own_robots, ball, geometry):
        self._enter_chase()
        forward_sign = -1.0 if self._config.defend_positive_x else 1.0
        commands = []
        for rid, robot in own_robots.items():
            if rid == self._goalkeeper_id:
                commands.append(self._keeper_command(rid, robot, geometry, ball, own_robots))
            else:
                commands.append(self._own_half_formation_command(
                    rid, robot, world, ball, own_robots, forward_sign))
        return commands

    def _penalty_theirs(self, world, own_robots, ball, geometry):
        self._enter_chase()
        forward_sign = -1.0 if self._config.defend_positive_x else 1.0
        commands = []
        for rid, robot in own_robots.items():
            if rid == self._goalkeeper_id:
                commands.append(self._keeper_command(rid, robot, geometry, ball, own_robots))
            else:
                # stay behind the ball, away from our goal under attack
                target_x = ball.x + forward_sign * PENALTY_RETREAT_M
                vx, vy = seek(robot.x, robot.y, target_x, robot.y)
                vx, vy = apply_separation(rid, vx, vy, robot, own_robots)
                commands.append(RobotCommand(rid, vx, vy, 0.0, robot.orientation))
        return commands

    # --- per-role command builders ---------------------------------------

    def _keeper_command(self, rid, robot, geometry, ball, own_robots):
        target_x, target_y = goalkeeper_target(
            geometry, ball.y, self._config.defend_positive_x
        )
        vx, vy = seek(robot.x, robot.y, target_x, target_y)
        vx, vy = apply_separation(rid, vx, vy, robot, own_robots)
        return RobotCommand(rid, vx, vy, 0.0, robot.orientation)

    def _holder_command(self, rid, robot, ball, own_robots, opponents, goal_xy):
        mates = {
            other_id: other
            for other_id, other in own_robots.items()
            if other_id not in (rid, self._goalkeeper_id)
        }
        action = choose_action((ball.x, ball.y), mates, opponents, goal_xy)

        if action.kind == "dribble":
            # Advance toward the goal at a controlled speed while keeping
            # the dribbler on; turn to face the goal as we go.
            distance = math.hypot(goal_xy[0] - robot.x, goal_xy[1] - robot.y)
            if distance > 1e-6:
                vx = (goal_xy[0] - robot.x) / distance * DRIBBLE_ADVANCE_SPEED_MPS
                vy = (goal_xy[1] - robot.y) / distance * DRIBBLE_ADVANCE_SPEED_MPS
            else:
                vx = vy = 0.0
            target_orientation = math.atan2(goal_xy[1] - robot.y, goal_xy[0] - robot.x)
            vel_angular, _ = face(robot.orientation, target_orientation)
            vx, vy = apply_separation(rid, vx, vy, robot, own_robots)
            return RobotCommand(rid, vx, vy, vel_angular, robot.orientation, dribble=True)

        # shoot / pass: stay on the ball, rotate toward the aim point, and
        # kick once aligned and in range.
        vx, vy = seek(robot.x, robot.y, ball.x, ball.y)
        vx, vy = apply_separation(rid, vx, vy, robot, own_robots)
        target_orientation = math.atan2(
            action.target_y - robot.y, action.target_x - robot.x
        )
        vel_angular, angle_error = face(robot.orientation, target_orientation)
        ball_dist = math.hypot(robot.x - ball.x, robot.y - ball.y)
        aligned = abs(angle_error) < KICK_ANGLE_TOLERANCE
        kick = action.kick_speed if (aligned and ball_dist < KICK_RANGE_M) else 0.0
        if kick > 0.0 and action.kind == "pass":
            # Record only; compute_commands applies the PASS_IN_FLIGHT
            # transition at the end of the tick so robots iterated after the
            # holder still see the pre-kick state (deterministic roles).
            self._pending_pass = (action.receiver_id, ball.t_capture)
        return RobotCommand(
            rid, vx, vy, vel_angular, robot.orientation, kick_speed=kick, dribble=True
        )

    def _receiver_command(self, rid, robot, ball, own_robots):
        target_x, target_y = receiver_target(robot.x, robot.y, ball)
        vx, vy = seek(robot.x, robot.y, target_x, target_y)
        vx, vy = apply_separation(rid, vx, vy, robot, own_robots)
        target_orientation = math.atan2(ball.y - robot.y, ball.x - robot.x)
        vel_angular, _ = face(robot.orientation, target_orientation)
        return RobotCommand(rid, vx, vy, vel_angular, robot.orientation, dribble=True)

    def _chaser_command(self, rid, robot, ball, own_robots):
        vx, vy = seek(robot.x, robot.y, ball.x, ball.y)
        vx, vy = apply_separation(rid, vx, vy, robot, own_robots)
        target_orientation = math.atan2(ball.y - robot.y, ball.x - robot.x)
        vel_angular, _ = face(robot.orientation, target_orientation)
        return RobotCommand(rid, vx, vy, vel_angular, robot.orientation, dribble=True)

    def _support_command(self, rid, robot, world, ball, own_robots, opponents, forward_sign):
        slot_index = self._formation_slots[rid]
        base_xy = formation_target(
            world, self._config.defend_positive_x, slot_index, ball.x
        )
        target_x, target_y = support_target(base_xy, forward_sign, (ball.x, ball.y), opponents)
        vx, vy = seek(robot.x, robot.y, target_x, target_y)
        vx, vy = apply_separation(rid, vx, vy, robot, own_robots)
        return RobotCommand(rid, vx, vy, 0.0, robot.orientation)
