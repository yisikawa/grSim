"""Team-view interpretation of referee commands.

Pure mapping from (Referee command, our team color) to a Phase, plus
GameStateTracker for the stateful parts: NORMAL_START inherits the
preceding PREPARE state, and set pieces end once the ball moves.
"""
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Tuple

from state import ssl_gc_referee_message_pb2 as referee_pb2

_MM_TO_M = 1.0 / 1000.0

Cmd = referee_pb2.Referee


class Phase(Enum):
    HALT = auto()
    STOP = auto()
    RUNNING = auto()
    KICKOFF_OURS = auto()
    KICKOFF_THEIRS = auto()
    FREE_KICK_OURS = auto()
    FREE_KICK_THEIRS = auto()
    PENALTY_OURS = auto()
    PENALTY_THEIRS = auto()
    BALL_PLACEMENT_OURS = auto()
    BALL_PLACEMENT_THEIRS = auto()


@dataclass(frozen=True)
class GameState:
    phase: Phase
    may_kick: bool = False  # set pieces: our kicker may strike the ball
    designated_position: Optional[Tuple[float, float]] = None  # meters


_PREPARE_PHASES = (
    Phase.KICKOFF_OURS,
    Phase.KICKOFF_THEIRS,
    Phase.PENALTY_OURS,
    Phase.PENALTY_THEIRS,
)

# command -> (phase when we are yellow, phase when we are blue)
_TEAM_COMMANDS = {
    Cmd.PREPARE_KICKOFF_YELLOW: (Phase.KICKOFF_OURS, Phase.KICKOFF_THEIRS),
    Cmd.PREPARE_KICKOFF_BLUE: (Phase.KICKOFF_THEIRS, Phase.KICKOFF_OURS),
    Cmd.PREPARE_PENALTY_YELLOW: (Phase.PENALTY_OURS, Phase.PENALTY_THEIRS),
    Cmd.PREPARE_PENALTY_BLUE: (Phase.PENALTY_THEIRS, Phase.PENALTY_OURS),
    Cmd.DIRECT_FREE_YELLOW: (Phase.FREE_KICK_OURS, Phase.FREE_KICK_THEIRS),
    Cmd.DIRECT_FREE_BLUE: (Phase.FREE_KICK_THEIRS, Phase.FREE_KICK_OURS),
    Cmd.INDIRECT_FREE_YELLOW: (Phase.FREE_KICK_OURS, Phase.FREE_KICK_THEIRS),
    Cmd.INDIRECT_FREE_BLUE: (Phase.FREE_KICK_THEIRS, Phase.FREE_KICK_OURS),
    Cmd.BALL_PLACEMENT_YELLOW: (Phase.BALL_PLACEMENT_OURS, Phase.BALL_PLACEMENT_THEIRS),
    Cmd.BALL_PLACEMENT_BLUE: (Phase.BALL_PLACEMENT_THEIRS, Phase.BALL_PLACEMENT_OURS),
}


def phase_for_command(command: int, is_team_yellow: bool, prev_phase: Phase = Phase.STOP) -> Phase:
    if command == Cmd.HALT:
        return Phase.HALT
    if command == Cmd.FORCE_START:
        return Phase.RUNNING
    if command == Cmd.NORMAL_START:
        if prev_phase in _PREPARE_PHASES:
            return prev_phase
        return Phase.RUNNING
    if command in _TEAM_COMMANDS:
        ours_if_yellow, ours_if_blue = _TEAM_COMMANDS[command]
        return ours_if_yellow if is_team_yellow else ours_if_blue
    # STOP, TIMEOUT_*, GOAL_*, and anything unknown: safe fallback
    return Phase.STOP


KICK_DETECT_DIST_M = 0.05  # set piece ends once the ball moves this far

_SET_PIECE_PHASES = (
    Phase.KICKOFF_OURS,
    Phase.KICKOFF_THEIRS,
    Phase.FREE_KICK_OURS,
    Phase.FREE_KICK_THEIRS,
    Phase.PENALTY_OURS,
    Phase.PENALTY_THEIRS,
)

_FREE_KICK_PHASES = (Phase.FREE_KICK_OURS, Phase.FREE_KICK_THEIRS)


class GameStateTracker:
    """Holds the small amount of state needed across referee commands:
    which command counter was last seen, the phase NORMAL_START continues,
    and the ball reference position used to detect that a set piece has
    actually been taken."""

    def __init__(self, is_team_yellow: bool):
        self._is_yellow = is_team_yellow
        self._counter: Optional[int] = None
        self._phase = Phase.RUNNING  # no referee yet: free play
        self._may_kick = True
        self._designated: Optional[Tuple[float, float]] = None
        self._ball_ref: Optional[Tuple[float, float]] = None

    def update(self, msg, ball) -> GameState:
        if msg is None:
            return GameState(Phase.RUNNING, may_kick=True)
        if msg.command_counter != self._counter:
            self._counter = msg.command_counter
            self._apply_command(msg)
        self._maybe_finish_set_piece(ball)
        return GameState(self._phase, self._may_kick, self._designated)

    def _apply_command(self, msg) -> None:
        new_phase = phase_for_command(msg.command, self._is_yellow, self._phase)
        if msg.command == Cmd.NORMAL_START:
            self._may_kick = True  # arms the pending kickoff/penalty
        else:
            self._may_kick = new_phase in _FREE_KICK_PHASES or new_phase is Phase.RUNNING
        self._phase = new_phase
        self._designated = None
        if msg.HasField("designated_position"):
            self._designated = (
                msg.designated_position.x * _MM_TO_M,
                msg.designated_position.y * _MM_TO_M,
            )
        self._ball_ref = None

    def _maybe_finish_set_piece(self, ball) -> None:
        if self._phase not in _SET_PIECE_PHASES or not self._may_kick or ball is None:
            return
        if self._ball_ref is None:
            self._ball_ref = (ball.x, ball.y)
            return
        dx = ball.x - self._ball_ref[0]
        dy = ball.y - self._ball_ref[1]
        if (dx * dx + dy * dy) ** 0.5 > KICK_DETECT_DIST_M:
            self._phase = Phase.RUNNING
            self._may_kick = True
