"""Team-view interpretation of referee commands.

Pure mapping from (Referee command, our team color) to a Phase, plus (in a
later task) a small tracker for the stateful parts: NORMAL_START inherits
the preceding PREPARE state, and set pieces end once the ball moves.
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
