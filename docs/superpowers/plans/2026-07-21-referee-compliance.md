# 審判状態への完全準拠 実装計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `clients/python/` の対戦クライアントを ssl-game-controller の全主要審判状態(キックオフ/フリーキック/PK/ボールプレースメント/STOP速度制限等)に準拠させる。

**Architecture:** GameState層(Refereeメッセージ→チーム視点状態への純粋変換+小さなトラッカー)+制約フィルタ(戦略出力への安全網)の二段構え。戦略層は状態別の目標位置を選び、`rules.py` が速度上限・ボール退避・ディフェンスエリア進入禁止を最終強制する。

**Tech Stack:** Python 3 / protobuf (生成済みバインディング `ssl_client/pb/`) / pytest

**Spec:** `docs/superpowers/specs/2026-07-21-referee-compliance-design.md`

## Global Constraints

- 新規外部依存を追加しない(protobuf と pytest のみ。`clients/python/requirements.txt` は変更しない)
- テスト実行はすべて `clients/python` ディレクトリから: `python -m pytest tests/ -v`(pytest.ini と conftest.py が sys.path を設定済み)
- 座標系: フィールド座標はメートル。Refereeの `designated_position` は **mm** なので m へ変換する
- Referee protobuf は `from state import ssl_gc_referee_message_pb2 as referee_pb2` でインポート(`ssl_client/__init__.py` が `pb/` を sys.path に載せるブートストラップを行うため、テスト・実装とも先に `import ssl_client` が必要。既存コード [referee.py](../../clients/python/ssl_client/referee.py) と同じ流儀)
- コミットメッセージは日本語(リポジトリの既存流儀)。各タスク完了ごとにコミット
- 既存のコードスタイル(dataclass・純粋関数・モジュール定数)を踏襲する

---

### Task 1: FieldGeometry にペナルティエリア寸法を追加

**Files:**
- Modify: `clients/python/ssl_client/world.py`
- Test: `clients/python/tests/test_world.py`

**Interfaces:**
- Consumes: 既存 `FieldGeometry` dataclass、`update_from_geometry_data(world, geometry)`
- Produces: `FieldGeometry.penalty_area_depth: float`(デフォルト1.0)、`FieldGeometry.penalty_area_width: float`(デフォルト2.0)。geometryパケットに `penalty_area_depth`/`penalty_area_width` があれば mm→m 変換して採用、なければデフォルト値。

- [ ] **Step 1: 失敗するテストを書く**

`clients/python/tests/test_world.py` の末尾に追加:

```python
class _FakeFieldWithPenalty:
    def __init__(self):
        self.field_length = 12000
        self.field_width = 9000
        self.goal_width = 1800
        self.goal_depth = 180
        self.boundary_width = 300
        self._has_penalty = True
        self.penalty_area_depth = 1800
        self.penalty_area_width = 3600

    def HasField(self, name):
        return self._has_penalty


class _FakeGeometryPacket:
    def __init__(self, field):
        self.field = field


def test_geometry_uses_penalty_area_from_packet():
    from ssl_client.world import WorldModel, update_from_geometry_data

    world = WorldModel()
    update_from_geometry_data(world, _FakeGeometryPacket(_FakeFieldWithPenalty()))
    assert world.geometry.penalty_area_depth == 1.8
    assert world.geometry.penalty_area_width == 3.6


def test_geometry_falls_back_to_division_b_defaults():
    from ssl_client.world import WorldModel, update_from_geometry_data

    field = _FakeFieldWithPenalty()
    field._has_penalty = False
    world = WorldModel()
    update_from_geometry_data(world, _FakeGeometryPacket(field))
    assert world.geometry.penalty_area_depth == 1.0
    assert world.geometry.penalty_area_width == 2.0
```

注意: 既存テストが `Fake` 系ヘルパーを既に持つ場合は命名衝突を避けて再利用すること(先に `tests/test_world.py` を読むこと)。

- [ ] **Step 2: テストが失敗することを確認**

Run: `cd clients/python && python -m pytest tests/test_world.py -v`
Expected: 新規2テストが FAIL(`penalty_area_depth` が存在しない/TypeError)

- [ ] **Step 3: 最小実装**

`ssl_client/world.py` の `FieldGeometry` にフィールドを追加し、`update_from_geometry_data` を拡張:

```python
@dataclass
class FieldGeometry:
    field_length: float  # meters
    field_width: float  # meters
    goal_width: float  # meters
    goal_depth: float  # meters
    boundary_width: float  # meters
    penalty_area_depth: float = 1.0  # meters; Division B default when absent
    penalty_area_width: float = 2.0  # meters; Division B default when absent
```

```python
def update_from_geometry_data(world: WorldModel, geometry) -> None:
    f = geometry.field
    kwargs = {}
    if f.HasField("penalty_area_depth"):
        kwargs["penalty_area_depth"] = f.penalty_area_depth * _MM_TO_M
    if f.HasField("penalty_area_width"):
        kwargs["penalty_area_width"] = f.penalty_area_width * _MM_TO_M
    world.geometry = FieldGeometry(
        field_length=f.field_length * _MM_TO_M,
        field_width=f.field_width * _MM_TO_M,
        goal_width=f.goal_width * _MM_TO_M,
        goal_depth=f.goal_depth * _MM_TO_M,
        boundary_width=f.boundary_width * _MM_TO_M,
        **kwargs,
    )
```

- [ ] **Step 4: テスト通過を確認**

Run: `cd clients/python && python -m pytest tests/test_world.py -v`
Expected: 全テスト PASS

- [ ] **Step 5: コミット**

```bash
git add clients/python/ssl_client/world.py clients/python/tests/test_world.py
git commit -m "ペナルティエリア寸法をFieldGeometryに追加"
```

---

### Task 2: game_state.py — Refereeコマンド→Phase の純粋変換

**Files:**
- Create: `clients/python/ssl_client/game_state.py`
- Test: `clients/python/tests/test_game_state.py`

**Interfaces:**
- Consumes: `state.ssl_gc_referee_message_pb2.Referee`(コマンドenum: HALT=0, STOP=1, NORMAL_START=2, FORCE_START=3, PREPARE_KICKOFF_YELLOW=4, PREPARE_KICKOFF_BLUE=5, PREPARE_PENALTY_YELLOW=6, PREPARE_PENALTY_BLUE=7, DIRECT_FREE_YELLOW=8, DIRECT_FREE_BLUE=9, INDIRECT_FREE_YELLOW=10, INDIRECT_FREE_BLUE=11, TIMEOUT_YELLOW=12, TIMEOUT_BLUE=13, GOAL_YELLOW=14, GOAL_BLUE=15, BALL_PLACEMENT_YELLOW=16, BALL_PLACEMENT_BLUE=17)
- Produces: `Phase`(Enum)、`GameState`(frozen dataclass: `phase: Phase`, `may_kick: bool = False`, `designated_position: Optional[Tuple[float, float]] = None`)、`phase_for_command(command: int, is_team_yellow: bool, prev_phase: Phase = Phase.STOP) -> Phase`

- [ ] **Step 1: 失敗するテストを書く**

`clients/python/tests/test_game_state.py` を新規作成:

```python
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
```

- [ ] **Step 2: テストが失敗することを確認**

Run: `cd clients/python && python -m pytest tests/test_game_state.py -v`
Expected: FAIL(ModuleNotFoundError: ssl_client.game_state)

- [ ] **Step 3: 最小実装**

`clients/python/ssl_client/game_state.py` を新規作成:

```python
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
```

- [ ] **Step 4: テスト通過を確認**

Run: `cd clients/python && python -m pytest tests/test_game_state.py -v`
Expected: 全テスト PASS

- [ ] **Step 5: コミット**

```bash
git add clients/python/ssl_client/game_state.py clients/python/tests/test_game_state.py
git commit -m "Refereeコマンドをチーム視点Phaseへ変換するgame_stateを追加"
```

---

### Task 3: GameStateTracker — NORMAL_START引き継ぎとキック後遷移

**Files:**
- Modify: `clients/python/ssl_client/game_state.py`
- Test: `clients/python/tests/test_game_state.py`

**Interfaces:**
- Consumes: Task 2 の `Phase` / `GameState` / `phase_for_command`、`Referee` メッセージ(`command`, `command_counter`, `designated_position`{x,y in mm})、ボール観測(`.x`/`.y` を持つオブジェクト or None)
- Produces: `GameStateTracker(is_team_yellow: bool)` と `GameStateTracker.update(msg, ball) -> GameState`。`KICK_DETECT_DIST_M = 0.05`。msg=None → `GameState(Phase.RUNNING, may_kick=True)`。同じ `command_counter` の再受信では状態を再適用しない。set piece 中に `may_kick` かつボールが基準位置から 0.05m 超動いたら `Phase.RUNNING` へ遷移。

- [ ] **Step 1: 失敗するテストを書く**

`tests/test_game_state.py` の末尾に追加:

```python
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
```

注意: `_msg` ヘルパーの必須フィールドは、生成済みバインディングが proto2 必須フィールドを検証しない場合は簡略化してよい(まず `m = referee_pb2.Referee(); m.command = ...; m.command_counter = ...` だけで動くか試し、SerializeToString しない限り不要なら省く)。

- [ ] **Step 2: テストが失敗することを確認**

Run: `cd clients/python && python -m pytest tests/test_game_state.py -v`
Expected: 新規テストが FAIL(ImportError: GameStateTracker)

- [ ] **Step 3: 最小実装**

`ssl_client/game_state.py` の末尾に追加:

```python
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
```

- [ ] **Step 4: テスト通過を確認**

Run: `cd clients/python && python -m pytest tests/test_game_state.py -v`
Expected: 全テスト PASS

- [ ] **Step 5: コミット**

```bash
git add clients/python/ssl_client/game_state.py clients/python/tests/test_game_state.py
git commit -m "GameStateTrackerを追加(NORMAL_START引き継ぎとキック後遷移)"
```

---

### Task 4: rules.py — 速度上限・ボール退避・プレースメント線分退避

**Files:**
- Create: `clients/python/ssl_client/rules.py`
- Test: `clients/python/tests/test_rules.py`

**Interfaces:**
- Consumes: `RobotCommand`([commands.py](../../clients/python/ssl_client/commands.py))、`GameState`/`Phase`(Task 2-3)、`RobotObservation`/`BallObservation`/`FieldGeometry`(world.py)
- Produces:
  ```python
  apply_rule_constraints(
      commands: list[RobotCommand],
      game_state: GameState,
      *,
      own_robots: dict[int, RobotObservation],
      ball: Optional[BallObservation],
      geometry: FieldGeometry,
      defend_positive_x: bool,
      keeper_id: Optional[int],
      exempt_ids: frozenset = frozenset(),
  ) -> list[RobotCommand]
  ```
  定数: `STOP_SPEED_CAP_MPS = 1.5`, `BALL_KEEP_OUT_M = 0.5`, `RETREAT_SPEED_MPS = 1.0`。Task 5 でディフェンスエリア・場外制約を同関数に追加する。

- [ ] **Step 1: 失敗するテストを書く**

`clients/python/tests/test_rules.py` を新規作成:

```python
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
```

- [ ] **Step 2: テストが失敗することを確認**

Run: `cd clients/python && python -m pytest tests/test_rules.py -v`
Expected: FAIL(ModuleNotFoundError: ssl_client.rules)

- [ ] **Step 3: 最小実装**

`clients/python/ssl_client/rules.py` を新規作成:

```python
"""Rule-constraint filter applied to strategy output as a safety net.

The strategy is expected to pick rule-compliant targets already; this
filter force-corrects whatever still violates the rules, so a strategy
bug degrades into conservative motion instead of a foul.
"""
import math
from typing import Optional

from .commands import RobotCommand
from .game_state import GameState, Phase

STOP_SPEED_CAP_MPS = 1.5
BALL_KEEP_OUT_M = 0.5
RETREAT_SPEED_MPS = 1.0

_SPEED_CAP_PHASES = (
    Phase.STOP,
    Phase.BALL_PLACEMENT_OURS,
    Phase.BALL_PLACEMENT_THEIRS,
)

# Phases where non-exempt robots must stay away from the ball (or, for
# ball placement, from the ball->designated_position corridor).
_BALL_KEEP_OUT_PHASES = (
    Phase.STOP,
    Phase.KICKOFF_OURS,
    Phase.KICKOFF_THEIRS,
    Phase.FREE_KICK_OURS,
    Phase.FREE_KICK_THEIRS,
    Phase.PENALTY_OURS,
    Phase.PENALTY_THEIRS,
    Phase.BALL_PLACEMENT_OURS,
    Phase.BALL_PLACEMENT_THEIRS,
)

_PLACEMENT_PHASES = (Phase.BALL_PLACEMENT_OURS, Phase.BALL_PLACEMENT_THEIRS)


def _nearest_point_on_segment(px, py, ax, ay, bx, by):
    abx, aby = bx - ax, by - ay
    ab_len_sq = abx * abx + aby * aby
    if ab_len_sq == 0.0:
        return ax, ay
    t = ((px - ax) * abx + (py - ay) * aby) / ab_len_sq
    t = max(0.0, min(1.0, t))
    return ax + t * abx, ay + t * aby


def _away_velocity(from_x, from_y, robot_x, robot_y):
    dx, dy = robot_x - from_x, robot_y - from_y
    dist = math.hypot(dx, dy)
    if dist < 1e-9:
        return RETREAT_SPEED_MPS, 0.0  # arbitrary but deterministic direction
    return dx / dist * RETREAT_SPEED_MPS, dy / dist * RETREAT_SPEED_MPS


def apply_rule_constraints(
    commands,
    game_state: GameState,
    *,
    own_robots,
    ball,
    geometry,
    defend_positive_x: bool,
    keeper_id: Optional[int],
    exempt_ids: frozenset = frozenset(),
):
    phase = game_state.phase
    out = []
    for cmd in commands:
        robot = own_robots.get(cmd.robot_id)
        if robot is None:
            out.append(cmd)
            continue
        vx, vy = cmd.vel_x, cmd.vel_y
        vel_angular = cmd.vel_angular
        kick, chip, dribble = cmd.kick_speed, cmd.chip_speed, cmd.dribble
        exempt = cmd.robot_id in exempt_ids

        if phase is Phase.HALT:
            vx = vy = vel_angular = 0.0
            kick = chip = 0.0
            dribble = False
        else:
            if ball is not None and phase in _BALL_KEEP_OUT_PHASES and not exempt:
                if phase in _PLACEMENT_PHASES and game_state.designated_position is not None:
                    nx, ny = _nearest_point_on_segment(
                        robot.x, robot.y, ball.x, ball.y,
                        game_state.designated_position[0], game_state.designated_position[1],
                    )
                else:
                    nx, ny = ball.x, ball.y
                if math.hypot(robot.x - nx, robot.y - ny) < BALL_KEEP_OUT_M:
                    vx, vy = _away_velocity(nx, ny, robot.x, robot.y)
            # Kicks are only legal in open play, or for the designated
            # kicker once the set piece is armed.
            if phase is not Phase.RUNNING and not (game_state.may_kick and exempt):
                kick = chip = 0.0
            if phase in _SPEED_CAP_PHASES:
                speed = math.hypot(vx, vy)
                if speed > STOP_SPEED_CAP_MPS:
                    scale = STOP_SPEED_CAP_MPS / speed
                    vx *= scale
                    vy *= scale

        out.append(RobotCommand(
            cmd.robot_id, vx, vy, vel_angular, cmd.robot_orientation,
            kick_speed=kick, chip_speed=chip, dribble=dribble,
        ))
    return out
```

(`geometry`/`defend_positive_x`/`keeper_id` はこのタスクでは未使用だが、Task 5 で使うためシグネチャに含めておく。)

- [ ] **Step 4: テスト通過を確認**

Run: `cd clients/python && python -m pytest tests/test_rules.py -v`
Expected: 全テスト PASS

- [ ] **Step 5: コミット**

```bash
git add clients/python/ssl_client/rules.py clients/python/tests/test_rules.py
git commit -m "制約フィルタrules.pyを追加(速度上限・ボール退避・キック抑止)"
```

---

### Task 5: rules.py — ディフェンスエリア進入禁止と場外復帰

**Files:**
- Modify: `clients/python/ssl_client/rules.py`
- Test: `clients/python/tests/test_rules.py`

**Interfaces:**
- Consumes: Task 4 の `apply_rule_constraints` と `FieldGeometry`(penalty_area_depth/width)
- Produces: 同関数に常時制約を追加 — (a) フィールド外のロボットはフィールド内へ戻る速度に上書き、(b) キーパー以外が自陣ディフェンスエリア内なら退出速度に上書き、(c) 全ロボットが敵ディフェンスエリア内なら退出速度に上書き。優先順位: 場外復帰 > エリア退出 > ボール退避 > キック抑止 > 速度上限。

- [ ] **Step 1: 失敗するテストを書く**

`tests/test_rules.py` の末尾に追加:

```python
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
```

- [ ] **Step 2: テストが失敗することを確認**

Run: `cd clients/python && python -m pytest tests/test_rules.py -v`
Expected: 新規4テストが FAIL

- [ ] **Step 3: 実装**

`ssl_client/rules.py` にヘルパーを追加:

```python
def _defense_area_bounds(geometry, positive_x: bool):
    half_len = geometry.field_length / 2.0
    half_w = geometry.penalty_area_width / 2.0
    if positive_x:
        return half_len - geometry.penalty_area_depth, half_len, -half_w, half_w
    return -half_len, -half_len + geometry.penalty_area_depth, -half_w, half_w


def _push_out_of_rect(x, y, x_min, x_max, y_min, y_max, goal_line_at_max_x: bool):
    """Push toward the nearest legal exit face of the rect. The face on the
    goal-line side is never a legal exit (it leads out of the field)."""
    if not (x_min <= x <= x_max and y_min <= y <= y_max):
        return None
    exits = []
    if not goal_line_at_max_x:
        exits.append((x_max - x, (1.0, 0.0)))
    else:
        exits.append((x - x_min, (-1.0, 0.0)))
    exits.append((y - y_min, (0.0, -1.0)))
    exits.append((y_max - y, (0.0, 1.0)))
    _, (dx, dy) = min(exits, key=lambda e: e[0])
    return dx * RETREAT_SPEED_MPS, dy * RETREAT_SPEED_MPS


def _keep_out_push(robot, geometry, defend_positive_x: bool, is_keeper: bool):
    """Always-on constraints: return an override velocity, or None."""
    half_len = geometry.field_length / 2.0
    half_w = geometry.field_width / 2.0
    if abs(robot.x) > half_len or abs(robot.y) > half_w:
        tx = max(-half_len, min(half_len, robot.x))
        ty = max(-half_w, min(half_w, robot.y))
        return _away_velocity(robot.x, robot.y, tx, ty)  # toward clamped point
    if not is_keeper:
        push = _push_out_of_rect(
            robot.x, robot.y,
            *_defense_area_bounds(geometry, defend_positive_x),
            goal_line_at_max_x=defend_positive_x,
        )
        if push is not None:
            return push
    return _push_out_of_rect(
        robot.x, robot.y,
        *_defense_area_bounds(geometry, not defend_positive_x),
        goal_line_at_max_x=not defend_positive_x,
    )
```

注意: `_away_velocity(from_x, from_y, robot_x, robot_y)` は「from から robot が離れる向き」を返す。場外復帰は逆に「robot がフィールド内の最近点 (tx,ty) へ向かう」必要があるため、引数順を `_away_velocity(robot.x, robot.y, tx, ty)` として反転利用している(robot位置から見て tx,ty 方向へ向かう速度になる)。

`apply_rule_constraints` のループ内、`if phase is Phase.HALT:` の `else:` 分岐の**先頭**(ボール退避より前)に挿入:

```python
        else:
            push = _keep_out_push(robot, geometry, defend_positive_x,
                                  cmd.robot_id == keeper_id)
            if push is not None:
                vx, vy = push
            elif ball is not None and phase in _BALL_KEEP_OUT_PHASES and not exempt:
                ...  # 既存のボール退避ブロック(elif に変更)
```

既存のボール退避 `if` を `elif` に変更するだけで、キック抑止・速度上限のブロックはその後にそのまま残す。

- [ ] **Step 4: テスト通過を確認**

Run: `cd clients/python && python -m pytest tests/test_rules.py -v`
Expected: 全テスト PASS

- [ ] **Step 5: コミット**

```bash
git add clients/python/ssl_client/rules.py clients/python/tests/test_rules.py
git commit -m "rules.pyにディフェンスエリア進入禁止と場外復帰を追加"
```

---

### Task 6: referee.py — 最新メッセージ公開APIへ整理

**Files:**
- Modify: `clients/python/ssl_client/referee.py`
- Modify: `clients/python/tests/test_referee.py`

**Interfaces:**
- Consumes: 既存 `RefereeReceiver`(`latest` 属性は既にある)
- Produces: `RefereeReceiver` は `latest`(最新Refereeメッセージ or None)のみ公開。`is_running()` / `is_match_running()` / `_STOPPED_COMMANDS` は削除(解釈は game_state.py に一元化)。`parse_referee_packet` / `handle_packet` / `run_forever` は維持。

- [ ] **Step 1: テストを更新(先に赤にする)**

`tests/test_referee.py` を読み、`is_running` / `is_match_running` に依存するテストを削除し、以下へ置き換え:

```python
def test_receiver_starts_with_no_message():
    receiver = RefereeReceiver()
    assert receiver.latest is None


def test_handle_packet_updates_latest():
    msg = referee_pb2.Referee()
    msg.command = referee_pb2.Referee.STOP
    msg.command_counter = 7
    # (proto2必須フィールドでSerializeToStringが失敗する場合は、
    #  test_game_state.py の _msg ヘルパーと同じ初期化を行うこと)
    raw = msg.SerializeToString()
    receiver = RefereeReceiver()
    receiver.handle_packet(raw)
    assert receiver.latest.command == referee_pb2.Referee.STOP
    assert receiver.latest.command_counter == 7
```

既存のimport行・`parse_referee_packet` のテストは維持する。

- [ ] **Step 2: 実装が旧APIのままテストが通ることを確認 → 旧API削除で赤→緑を確認**

Run: `cd clients/python && python -m pytest tests/test_referee.py -v`
(この時点で新テストはPASSするはず — `latest` は既存。次のStepで旧APIを削除し、他モジュールが壊れないことを全体テストで確認する)

- [ ] **Step 3: 旧APIを削除**

`ssl_client/referee.py` から `_STOPPED_COMMANDS`、`is_match_running`、`RefereeReceiver.is_running` を削除。残るのは:

```python
from typing import Optional

from state import ssl_gc_referee_message_pb2 as referee_pb2

from .net import open_multicast_socket


def parse_referee_packet(raw: bytes) -> referee_pb2.Referee:
    msg = referee_pb2.Referee()
    msg.ParseFromString(raw)
    return msg


class RefereeReceiver:
    def __init__(self, group: str = "224.5.23.1", port: int = 10003):
        self.group = group
        self.port = port
        self.latest: Optional[referee_pb2.Referee] = None

    def handle_packet(self, raw: bytes) -> None:
        self.latest = parse_referee_packet(raw)

    def run_forever(self) -> None:
        sock = open_multicast_socket(self.group, self.port)
        while True:
            raw, _ = sock.recvfrom(65536)
            self.handle_packet(raw)
```

注意: この時点で `team_blue.py` / `team_yellow.py` と `strategy.py` はまだ `is_running()` / `referee_running` を参照しており全体テストは壊れる。**このタスクでは `tests/test_referee.py` の通過のみ確認**し、全体の配線は Task 7/10 で直す(Task 7 が strategy を、Task 10 がエントリポイントを更新する)。全体テストがこのタスク単体で赤くなるのを避けたい場合は、`python -m pytest tests/test_referee.py tests/test_game_state.py tests/test_rules.py -v` のみ実行して確認する。

- [ ] **Step 4: 対象テスト通過を確認**

Run: `cd clients/python && python -m pytest tests/test_referee.py -v`
Expected: PASS

- [ ] **Step 5: コミット**

```bash
git add clients/python/ssl_client/referee.py clients/python/tests/test_referee.py
git commit -m "referee.pyを最新メッセージ公開APIに整理(解釈はgame_stateへ一元化)"
```

---

### Task 7: strategy.py — GameStateベースのディスパッチ(HALT/STOP/RUNNING)

**Files:**
- Modify: `clients/python/ssl_client/strategy.py`
- Modify: `clients/python/tests/test_strategy.py`

**Interfaces:**
- Consumes: `GameState`/`Phase`(game_state.py)、既存の役割・状態機械ロジック
- Produces:
  - `TeamStrategy.compute_commands(world: WorldModel, game_state: GameState) -> list[RobotCommand]`(旧 `referee_running: bool` 引数は廃止)
  - `TeamStrategy.goalkeeper_id` プロパティ(`Optional[int]`、エントリポイントが rules.py へ渡す)
  - `TeamStrategy.rule_exempt_ids: frozenset`(毎tick更新。このタスクでは常に空。Task 8 でキッカーIDが入る)
  - 内部: 既存の通常プレー本体を `_run_play(own_robots, opponents, ball, geometry)` へ切り出し。`_stop_commands(own_robots, ball, geometry, world)` を追加(キーパー+フォーメーション、kick/dribbleなし)。

- [ ] **Step 1: 既存テストの呼び出しを新シグネチャに更新し、新テストを追加**

`tests/test_strategy.py` を読み、`compute_commands(world, True)` / `compute_commands(world, False)` 形式の呼び出しをすべて次へ機械的に置換:

- `compute_commands(world, True)` → `compute_commands(world, GameState(Phase.RUNNING, may_kick=True))`
- `compute_commands(world, False)` → `compute_commands(world, GameState(Phase.HALT))`

ファイル冒頭に `from ssl_client.game_state import GameState, Phase` を追加。さらに新テストを追加:

```python
def _running():
    return GameState(Phase.RUNNING, may_kick=True)


def test_stop_phase_moves_to_formation_without_kicks(make_world):
    # make_world: 既存テストのworld構築ヘルパーに合わせること(先にファイルを読む)。
    # 意図: STOPではコマンドは出るが kick_speed==0 かつ dribble==False。
    world = make_world()
    strategy = TeamStrategy(TeamConfig(is_team_yellow=True, defend_positive_x=False))
    commands = strategy.compute_commands(world, GameState(Phase.STOP))
    assert commands  # robots still reposition during STOP
    assert all(c.kick_speed == 0.0 and not c.dribble for c in commands)


def test_halt_phase_sends_zero_velocities(make_world):
    world = make_world()
    strategy = TeamStrategy(TeamConfig(is_team_yellow=True, defend_positive_x=False))
    commands = strategy.compute_commands(world, GameState(Phase.HALT))
    assert all(c.vel_x == 0.0 and c.vel_y == 0.0 for c in commands)


def test_rule_exempt_ids_is_empty_in_running(make_world):
    world = make_world()
    strategy = TeamStrategy(TeamConfig(is_team_yellow=True, defend_positive_x=False))
    strategy.compute_commands(world, _running())
    assert strategy.rule_exempt_ids == frozenset()
```

(`make_world` は既存テストのworld構築パターンに合わせて調整する — 既存テストが fixture でなく直接構築している場合は同じ構築コードをコピーする。)

- [ ] **Step 2: テストが失敗することを確認**

Run: `cd clients/python && python -m pytest tests/test_strategy.py -v`
Expected: FAIL(TypeError: compute_commands引数 / AttributeError等)

- [ ] **Step 3: 実装**

`ssl_client/strategy.py` の変更点:

1. import追加: `from .game_state import GameState, Phase`
2. `__init__` に `self.rule_exempt_ids: frozenset = frozenset()` を追加
3. `goalkeeper_id` プロパティを追加:

```python
    @property
    def goalkeeper_id(self):
        return self._goalkeeper_id
```

4. `compute_commands` を次の形へ書き換え(既存本体の大部分は `_run_play` へ移動):

```python
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

        # Any non-running, non-halt phase: strategy repositions; rules.py
        # enforces distances/speed on top. Set-piece placement is refined
        # in a later task; STOP-style formation is the safe default.
        self._enter_chase()
        return self._stop_commands(world, own_robots, ball, geometry)
```

5. `_run_play(self, world, own_robots, opponents, non_keeper_ids, ball, geometry)` を新設し、旧 `compute_commands` の holder検出〜コマンド生成〜`_pending_pass` 適用(旧 lines 114-168 相当)をそのまま移す(`referee_running` 分岐と snapshot/keeper割当部分は移さない — 上で済んでいる)。
6. `_stop_commands` を新設:

```python
    def _stop_commands(self, world, own_robots, ball, geometry):
        commands = []
        forward_sign = -1.0 if self._config.defend_positive_x else 1.0
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
```

- [ ] **Step 4: テスト通過を確認**

Run: `cd clients/python && python -m pytest tests/test_strategy.py tests/test_game_state.py tests/test_rules.py -v`
Expected: 全テスト PASS

- [ ] **Step 5: コミット**

```bash
git add clients/python/ssl_client/strategy.py clients/python/tests/test_strategy.py
git commit -m "strategyをGameStateディスパッチに変更(HALT/STOP/RUNNING)"
```

---

### Task 8: strategy.py — キックオフ・PKの配置とキック実行

**Files:**
- Modify: `clients/python/ssl_client/strategy.py`
- Test: `clients/python/tests/test_strategy.py`

**Interfaces:**
- Consumes: Task 7 のディスパッチ構造、`roles.seek/face/apply_separation/formation_target/goalkeeper_target`、`evaluation.SHOOT_KICK_SPEED_MPS`
- Produces:
  - `KICKOFF_OURS/THEIRS`, `PENALTY_OURS/THEIRS` の専用配置。定数 `KICKOFF_KICKER_STANDOFF_M = 0.2`, `PENALTY_RETREAT_M = 1.0`, `OWN_HALF_MARGIN_M = 0.2`
  - 自チームset pieceでは `rule_exempt_ids = frozenset({kicker_id})`
  - `may_kick=True` のときキッカーはボールへ寄り、ゴールセンターへ向いて整列したら `SHOOT_KICK_SPEED_MPS` でキック

- [ ] **Step 1: 失敗するテストを書く**

`tests/test_strategy.py` に追加(world構築は既存ヘルパーに合わせる。想定: yellowチーム、defend_positive_x=False → 自陣-x側、ボール中央(0,0)):

```python
def test_kickoff_ours_nominates_exactly_one_exempt_kicker(make_world):
    world = make_world()
    strategy = TeamStrategy(TeamConfig(is_team_yellow=True, defend_positive_x=False))
    commands = strategy.compute_commands(world, GameState(Phase.KICKOFF_OURS))
    assert len(strategy.rule_exempt_ids) == 1
    kicker_id = next(iter(strategy.rule_exempt_ids))
    assert kicker_id != strategy.goalkeeper_id
    assert commands  # everyone gets a command


def test_kickoff_ours_without_normal_start_does_not_kick(make_world):
    world = make_world()
    strategy = TeamStrategy(TeamConfig(is_team_yellow=True, defend_positive_x=False))
    commands = strategy.compute_commands(world, GameState(Phase.KICKOFF_OURS, may_kick=False))
    assert all(c.kick_speed == 0.0 for c in commands)


def test_kickoff_theirs_keeps_everyone_in_own_half(make_world):
    world = make_world()
    strategy = TeamStrategy(TeamConfig(is_team_yellow=True, defend_positive_x=False))
    strategy.compute_commands(world, GameState(Phase.KICKOFF_THEIRS))
    # own half is x < 0; targets are not directly observable, so verify by
    # running the tick and checking no command pushes a robot already at
    # x >= 0 further into the opponent half.
    for cmd in strategy.compute_commands(world, GameState(Phase.KICKOFF_THEIRS)):
        robot = world.yellow_robots[cmd.robot_id]
        if robot.x >= 0.0:
            assert cmd.vel_x <= 0.0


def test_penalty_theirs_places_keeper_on_goal_line(make_world):
    world = make_world()
    strategy = TeamStrategy(TeamConfig(is_team_yellow=True, defend_positive_x=False))
    commands = strategy.compute_commands(world, GameState(Phase.PENALTY_THEIRS))
    keeper_cmd = next(c for c in commands if c.robot_id == strategy.goalkeeper_id)
    keeper = world.yellow_robots[strategy.goalkeeper_id]
    # keeper is commanded toward own goal line (x = -field_length/2)
    if keeper.x > -world.geometry.field_length / 2.0 + 0.05:
        assert keeper_cmd.vel_x < 0.0
```

- [ ] **Step 2: テストが失敗することを確認**

Run: `cd clients/python && python -m pytest tests/test_strategy.py -v`
Expected: 新規テストが FAIL(現状は全set pieceが `_stop_commands` に落ちるため exempt が空/配置が違う)

- [ ] **Step 3: 実装**

`strategy.py` に定数とメソッドを追加:

```python
KICKOFF_KICKER_STANDOFF_M = 0.2  # kicker waits this far on our side of the ball
PENALTY_RETREAT_M = 1.0  # non-kicker robots stay this far behind the ball
OWN_HALF_MARGIN_M = 0.2  # clamp margin from the halfway line
```

`compute_commands` のディスパッチ(Task 7 で `_stop_commands` に落としていた部分)を置換:

```python
        if phase in (Phase.KICKOFF_OURS, Phase.PENALTY_OURS):
            return self._set_piece_ours(phase, game_state, world, own_robots,
                                        non_keeper_ids, ball, geometry)
        if phase is Phase.KICKOFF_THEIRS:
            return self._kickoff_theirs(world, own_robots, ball, geometry)
        if phase is Phase.PENALTY_THEIRS:
            return self._penalty_theirs(world, own_robots, ball, geometry)
        self._enter_chase()
        return self._stop_commands(world, own_robots, ball, geometry)
```

メソッド実装:

```python
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
                commands.append(self._kicker_command(
                    rid, robot, ball, own_robots, game_state.may_kick, goal_xy))
            elif phase is Phase.PENALTY_OURS:
                target_x = ball.x - forward_sign * PENALTY_RETREAT_M
                vx, vy = seek(robot.x, robot.y, target_x, robot.y)
                vx, vy = apply_separation(rid, vx, vy, robot, own_robots)
                commands.append(RobotCommand(rid, vx, vy, 0.0, robot.orientation))
            else:  # KICKOFF_OURS support robots: formation clamped to own half
                commands.append(self._own_half_formation_command(
                    rid, robot, world, ball, own_robots, forward_sign))
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
```

import行に `SHOOT_KICK_SPEED_MPS` を追加: `from .evaluation import SHOOT_KICK_SPEED_MPS, choose_action`

- [ ] **Step 4: テスト通過を確認**

Run: `cd clients/python && python -m pytest tests/test_strategy.py -v`
Expected: 全テスト PASS

- [ ] **Step 5: コミット**

```bash
git add clients/python/ssl_client/strategy.py clients/python/tests/test_strategy.py
git commit -m "キックオフ・PKの配置とキック実行を追加"
```

---

### Task 9: strategy.py — フリーキック・2度触り禁止・プレースメント退避

**Files:**
- Modify: `clients/python/ssl_client/strategy.py`
- Test: `clients/python/tests/test_strategy.py`

**Interfaces:**
- Consumes: Task 8 の `_choose_kicker` / `_kicker_command` / `_stop_commands`、既存 `_holder_command` / `POSSESSION_DIST_M`
- Produces:
  - `FREE_KICK_OURS`: キッカー1台(exempt)が `_holder_command` 相当でボールへ寄りキック、他は `_support_command`。キック発行時に `self._forbidden_toucher_id = kicker_id` を記録
  - `FREE_KICK_THEIRS` / `BALL_PLACEMENT_*`: `_stop_commands`(距離はrules.pyが強制)
  - RUNNING 中: `_forbidden_toucher_id` のロボットは chaser/holder 候補から除外。他のロボット(敵味方問わず)がボールから `POSSESSION_DIST_M` 以内に来たら解除

- [ ] **Step 1: 失敗するテストを書く**

```python
def test_free_kick_ours_has_exempt_kicker_and_records_double_touch(make_world):
    world = make_world_with_kicker_on_ball()  # キッカーがボール脇・ゴールに整列済みのworld
    strategy = TeamStrategy(TeamConfig(is_team_yellow=True, defend_positive_x=False))
    commands = strategy.compute_commands(world, GameState(Phase.FREE_KICK_OURS, may_kick=True))
    kicker_id = next(iter(strategy.rule_exempt_ids))
    kicker_cmd = next(c for c in commands if c.robot_id == kicker_id)
    assert kicker_cmd.kick_speed > 0.0
    assert strategy._forbidden_toucher_id == kicker_id


def test_forbidden_toucher_excluded_from_chase(make_world):
    world = make_world()
    strategy = TeamStrategy(TeamConfig(is_team_yellow=True, defend_positive_x=False))
    # ボールに一番近い非キーパーを禁止指定し、chaserに選ばれないことを確認
    running = GameState(Phase.RUNNING, may_kick=True)
    commands = strategy.compute_commands(world, running)
    nearest = min(
        (rid for rid in world.yellow_robots if rid != strategy.goalkeeper_id),
        key=lambda rid: math.hypot(world.yellow_robots[rid].x - world.ball.x,
                                   world.yellow_robots[rid].y - world.ball.y))
    strategy._forbidden_toucher_id = nearest
    commands = strategy.compute_commands(world, running)
    # 禁止ロボットへのコマンドがボールへ向かうchaser挙動でない(=dribble無効)ことで確認
    forbidden_cmd = next(c for c in commands if c.robot_id == nearest)
    assert not forbidden_cmd.dribble


def test_forbidden_toucher_cleared_when_someone_else_reaches_ball(make_world):
    world = make_world_with_opponent_on_ball()  # 敵ロボットがボールに接触距離
    strategy = TeamStrategy(TeamConfig(is_team_yellow=True, defend_positive_x=False))
    strategy._forbidden_toucher_id = 1
    strategy.compute_commands(world, GameState(Phase.RUNNING, may_kick=True))
    assert strategy._forbidden_toucher_id is None
```

(`make_world_with_kicker_on_ball` / `make_world_with_opponent_on_ball` は既存のworld構築コードを流用し、位置だけ調整して定義する。)

- [ ] **Step 2: テストが失敗することを確認**

Run: `cd clients/python && python -m pytest tests/test_strategy.py -v`
Expected: 新規テストが FAIL

- [ ] **Step 3: 実装**

1. `__init__` に `self._forbidden_toucher_id: Optional[int] = None` を追加
2. ディスパッチに追加(Task 8 のディスパッチの `self._enter_chase()` フォールバックの前):

```python
        if phase is Phase.FREE_KICK_OURS:
            return self._free_kick_ours(game_state, world, own_robots, opponents,
                                        non_keeper_ids, ball, geometry)
```

(`FREE_KICK_THEIRS` / `BALL_PLACEMENT_*` はフォールバックの `_stop_commands` のままでよい)

3. メソッド追加:

```python
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
```

4. `_run_play` 内の変更:
   - 冒頭に解除判定を追加:

```python
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
```

   - holder候補(`holders = [...]`)とchaser候補(`chaser_id = min(non_keeper_ids, ...)`)の対象リストから `self._forbidden_toucher_id` を除外:

```python
        eligible_ids = [rid for rid in non_keeper_ids if rid != self._forbidden_toucher_id]
```

を作り、holders生成とchaser選択の両方で `non_keeper_ids` の代わりに `eligible_ids` を使う(空の場合の `min` 呼び出しガードに注意 — `if ... and eligible_ids:`)。

- [ ] **Step 4: テスト通過を確認**

Run: `cd clients/python && python -m pytest tests/ -v`
Expected: 全テスト PASS(このタスク以降は全体スイートを回す。エントリポイントはテスト対象外なので壊れていても影響しない)

- [ ] **Step 5: コミット**

```bash
git add clients/python/ssl_client/strategy.py clients/python/tests/test_strategy.py
git commit -m "フリーキック・2度触り禁止・プレースメント退避を追加"
```

---

### Task 10: エントリポイント配線と全体テスト

**Files:**
- Modify: `clients/python/team_blue.py`
- Modify: `clients/python/team_yellow.py`
- Modify: `clients/python/ssl_client/__init__.py`(公開シンボル追記が必要な場合のみ)

**Interfaces:**
- Consumes: `GameStateTracker`(Task 3)、`apply_rule_constraints`(Task 4-5)、新 `compute_commands`(Task 7-9)、`strategy.goalkeeper_id` / `strategy.rule_exempt_ids`
- Produces: 動作する2エントリポイント。メインループは tracker→strategy→rules→sender の順

- [ ] **Step 1: team_yellow.py のメインループを更新**

```python
from ssl_client.commands import CommandSender
from ssl_client.game_state import GameStateTracker
from ssl_client.referee import RefereeReceiver
from ssl_client.rules import apply_rule_constraints
from ssl_client.strategy import TeamConfig, TeamStrategy
from ssl_client.vision import VisionReceiver
from ssl_client.world import WorldModel
```

main() 内、ループ前に `tracker = GameStateTracker(is_team_yellow=TEAM_IS_YELLOW)` を追加し、ループ本体を置換:

```python
    try:
        while True:
            game_state = tracker.update(referee.latest, world.ball)
            commands = strategy.compute_commands(world, game_state)
            if commands and world.geometry is not None:
                own = world.yellow_robots if TEAM_IS_YELLOW else world.blue_robots
                commands = apply_rule_constraints(
                    commands, game_state,
                    own_robots=dict(own),
                    ball=world.ball,
                    geometry=world.geometry,
                    defend_positive_x=not args.defend_negative_x,
                    keeper_id=strategy.goalkeeper_id,
                    exempt_ids=strategy.rule_exempt_ids,
                )
                sender.send(TEAM_IS_YELLOW, commands)
            time.sleep(period)
```

- [ ] **Step 2: team_blue.py に同じ変更を適用**

team_blue.py は `TEAM_IS_YELLOW = False` である以外同型。同じ変更を適用する。

- [ ] **Step 3: 起動スモークテスト**

Run: `cd clients/python && python -c "import team_yellow, team_blue"` および `python team_yellow.py --help`
Expected: ImportError なし、usage表示

- [ ] **Step 4: 全体テストを実行**

Run: `cd clients/python && python -m pytest tests/ -v`
Expected: 全テスト PASS

- [ ] **Step 5: コミット**

```bash
git add clients/python/team_yellow.py clients/python/team_blue.py
git commit -m "エントリポイントをGameState+制約フィルタ構成に配線"
```

---

### Task 11: E2E検証手順書とREADME更新

**Files:**
- Create: `clients/python/docs/referee-compliance-verification.md`
- Modify: `clients/python/README.md`

**Interfaces:**
- Consumes: 完成した実装(Task 1-10)
- Produces: 手動E2E検証チェックリストと更新されたREADME

- [ ] **Step 1: 検証手順書を作成**

`clients/python/docs/referee-compliance-verification.md` を新規作成(dirも作成):

```markdown
# 審判準拠 手動E2E検証手順

前提: grSim起動済み(Vision 224.5.23.2:10020 / Command 20011)、
ssl-game-controller起動済み(Referee 224.5.23.1:10003, Web UI http://localhost:8081)。
WSL2の2ターミナルで `python3 team_yellow.py` と `python3 team_blue.py --defend-negative-x` を起動。

各項目をWeb UIから操作し、grSim画面で目視確認する:

- [ ] **Halt**: 全ロボットが即停止する
- [ ] **Stop**: 全ロボットがボールから0.5m以上離れ、ゆっくり(≤1.5m/s)フォーメーション位置へ移動する
- [ ] **Kickoff (Yellow) → Normal Start**: Yellowキッカー1台のみボールへ寄り、他は自陣半分に留まる。Normal Startでキックオフが蹴られ、通常プレーに移行する
- [ ] **Kickoff (Blue) → Normal Start**: Blue側も同様
- [ ] **Free Kick (Yellow)**: Blueロボット全機がボールから0.5m以上離れる。Yellowキッカーが蹴り、同じロボットが連続してボールに触れない(2度触り回避)
- [ ] **Free Kick (Blue)**: 逆側も同様
- [ ] **Penalty (Yellow) → Normal Start**: Blueキーパーがゴールライン上、両チームの他ロボットがボール後方1mへ退避。Normal StartでYellowが蹴る
- [ ] **Penalty (Blue) → Normal Start**: 逆側も同様
- [ ] **Ball Placement (どちらか)**: 全ロボットがボール→指定地点の線分から0.5m以上離れる
- [ ] **試合を数分間流す**: ディフェンスエリアに攻撃ロボットが進入しない(キーパー以外が自陣エリアに入らない・全ロボットが敵エリアに入らない)
- [ ] **game-controllerなしで起動**: refereeを止めてもフリープレーで動き続ける
```

- [ ] **Step 2: README更新**

`clients/python/README.md` を読み、審判対応の記述(現状 HALT/STOP のみ等の記載)を新機能に合わせて更新する。追加内容: 対応する審判状態一覧(spec の表を簡約)、`ssl_client/game_state.py` / `ssl_client/rules.py` のモジュール説明、検証手順書へのリンク。

- [ ] **Step 3: 全体テスト最終確認**

Run: `cd clients/python && python -m pytest tests/ -v`
Expected: 全テスト PASS

- [ ] **Step 4: コミット**

```bash
git add clients/python/docs/referee-compliance-verification.md clients/python/README.md
git commit -m "審判準拠のE2E検証手順書を追加しREADMEを更新"
```

---

## 完了条件

- 全pytestスイートがグリーン
- `docs/referee-compliance-verification.md` の全項目をユーザーが手動E2Eで確認(grSim + ssl-game-controller はユーザー環境のWSL2で実行)
