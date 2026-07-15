# 戦術的パスワーク 実装プラン

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `clients/python/` の戦略を、パスコース評価・受け手連携・オフボール動き直しを備えたポゼッション重視のパスワークに拡張する。

**Architecture:** 純粋関数の評価層(`evaluation.py`)とロール層(`roles.py`)を新設し、`strategy.py` は3状態(CHASE/POSSESS/PASS_IN_FLIGHT)のチーム状態機械とロール割当てのみを持つコーディネーターに縮小する。`world.py` にボール速度推定を追加する。

**Tech Stack:** Python 3.10(WSL2 Ubuntu-22.04 の venv)、pytest、protobuf(生成済み `_pb2` を使用、通信層は変更なし)

**Spec:** `docs/superpowers/specs/2026-07-15-passing-strategy-design.md`

## Global Constraints

- 作業ディレクトリ: worktree `.claude/worktrees/passing-strategy/` 配下の `clients/python/`。
- **テスト実行は必ず WSL 経由**(Windows Git Bash では venv のシンボリックリンクが壊れて動かない):
  ```
  wsl.exe -d Ubuntu-22.04 -- bash -lc "cd /mnt/d/LLMprojects/grSim/.claude/worktrees/passing-strategy/clients/python && ./venv/bin/pytest -v"
  ```
  以下、各ステップの `Run:` はこのラッパーの `&&` 以降のコマンド部分だけを記す。
- 既存28テストは常にグリーンを維持する。例外は Task 3 で `test_strategy.py` の `_apply_separation` の import 先を `ssl_client.roles` に変えることのみ(検証内容は不変)。
- `TeamStrategy(TeamConfig).compute_commands(world, referee_running) -> list[RobotCommand]` の外部シグネチャは変更しない。
- 数値定数はスペックの値をそのまま使う(各タスクのコードに埋め込み済み)。モジュール定数として定義し、マジックナンバーをロジック中に直書きしない。
- スペックからの意図的な追加が1点ある: `PASS_MIN_FLIGHT_S = 0.3`(キック直後は Vision がまだ遅いボールを報告するため、「ボール減速」による PASS_IN_FLIGHT 終了判定をキック後 0.3 秒間抑止する)。タイムアウト(2.0 s)と受け手保持による終了は抑止しない。
- コミットメッセージ末尾に `Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>` を付ける。

---

### Task 1: ボール速度推定(world.py)

**Files:**
- Modify: `clients/python/ssl_client/world.py`
- Test: `clients/python/tests/test_world.py`(追記)

**Interfaces:**
- Consumes: なし(最初のタスク)
- Produces: `BallObservation` に `vx: float = 0.0`, `vy: float = 0.0` フィールド(m/s、フィールド座標系)。`estimate_ball_velocity(prev: Optional[BallObservation], x: float, y: float, t_capture: float) -> tuple[float, float]`。後続タスクは `world.ball.vx` / `world.ball.vy` を読む。

- [ ] **Step 1: 環境準備(worktree 初回のみ)**

worktree には venv と生成済み protobuf がないので作る:

Run: `./scripts/generate_protos.sh && python3 -m venv venv && ./venv/bin/pip install -r requirements.txt && ./venv/bin/pytest -q`
Expected: 最後に `28 passed`

- [ ] **Step 2: 失敗するテストを書く**

`clients/python/tests/test_world.py` の `_make_detection_frame` に `t_capture` 引数を追加する(既存呼び出しは変更不要):

```python
def _make_detection_frame(ball_xy=None, blue=(), yellow=(), t_capture=123.456):
    frame = ssl_vision_detection_pb2.SSL_DetectionFrame()
    frame.frame_number = 1
    frame.t_capture = t_capture
    frame.t_sent = t_capture
    ...  # 以降は既存のまま
```

ファイル末尾に追記:

```python
def test_first_ball_observation_has_zero_velocity():
    world = WorldModel()
    update_from_detection_frame(world, _make_detection_frame(ball_xy=(0.0, 0.0), t_capture=0.0))

    assert world.ball.vx == 0.0
    assert world.ball.vy == 0.0


def test_ball_velocity_is_ema_smoothed_finite_difference():
    world = WorldModel()
    # 0.1 s ごとに +x へ 0.1 m 移動 => 生の速度 1.0 m/s
    update_from_detection_frame(world, _make_detection_frame(ball_xy=(0.0, 0.0), t_capture=0.0))
    update_from_detection_frame(world, _make_detection_frame(ball_xy=(100.0, 0.0), t_capture=0.1))
    # EMA(alpha=0.5): 0.5 * 1.0 + 0.5 * 0.0 = 0.5
    assert world.ball.vx == pytest.approx(0.5)
    assert world.ball.vy == pytest.approx(0.0)

    update_from_detection_frame(world, _make_detection_frame(ball_xy=(200.0, 0.0), t_capture=0.2))
    # 0.5 * 1.0 + 0.5 * 0.5 = 0.75
    assert world.ball.vx == pytest.approx(0.75)


def test_ball_velocity_held_when_dt_is_zero_or_frame_gap_too_large():
    from ssl_client.world import BallObservation, estimate_ball_velocity

    prev = BallObservation(x=0.0, y=0.0, t_capture=1.0, vx=0.5, vy=-0.2)
    # dt = 0
    assert estimate_ball_velocity(prev, 1.0, 1.0, 1.0) == (0.5, -0.2)
    # dt < 0(順序が乱れたフレーム)
    assert estimate_ball_velocity(prev, 1.0, 1.0, 0.5) == (0.5, -0.2)
    # dt > 0.5 s(フレーム落ち)
    assert estimate_ball_velocity(prev, 1.0, 1.0, 2.0) == (0.5, -0.2)


def test_estimate_ball_velocity_none_prev_returns_zero():
    from ssl_client.world import estimate_ball_velocity

    assert estimate_ball_velocity(None, 1.0, 2.0, 0.0) == (0.0, 0.0)
```

- [ ] **Step 3: 失敗を確認する**

Run: `./venv/bin/pytest tests/test_world.py -v`
Expected: 新規4テストが FAIL(`vx` 属性なし / `estimate_ball_velocity` import 不可)、既存4テストは PASS

- [ ] **Step 4: 実装する**

`clients/python/ssl_client/world.py`:

`BallObservation` を差し替え:

```python
@dataclass
class BallObservation:
    x: float  # meters, field frame
    y: float  # meters, field frame
    t_capture: float
    vx: float = 0.0  # m/s, field frame, EMA-smoothed finite difference
    vy: float = 0.0  # m/s
```

`_MM_TO_M` の下にモジュール定数と関数を追加:

```python
_VELOCITY_EMA_ALPHA = 0.5
_MAX_FRAME_GAP_S = 0.5  # beyond this, treat as a dropped-frame gap and hold velocity


def estimate_ball_velocity(prev, x: float, y: float, t_capture: float):
    """Finite-difference ball velocity from the previous observation,
    EMA-smoothed. Holds the previous velocity across bad dt (<= 0 or
    dropped frames). Returns (0.0, 0.0) for the first observation."""
    if prev is None:
        return 0.0, 0.0
    dt = t_capture - prev.t_capture
    if dt <= 0.0 or dt > _MAX_FRAME_GAP_S:
        return prev.vx, prev.vy
    raw_vx = (x - prev.x) / dt
    raw_vy = (y - prev.y) / dt
    vx = _VELOCITY_EMA_ALPHA * raw_vx + (1.0 - _VELOCITY_EMA_ALPHA) * prev.vx
    vy = _VELOCITY_EMA_ALPHA * raw_vy + (1.0 - _VELOCITY_EMA_ALPHA) * prev.vy
    return vx, vy
```

`update_from_detection_frame` のボール処理を差し替え:

```python
    if detection.balls:
        ball = max(detection.balls, key=lambda b: b.confidence)
        x = ball.x * _MM_TO_M
        y = ball.y * _MM_TO_M
        vx, vy = estimate_ball_velocity(world.ball, x, y, detection.t_capture)
        world.ball = BallObservation(
            x=x, y=y, t_capture=detection.t_capture, vx=vx, vy=vy
        )
```

- [ ] **Step 5: 全テストが通ることを確認する**

Run: `./venv/bin/pytest -v`
Expected: 32 passed(28 + 4)

- [ ] **Step 6: コミット**

```bash
git add clients/python/ssl_client/world.py clients/python/tests/test_world.py
git commit -m "feat(clients/python): estimate ball velocity in world model"
```

---

### Task 2: パス/シュートコース評価(evaluation.py)

**Files:**
- Create: `clients/python/ssl_client/evaluation.py`
- Test: `clients/python/tests/test_evaluation.py`(新規)

**Interfaces:**
- Consumes: `RobotObservation`(`.x` / `.y` を持つオブジェクトなら何でもよい — テストでは本物を使う)
- Produces(後続タスクが使う正確なシグネチャ):
  - `lane_safety(from_xy: tuple, to_xy: tuple, opponents: Iterable) -> float`(0..1)
  - `pass_score(holder_xy: tuple, mate_xy: tuple, opponents: Iterable, goal_xy: tuple) -> float`
  - `@dataclass Action(kind: str, target_x: float, target_y: float, kick_speed: float, receiver_id: Optional[int] = None)` — `kind` は `"shoot" | "pass" | "dribble"`
  - `choose_action(ball_xy: tuple, teammates: Dict[int, RobotObservation], opponents: Iterable, goal_xy: tuple) -> Action` — `teammates` は**キーパーとホルダー自身を除いた**候補(呼び出し側の責務)

- [ ] **Step 1: 失敗するテストを書く**

`clients/python/tests/test_evaluation.py`(新規):

```python
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
```

- [ ] **Step 2: 失敗を確認する**

Run: `./venv/bin/pytest tests/test_evaluation.py -v`
Expected: 全テスト FAIL(`No module named 'ssl_client.evaluation'`)

- [ ] **Step 3: 実装する**

`clients/python/ssl_client/evaluation.py`(新規):

```python
"""Pure-function lane/shot evaluation for the passing strategy.

All coordinates are meters in the field (global) frame. All functions are
side-effect free so they can be unit tested with synthetic positions.
"""
import math
from dataclasses import dataclass
from typing import Dict, Iterable, Optional, Tuple

LANE_CAP_M = 0.5  # opponent distance at which a lane counts as fully open
W_DIST = 0.3  # weight of the pass-distance penalty
W_FWD = 0.3  # weight of the forward-progress bonus
PASS_DIST_MIN_M = 0.8
PASS_DIST_MAX_M = 4.0
FWD_NORM_M = 3.0  # forward progress saturates at this many meters gained
SHOOT_SCORE_MIN = 0.8
SHOOT_RANGE_M = 2.5
SHOOT_KICK_SPEED_MPS = 4.0
PASS_SCORE_MIN = 0.3
PASS_KICK_GAIN = 1.5  # kick speed per meter of pass distance
PASS_KICK_MIN_MPS = 1.5
PASS_KICK_MAX_MPS = 3.5


@dataclass
class Action:
    kind: str  # "shoot" | "pass" | "dribble"
    target_x: float  # aim point: goal center (shoot/dribble) or mate (pass)
    target_y: float
    kick_speed: float  # m/s; 0.0 for dribble
    receiver_id: Optional[int] = None  # set only for kind == "pass"


def point_to_segment_distance(
    px: float, py: float, ax: float, ay: float, bx: float, by: float
) -> float:
    abx, aby = bx - ax, by - ay
    ab_len_sq = abx * abx + aby * aby
    if ab_len_sq == 0.0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * abx + (py - ay) * aby) / ab_len_sq
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (ax + t * abx), py - (ay + t * aby))


def lane_safety(from_xy: Tuple[float, float], to_xy: Tuple[float, float], opponents: Iterable) -> float:
    """0..1: how clear the from->to segment is of opponents. 1.0 = wide open."""
    min_dist = None
    for opp in opponents:
        d = point_to_segment_distance(opp.x, opp.y, from_xy[0], from_xy[1], to_xy[0], to_xy[1])
        if min_dist is None or d < min_dist:
            min_dist = d
    if min_dist is None:
        return 1.0
    return min(min_dist, LANE_CAP_M) / LANE_CAP_M


def _dist_penalty(distance: float) -> float:
    if distance < PASS_DIST_MIN_M:
        return PASS_DIST_MIN_M - distance
    if distance > PASS_DIST_MAX_M:
        return distance - PASS_DIST_MAX_M
    return 0.0


def _forward_progress(
    holder_xy: Tuple[float, float], mate_xy: Tuple[float, float], goal_xy: Tuple[float, float]
) -> float:
    holder_to_goal = math.hypot(goal_xy[0] - holder_xy[0], goal_xy[1] - holder_xy[1])
    mate_to_goal = math.hypot(goal_xy[0] - mate_xy[0], goal_xy[1] - mate_xy[1])
    return max(0.0, min(1.0, (holder_to_goal - mate_to_goal) / FWD_NORM_M))


def pass_score(
    holder_xy: Tuple[float, float],
    mate_xy: Tuple[float, float],
    opponents: Iterable,
    goal_xy: Tuple[float, float],
) -> float:
    distance = math.hypot(mate_xy[0] - holder_xy[0], mate_xy[1] - holder_xy[1])
    return (
        lane_safety(holder_xy, mate_xy, opponents)
        - W_DIST * _dist_penalty(distance)
        + W_FWD * _forward_progress(holder_xy, mate_xy, goal_xy)
    )


def choose_action(
    ball_xy: Tuple[float, float],
    teammates: Dict[int, object],
    opponents: Iterable,
    goal_xy: Tuple[float, float],
) -> Action:
    """Pass-first action selection for the ball holder.

    `teammates` must already exclude the goalkeeper and the holder itself.
    `opponents` may be any iterable of objects with .x/.y (re-iterated, so
    pass a list, not a generator).
    """
    opponents = list(opponents)
    goal_dist = math.hypot(goal_xy[0] - ball_xy[0], goal_xy[1] - ball_xy[1])
    if lane_safety(ball_xy, goal_xy, opponents) > SHOOT_SCORE_MIN and goal_dist < SHOOT_RANGE_M:
        return Action("shoot", goal_xy[0], goal_xy[1], SHOOT_KICK_SPEED_MPS)

    best_id = None
    best_score = -math.inf
    for rid, mate in teammates.items():
        score = pass_score(ball_xy, (mate.x, mate.y), opponents, goal_xy)
        if score > best_score:
            best_id, best_score = rid, score
    if best_id is not None and best_score > PASS_SCORE_MIN:
        mate = teammates[best_id]
        distance = math.hypot(mate.x - ball_xy[0], mate.y - ball_xy[1])
        kick = max(PASS_KICK_MIN_MPS, min(PASS_KICK_MAX_MPS, distance * PASS_KICK_GAIN))
        return Action("pass", mate.x, mate.y, kick, receiver_id=best_id)

    return Action("dribble", goal_xy[0], goal_xy[1], 0.0)
```

- [ ] **Step 4: 全テストが通ることを確認する**

Run: `./venv/bin/pytest -v`
Expected: 45 passed(32 + 13)

- [ ] **Step 5: コミット**

```bash
git add clients/python/ssl_client/evaluation.py clients/python/tests/test_evaluation.py
git commit -m "feat(clients/python): add pass/shot lane evaluation"
```

---

### Task 3: 運動プリミティブを roles.py へ移動(リファクタリング)

**Files:**
- Create: `clients/python/ssl_client/roles.py`
- Modify: `clients/python/ssl_client/strategy.py`(移動した関数を import に置き換え)
- Modify: `clients/python/tests/test_strategy.py`(`_apply_separation` の import 先を2箇所変更)

**Interfaces:**
- Consumes: なし(独立リファクタリング)
- Produces(`roles.py` の公開名 — Task 5 が import する):
  - `normalize_angle(angle: float) -> float`
  - `seek(current_x, current_y, target_x, target_y) -> tuple[float, float]`
  - `face(current_orientation: float, target_orientation: float) -> tuple[float, float]`(`(vel_angular, angle_error)`)
  - `apply_separation(rid, vx, vy, robot, own_robots: dict) -> tuple[float, float]`
  - `own_goal_x(world, defend_positive_x: bool) -> float`
  - `goalkeeper_target(world, defend_positive_x: bool) -> tuple[float, float]`
  - `formation_target(world, defend_positive_x: bool, slot_index: int, ball_x: float) -> tuple[float, float]`
  - 定数: `SEEK_GAIN=2.0`, `MAX_SPEED=2.0`, `SEPARATION_DISTANCE=0.4`, `SEPARATION_GAIN=1.5`, `ANGLE_GAIN=3.0`, `MAX_ANGULAR_SPEED=4.0`, `FORMATION_PUSH_FORWARD=0.5`, `FORMATION_SLOTS`(既存の5スロット)

このタスクは**挙動を一切変えない**。既存 `strategy.py` の `_seek` / `_face` / `_normalize_angle` / `_apply_separation` / `_own_goal_x` / `_goalkeeper_target` / `_formation_target` と関連定数を、先頭のアンダースコアを外した公開名で `roles.py` に移し、`strategy.py` は import して使う。docstring・実装本体は文字どおりコピーする。

- [ ] **Step 1: roles.py を作る**

`clients/python/ssl_client/roles.py`(新規)— 内容は現行 `strategy.py` の該当部分の移動(名前だけ公開化):

```python
"""Per-role motion primitives and target computation.

Everything here is a pure function of observations -> velocities/targets,
so it can be unit tested without a running simulator.
"""
import math

SEEK_GAIN = 2.0
MAX_SPEED = 2.0  # m/s, conservative vs. grSim's configured VelAbsoluteMax=5
SEPARATION_DISTANCE = 0.4  # meters; own robots closer than this get pushed apart
SEPARATION_GAIN = 1.5
ANGLE_GAIN = 3.0
MAX_ANGULAR_SPEED = 4.0  # rad/s, conservative vs. grSim's configured VelAngularMax=20

FORMATION_PUSH_FORWARD = 0.5  # meters; extra forward shift when the ball is in the attacking half

# (forward_offset, lateral_offset) in meters, relative to the team's own goal
# line, for each non-keeper robot's fallback formation slot (assigned in
# ascending robot-id order, cycling if there are more robots than slots).
FORMATION_SLOTS = [
    (1.0, 0.0),
    (1.0, 1.2),
    (1.0, -1.2),
    (2.5, 0.8),
    (2.5, -0.8),
]


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def seek(current_x: float, current_y: float, target_x: float, target_y: float):
    vx = (target_x - current_x) * SEEK_GAIN
    vy = (target_y - current_y) * SEEK_GAIN
    speed = math.hypot(vx, vy)
    if speed > MAX_SPEED:
        scale = MAX_SPEED / speed
        vx *= scale
        vy *= scale
    return vx, vy


def face(current_orientation: float, target_orientation: float):
    """Returns (vel_angular, angle_error) to rotate from current_orientation
    toward target_orientation, clamped to MAX_ANGULAR_SPEED."""
    error = normalize_angle(target_orientation - current_orientation)
    vel_angular = error * ANGLE_GAIN
    if abs(vel_angular) > MAX_ANGULAR_SPEED:
        vel_angular = math.copysign(MAX_ANGULAR_SPEED, vel_angular)
    return vel_angular, error


def apply_separation(rid, vx: float, vy: float, robot, own_robots: dict):
    """Push a robot's commanded velocity away from own teammates that are
    closer than SEPARATION_DISTANCE, to avoid them stacking on top of each
    other. Not real path planning — just a simple repulsion term."""
    for other_id, other in own_robots.items():
        if other_id == rid:
            continue
        dx = robot.x - other.x
        dy = robot.y - other.y
        dist = math.hypot(dx, dy)
        if 0 < dist < SEPARATION_DISTANCE:
            push = (SEPARATION_DISTANCE - dist) * SEPARATION_GAIN
            vx += (dx / dist) * push
            vy += (dy / dist) * push
    speed = math.hypot(vx, vy)
    if speed > MAX_SPEED:
        scale = MAX_SPEED / speed
        vx *= scale
        vy *= scale
    return vx, vy


def own_goal_x(world, defend_positive_x: bool) -> float:
    half_length = world.geometry.field_length / 2.0
    return half_length if defend_positive_x else -half_length


def goalkeeper_target(world, defend_positive_x: bool):
    half_goal = world.geometry.goal_width / 2.0
    target_y = max(-half_goal, min(half_goal, world.ball.y))
    return own_goal_x(world, defend_positive_x), target_y


def formation_target(world, defend_positive_x: bool, slot_index: int, ball_x: float):
    """Fixed formation slot, relative to this team's own goal line, pushed
    FORMATION_PUSH_FORWARD further forward when the ball is in the
    opponent's half of the field (a fixed, symmetric field split around
    field-center x=0 - not the same axis as the goalkeeper's lateral
    tracking, which follows the ball's Y)."""
    forward_offset, lateral_offset = FORMATION_SLOTS[slot_index % len(FORMATION_SLOTS)]
    forward_sign = -1.0 if defend_positive_x else 1.0
    goal_x = own_goal_x(world, defend_positive_x)
    ball_in_attacking_half = (ball_x * forward_sign) > 0.0
    push_forward = FORMATION_PUSH_FORWARD if ball_in_attacking_half else 0.0
    target_x = goal_x + forward_sign * (forward_offset + push_forward)
    return target_x, lateral_offset
```

- [ ] **Step 2: strategy.py から移動元を削除して import に置き換える**

`clients/python/ssl_client/strategy.py` を以下の内容に差し替える(挙動は完全に同一。`_SEEK_GAIN` 等の定数と移動した7関数を削除し、`roles` から import。キック関連定数はこのタスクでは残す):

```python
import math
from dataclasses import dataclass
from typing import Dict, Optional

from .commands import RobotCommand
from .roles import (
    apply_separation,
    face,
    formation_target,
    goalkeeper_target,
    own_goal_x,
    seek,
)
from .world import BallObservation, WorldModel

_KICK_RANGE_M = 0.15
_KICK_SPEED_MPS = 4.0
_KICK_ANGLE_TOLERANCE = 0.35  # radians (~20 degrees)


@dataclass
class TeamConfig:
    is_team_yellow: bool
    defend_positive_x: bool  # True: this team's own goal is on the +X side


class TeamStrategy:
    def __init__(self, config: TeamConfig):
        self._config = config
        self._goalkeeper_id: Optional[int] = None
        self._formation_slots: Dict[int, int] = {}

    def compute_commands(self, world: WorldModel, referee_running: bool):
        # Snapshot: the Vision receiver runs on its own thread and mutates
        # world.*_robots concurrently with this method's iteration. Inserting a
        # newly-seen robot id mid-iteration would raise "dictionary changed size
        # during iteration".
        own_robots = dict(world.yellow_robots if self._config.is_team_yellow else world.blue_robots)
        if not own_robots or world.ball is None or world.geometry is None:
            return []

        if self._goalkeeper_id is None or self._goalkeeper_id not in own_robots:
            self._goalkeeper_id = min(own_robots.keys())

        non_keeper_ids = sorted(rid for rid in own_robots if rid != self._goalkeeper_id)
        for slot_index, rid in enumerate(non_keeper_ids):
            self._formation_slots.setdefault(rid, slot_index)

        attacker_id = None
        if non_keeper_ids:
            attacker_id = min(
                non_keeper_ids,
                key=lambda rid: math.hypot(
                    own_robots[rid].x - world.ball.x, own_robots[rid].y - world.ball.y
                ),
            )

        commands = []
        for rid, robot in own_robots.items():
            if not referee_running:
                commands.append(RobotCommand(rid, 0.0, 0.0, 0.0, robot.orientation))
                continue

            if rid == self._goalkeeper_id:
                target_x, target_y = goalkeeper_target(world, self._config.defend_positive_x)
                vx, vy = seek(robot.x, robot.y, target_x, target_y)
                vx, vy = apply_separation(rid, vx, vy, robot, own_robots)
                commands.append(RobotCommand(rid, vx, vy, 0.0, robot.orientation))
            elif rid == attacker_id:
                vx, vy = seek(robot.x, robot.y, world.ball.x, world.ball.y)
                vx, vy = apply_separation(rid, vx, vy, robot, own_robots)
                opponent_goal_x = -own_goal_x(world, self._config.defend_positive_x)
                target_orientation = math.atan2(0.0 - robot.y, opponent_goal_x - robot.x)
                vel_angular, angle_error = face(robot.orientation, target_orientation)
                dist = math.hypot(robot.x - world.ball.x, robot.y - world.ball.y)
                facing_goal = abs(angle_error) < _KICK_ANGLE_TOLERANCE
                kick = _KICK_SPEED_MPS if (dist < _KICK_RANGE_M and facing_goal) else 0.0
                commands.append(
                    RobotCommand(
                        rid, vx, vy, vel_angular, robot.orientation, kick_speed=kick, dribble=True
                    )
                )
            else:
                slot_index = self._formation_slots[rid]
                target_x, target_y = formation_target(
                    world, self._config.defend_positive_x, slot_index, world.ball.x
                )
                vx, vy = seek(robot.x, robot.y, target_x, target_y)
                vx, vy = apply_separation(rid, vx, vy, robot, own_robots)
                commands.append(RobotCommand(rid, vx, vy, 0.0, robot.orientation))

        return commands
```

- [ ] **Step 3: テストの import を追随させる(2箇所)**

`clients/python/tests/test_strategy.py` の `test_apply_separation_pushes_robots_apart_when_too_close` と `test_apply_separation_does_nothing_when_robots_are_far_apart` 内の

```python
    from ssl_client.strategy import _apply_separation
```

を両方

```python
    from ssl_client.roles import apply_separation as _apply_separation
```

に変える(アサーションは一切変えない)。

- [ ] **Step 4: 全テストが通ることを確認する**

Run: `./venv/bin/pytest -v`
Expected: 45 passed(挙動不変のリファクタリングなので増減なし)

- [ ] **Step 5: コミット**

```bash
git add clients/python/ssl_client/roles.py clients/python/ssl_client/strategy.py clients/python/tests/test_strategy.py
git commit -m "refactor(clients/python): move motion primitives to roles module"
```

---

### Task 4: レシーバーとサポートの目標計算(roles.py 追加)

**Files:**
- Modify: `clients/python/ssl_client/roles.py`(末尾に追加)
- Test: `clients/python/tests/test_roles.py`(新規)

**Interfaces:**
- Consumes: `lane_safety`(Task 2)、`BallObservation.vx/.vy`(Task 1)
- Produces(Task 5 が import する):
  - `receiver_target(robot_x: float, robot_y: float, ball) -> tuple[float, float]` — `ball` は `.x/.y/.vx/.vy` を持つ `BallObservation`
  - `support_target(base_xy: tuple, forward_sign: float, ball_xy: tuple, opponents: Iterable) -> tuple[float, float]`
  - 定数: `BALL_MOVING_MIN_SPEED_MPS = 0.1`, `SUPPORT_CANDIDATE_LATERAL_M = (-0.8, 0.0, 0.8)`, `SUPPORT_CANDIDATE_FORWARD_M = (0.0, 0.5)`

- [ ] **Step 1: 失敗するテストを書く**

`clients/python/tests/test_roles.py`(新規):

```python
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
```

- [ ] **Step 2: 失敗を確認する**

Run: `./venv/bin/pytest tests/test_roles.py -v`
Expected: 全テスト FAIL(`cannot import name 'receiver_target'`)

- [ ] **Step 3: 実装する**

`clients/python/ssl_client/roles.py` の import 部に `from .evaluation import lane_safety` を追加し、末尾に追記:

```python
BALL_MOVING_MIN_SPEED_MPS = 0.1  # below this the ball counts as stationary
SUPPORT_CANDIDATE_LATERAL_M = (-0.8, 0.0, 0.8)
SUPPORT_CANDIDATE_FORWARD_M = (0.0, 0.5)


def receiver_target(robot_x: float, robot_y: float, ball):
    """Intercept point for a pass receiver: the closest point to the robot
    on the ray from the ball along its velocity. Falls back to the ball
    position itself when the ball is (nearly) stationary."""
    speed = math.hypot(ball.vx, ball.vy)
    if speed < BALL_MOVING_MIN_SPEED_MPS:
        return ball.x, ball.y
    ux, uy = ball.vx / speed, ball.vy / speed
    t = (robot_x - ball.x) * ux + (robot_y - ball.y) * uy
    t = max(0.0, t)
    return ball.x + t * ux, ball.y + t * uy


def support_target(base_xy, forward_sign: float, ball_xy, opponents):
    """Off-ball repositioning: sample candidate points around the base
    formation slot and pick the one with the most open pass lane from the
    ball. Ties go to the candidate closest to the base slot."""
    opponents = list(opponents)
    best_key = None
    best_candidate = base_xy
    for forward in SUPPORT_CANDIDATE_FORWARD_M:
        for lateral in SUPPORT_CANDIDATE_LATERAL_M:
            candidate = (base_xy[0] + forward_sign * forward, base_xy[1] + lateral)
            safety = lane_safety(ball_xy, candidate, opponents)
            dist_to_base = math.hypot(candidate[0] - base_xy[0], candidate[1] - base_xy[1])
            key = (safety, -dist_to_base)  # maximize safety, then prefer near-base
            if best_key is None or key > best_key:
                best_key = key
                best_candidate = candidate
    return best_candidate
```

- [ ] **Step 4: 全テストが通ることを確認する**

Run: `./venv/bin/pytest -v`
Expected: 51 passed(45 + 6)

- [ ] **Step 5: コミット**

```bash
git add clients/python/ssl_client/roles.py clients/python/tests/test_roles.py
git commit -m "feat(clients/python): add receiver intercept and support repositioning"
```

---

### Task 5: チーム状態機械(strategy.py)と README 更新

**Files:**
- Modify: `clients/python/ssl_client/strategy.py`(全面書き換え)
- Modify: `clients/python/tests/test_strategy.py`(新規テスト追記。既存テストは import 済み変更以外いじらない)
- Modify: `clients/python/README.md`(挙動説明の更新)

**Interfaces:**
- Consumes: `choose_action` / `Action`(Task 2)、`roles` の全関数(Task 3, 4)、`world.ball.vx/.vy`(Task 1)
- Produces: 外部シグネチャ不変の `TeamStrategy`。テストから参照される内部状態: `strategy._state`(`"CHASE" | "POSSESS" | "PASS_IN_FLIGHT"`)、`strategy._receiver_id`、`strategy._goalkeeper_id`(既存)

**状態機械の仕様(スペックの要約 + 実装上の確定事項):**

- 保持判定: ロボットとボールの距離 < `POSSESSION_DIST_M`(0.12)かつボール速度 < `SLOW_BALL_SPEED_MPS`(0.5)。
- 毎 tick、PASS_IN_FLIGHT 以外の状態は保持判定から直接導出する(保持者がいれば POSSESS、いなければ CHASE)。**CHASE→POSSESS は同一 tick 内で反映される**(既存テストの「アタッカーがボールのそばで即キックする」挙動を保持判定経由で再現するため)。
- POSSESS でパスをキックした tick の末尾で PASS_IN_FLIGHT に遷移し、`_receiver_id` と `_pass_kick_time`(ball.t_capture)を記録する。
- PASS_IN_FLIGHT の終了判定(tick 冒頭、優先順): 受け手が視界から消えた → 即 CHASE / 受け手が保持 → CHASE / 経過 ≥ `PASS_TIMEOUT_S`(2.0)→ CHASE / 経過 ≥ `PASS_MIN_FLIGHT_S`(0.3)かつボール速度 < 0.5 → CHASE。時刻は `world.ball.t_capture`(シミュレーション時刻。テストが決定的になる)。
- `referee_running == False`: 全ロボット速度ゼロ、状態を CHASE にリセット、`_receiver_id = None`。
- 敵チーム = `world` の反対色のロボット(スナップショットを取る)。

- [ ] **Step 1: 失敗するテストを書く**

`clients/python/tests/test_strategy.py` の `_world` ヘルパーを拡張(既存呼び出しに影響しないデフォルト引数で):

```python
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
```

ファイル末尾に追記:

```python
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
```

- [ ] **Step 2: 失敗を確認する**

Run: `./venv/bin/pytest tests/test_strategy.py -v`
Expected: 新規15テストの大半が FAIL(`_state` 属性なし等)。既存11テストは PASS のまま

- [ ] **Step 3: strategy.py を全面書き換えする**

`clients/python/ssl_client/strategy.py` 全体を以下に差し替え:

```python
import math
from dataclasses import dataclass
from typing import Dict, Optional

from .commands import RobotCommand
from .evaluation import choose_action
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

    # --- main entry -----------------------------------------------------

    def compute_commands(self, world: WorldModel, referee_running: bool):
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

        if not referee_running:
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
        holder_id = None
        holders = [rid for rid in non_keeper_ids if _holds_ball(own_robots[rid], ball)]
        if holders:
            holder_id = min(
                holders,
                key=lambda rid: math.hypot(
                    own_robots[rid].x - ball.x, own_robots[rid].y - ball.y
                ),
            )

        self._update_state(own_robots, ball, holder_id)

        chaser_id = None
        if self._state == STATE_CHASE and non_keeper_ids:
            chaser_id = min(
                non_keeper_ids,
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
                commands.append(self._keeper_command(rid, robot, world, own_robots))
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
        return commands

    # --- per-role command builders ---------------------------------------

    def _keeper_command(self, rid, robot, world, own_robots):
        target_x, target_y = goalkeeper_target(world, self._config.defend_positive_x)
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
            self._state = STATE_PASS_IN_FLIGHT
            self._receiver_id = action.receiver_id
            self._pass_kick_time = ball.t_capture
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
```

**既存テストへの影響の見取り図(実装者の参考):** 既存の「アタッカー」系テストはボールが 5〜10 cm 先にある配置なので、新実装では保持判定が真になり POSSESS のホルダー分岐(seek ボール + shoot/pass/dribble)を通る。敵ロボットがいないためシュート(ゴール至近)かドリブル(味方なし)かパス(味方あり)になり、各テストのアサーション(vel_x の符号、kick_speed の有無、vel_angular の符号)は新実装でもそのまま成立する。`test_attacker_rotates_to_face_the_opponent_goal` はパス候補が0台なのでドリブル分岐(ゴールを向く)を通り、従来と同じ回転方向になる。

- [ ] **Step 4: 全テストが通ることを確認する**

Run: `./venv/bin/pytest -v`
Expected: 66 passed(51 + 15)。既存テストが落ちた場合は、上の見取り図と実際の分岐を突き合わせて実装側を直す(テストのアサーションを弱めない)

- [ ] **Step 5: README を更新する**

`clients/python/README.md` の「Verifying it works」セクションの手順1を以下に差し替え:

```markdown
1. With all three running, open the ssl-game-controller web UI and click
   **Force Start**. Both teams now play a possession-first passing game:
   the robot that reaches the ball first takes possession (dribbler on),
   turns toward the best-scoring option, and passes to an open teammate —
   shooting only when close to the goal with an open shot lane. While a
   pass is in flight, the designated receiver moves onto the ball's path
   to trap it. Robots without the ball shift around their formation slots
   to keep an open passing lane from the ball.
```

- [ ] **Step 6: コミット**

```bash
git add clients/python/ssl_client/strategy.py clients/python/tests/test_strategy.py clients/python/README.md
git commit -m "feat(clients/python): possession-first passing state machine"
```

---

## 完了後の手動検証(オーケストレーター実施)

SDD 完了後、コントローラーセッションが grSim + ssl-game-controller + 両クライアントを起動し、スペックの「検証方法(手動)」5項目を目視確認する。これはプランのタスクではない(subagent は実施しない)。
