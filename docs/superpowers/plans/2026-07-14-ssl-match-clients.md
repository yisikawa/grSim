# SSL 対戦クライアント(Python) 実装プラン

> **エージェント作業者向け:** 必須サブスキル: このプランをタスクごとに実行するには superpowers:subagent-driven-development(推奨)または superpowers:executing-plans を使用すること。各ステップはチェックボックス(`- [ ]`)構文で進捗を追跡する。

**目標:** 稼働中の grSim インスタンスと公式 `ssl-game-controller` に接続し、見た目にまとまりのあるプレー(ボールを追うアタッカー、ゴールキーパー、簡易フォーメーション、HALT/STOP での停止)を行う、2つの独立した Python クライアント(`team_blue.py`、`team_yellow.py`)を構築する。

**アーキテクチャ:** 共有の `ssl_client` パッケージ(ワールドモデル、Vision 受信、Referee 受信、コマンド組み立て/送信、戦略)を、チームカラーとデフォルトの守備サイドだけが異なる2つの薄いエントリポイントスクリプトが利用する。すべては WSL2 内のただの Python プロセスとして動作し、既存の Qt サンプルクライアントと同じ UDP/マルチキャストソケットで grSim と通信する。

**技術スタック:** Python 3.10(WSL2 Ubuntu 22.04 のシステム Python)、`protobuf`(pip)、`pytest`、システムの `protoc`(apt の `protobuf-compiler`、C++ ビルド用に既にインストール済み)。

## グローバル制約

- WSL2 Ubuntu 22.04 内、Windows マウントパス `/mnt/d/LLMprojects/grSim/clients/python` から実行する(承認済み設計書のとおり、Linux ネイティブファイルシステム下へのコピーは不要)。
- システムの `protoc` はバージョン 3.12.4(`protoc --version` で確認済み)。この生成 Python コードは descriptor ベースであり、`protobuf` pip パッケージのデフォルト(upb)バックエンドでは `TypeError: Descriptors cannot be created directly.` により**拒否される**。生成された `_pb2` モジュールを import する前に、すべてのエントリポイントとすべてのテスト実行で `PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python` を設定しなければならない(このリポジトリの環境で動作確認済み)。これは `ssl_client/__init__.py` で一度だけ設定する。
- `ssl_client` 内のすべての位置・ジオメトリ(つまり protobuf のパース境界のすぐ手前を除くすべての箇所)は**メートル**と**ラジアン**で扱う。grSim のワイヤーフォーマットは `SSL_DetectionBall`/`SSL_DetectionRobot`/`SSL_GeometryFieldSize` でミリメートルを、向きにはラジアンを使うため、mm → m の変換(`/1000.0`)は `ssl_client/world.py` の中で一度だけ正確に行う。
- grSim へ送信するコマンド速度(`veltangent`、`velnormal`)は**ロボット自身のローカル座標系**(tangent = 前方、normal = 左方向)であり、フィールド/グローバル座標系ではない — `src/robot.cpp:453-511` の `Robot::setSpeed` を読んで確認済み。ここでは `vx`/`vy` をホイール速度の計算式にそのまま使っており、向きに応じた回転は一切適用されていない。戦略コードはすべてグローバルなフィールド座標系で考え、`ssl_client/commands.py` だけがローカル座標系へ回転させる唯一の場所とする。
- コマンドは grSim へ `grSim_Packet`(本リポジトリの `src/proto/grSim_Packet.proto` / `grSim_Commands.proto`)として UDP ユニキャストで `127.0.0.1:20011` へ送信する。これは承認済み設計書および既存の `clients/qt` サンプルと一致するもので、新しいチームごとの `ssl-simulation-protocol` ポート(10301/10302)ではない。
- Vision のデフォルトはマルチキャスト `224.5.23.2:10020`(`ssl_vision_wrapper.proto`)、Referee のデフォルトはマルチキャスト `224.5.23.1:10003`(`ssl_gc_referee_message.proto`、`RoboCup-SSL/ssl-game-controller` から取得)。いずれも CLI フラグで上書き可能。
- 承認済み設計書のとおり、自動化されたエンドツーエンド/システムテストは存在しない。TDD は各モジュールの**純粋なロジック**(パース、ワールドモデルの更新、パケット組み立て、戦略の計算)に適用する。ソケット/スレッドの結合部分(`run_forever` ループ)は Task 8 で手動確認し、単体テストの対象にはしない。

---

### Task 1: プロジェクトの雛形、protobuf コード生成、共有ネットワークヘルパー

**Files:**
- Create: `clients/python/requirements.txt`
- Create: `clients/python/pytest.ini`
- Create: `clients/python/conftest.py`
- Create: `clients/python/scripts/generate_protos.sh`
- Create: `clients/python/ssl_client/__init__.py`
- Create: `clients/python/ssl_client/net.py`
- Create: `clients/python/.gitignore`

**Interfaces:**
- Produces: `ssl_client/pb/` ディレクトリ(git 管理外、生成物)。import 可能なフラットモジュール `grSim_Commands_pb2`、`grSim_Packet_pb2`、`grSim_Replacement_pb2`、`ssl_vision_geometry_pb2`、`ssl_vision_detection_pb2`、`ssl_vision_wrapper_pb2`、および `state/ssl_gc_referee_message_pb2`(とその依存先 `state/ssl_gc_game_event_pb2`、`state/ssl_gc_common_pb2`、`geom/ssl_gc_geometry_pb2`)を含む。`ssl_client` を import した後は `from state import ssl_gc_referee_message_pb2` として import できる。
- Produces: `ssl_client.net.open_multicast_socket(group: str, port: int) -> socket.socket` — Task 3 と Task 4 で使用。

- [ ] **Step 1: ディレクトリ構成と `.gitignore` を作る**

```bash
mkdir -p clients/python/scripts clients/python/ssl_client clients/python/tests
```

`clients/python/.gitignore` を作成:

```
pb/
venv/
__pycache__/
*.pyc
.pytest_cache/
```

- [ ] **Step 2: `requirements.txt` を書く**

この環境の `protoc`(3.12.4)と `PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python`
の回避策(グローバル制約を参照)に対して動作確認済みの正確なバージョンに
`protobuf` を固定する — `protobuf` の新しいメジャーバージョンは
gencode/ランタイムの互換性チェックが厳格化されており、ここでは未検証。

```
protobuf==7.35.1
pytest>=7.0
```

- [ ] **Step 3: proto 生成スクリプトを書く**

`clients/python/scripts/generate_protos.sh` を作成:

```bash
#!/usr/bin/env bash
# Regenerates clients/python/ssl_client/pb/ from this repo's own .proto files
# plus the SSL_Referee message tree fetched from ssl-game-controller.
# Safe to re-run; it wipes and rebuilds ssl_client/pb/ each time.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CLIENT_DIR="$(dirname "$SCRIPT_DIR")"
REPO_ROOT="$(cd "$CLIENT_DIR/../.." && pwd)"
PB_DIR="$CLIENT_DIR/ssl_client/pb"
GC_CHECKOUT="/tmp/ssl-game-controller-proto"

rm -rf "$PB_DIR"
mkdir -p "$PB_DIR"

echo "Generating grSim's own protobuf messages..."
protoc -I "$REPO_ROOT/src/proto" --python_out="$PB_DIR" \
  "$REPO_ROOT/src/proto/grSim_Commands.proto" \
  "$REPO_ROOT/src/proto/grSim_Packet.proto" \
  "$REPO_ROOT/src/proto/grSim_Replacement.proto" \
  "$REPO_ROOT/src/proto/ssl_vision_geometry.proto" \
  "$REPO_ROOT/src/proto/ssl_vision_detection.proto" \
  "$REPO_ROOT/src/proto/ssl_vision_wrapper.proto"

echo "Fetching ssl-game-controller's proto/ tree (sparse checkout, pinned commit)..."
# Pinned to a commit verified to contain this exact 4-file proto/state+geom
# dependency closure. Bump deliberately (and re-verify the closure) if you
# need a newer ssl-game-controller proto.
GC_COMMIT="c20fde58ecd4068836a5c8a1b9e34f923ad145d1"
rm -rf "$GC_CHECKOUT"
git clone --filter=blob:none --sparse \
  https://github.com/RoboCup-SSL/ssl-game-controller.git "$GC_CHECKOUT"
(cd "$GC_CHECKOUT" && git sparse-checkout set proto && git checkout "$GC_COMMIT")

echo "Generating SSL_Referee message tree..."
protoc -I "$GC_CHECKOUT/proto" --python_out="$PB_DIR" \
  "$GC_CHECKOUT/proto/state/ssl_gc_referee_message.proto" \
  "$GC_CHECKOUT/proto/state/ssl_gc_game_event.proto" \
  "$GC_CHECKOUT/proto/state/ssl_gc_common.proto" \
  "$GC_CHECKOUT/proto/geom/ssl_gc_geometry.proto"

touch "$PB_DIR/__init__.py" "$PB_DIR/state/__init__.py" "$PB_DIR/geom/__init__.py"

echo "Done. Generated files:"
find "$PB_DIR" -name '*.py' | sort
```

```bash
chmod +x clients/python/scripts/generate_protos.sh
```

- [ ] **Step 4: 生成スクリプトを実行し、出力を確認する**

Run: `cd clients/python && ./scripts/generate_protos.sh`

Expected: `Done. Generated files:` で終わり、続くファイル一覧に少なくとも `grSim_Commands_pb2.py`、`grSim_Packet_pb2.py`、`ssl_vision_wrapper_pb2.py`、`ssl_vision_detection_pb2.py`、`ssl_vision_geometry_pb2.py`、`state/ssl_gc_referee_message_pb2.py`、`state/ssl_gc_game_event_pb2.py`、`state/ssl_gc_common_pb2.py`、`geom/ssl_gc_geometry_pb2.py` が含まれること。

- [ ] **Step 5: 仮想環境を作成し依存関係をインストールする**

```bash
cd clients/python
python3 -m venv venv
./venv/bin/pip install -r requirements.txt
```

Run: `./venv/bin/python -c "import google.protobuf; print(google.protobuf.__version__)"`
Expected: エラーなくバージョン番号(例: `7.35.1`)が表示される。

- [ ] **Step 6: `ssl_client/__init__.py`(sys.path + 環境変数のブートストラップ)を書く**

```python
import os
import sys
from pathlib import Path

# protoc 3.12.4 (this repo's system protoc, via apt's protobuf-compiler)
# generates descriptor-based Python code that the protobuf pip package's
# default (upb) backend refuses to load ("Descriptors cannot be created
# directly."). Force the pure-Python implementation instead. This MUST be
# set before any generated _pb2 module is imported anywhere in this package.
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

_PB_DIR = Path(__file__).resolve().parent / "pb"
if str(_PB_DIR) not in sys.path:
    sys.path.insert(0, str(_PB_DIR))
```

- [ ] **Step 7: `ssl_client/net.py` を書く**

```python
import socket
import struct


def open_multicast_socket(group: str, port: int) -> socket.socket:
    """Open a UDP socket bound to `port` and joined to multicast `group`."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", port))
    mreq = struct.pack("4sl", socket.inet_aton(group), socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    return sock
```

- [ ] **Step 8: `pytest.ini` を書く**

```ini
[pytest]
testpaths = tests
```

- [ ] **Step 9: `conftest.py` を書く**

このファイルがないと、`clients/python/` から `pytest`(コンソールスクリプト
のエントリポイント。`python -m pytest` ではない)を実行しても
`clients/python` 自体は `sys.path` に載らない — `clients/python/tests` だけ
が載る(`tests/` に `__init__.py` がないため、pytest のデフォルトの import
モードでは `tests/` 自体がテストモジュールの import ルートとして挿入され
る)。その結果、各テストモジュールの `from ssl_client...` という import は
すべて `ModuleNotFoundError: No module named 'ssl_client'` で失敗してしま
う。`clients/python/` に置いた `conftest.py` はこれを解決する: pytest は
conftest を import するために、その conftest 自身のディレクトリを常に
`sys.path` に挿入し、これはどのテストモジュールが収集されるより前に実行
される — したがって、どのテストファイルのトップレベルの
`import ssl_vision_wrapper_pb2` 的な import が実行されるよりも前に、
`ssl_client/__init__.py`(Step 6)の `PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION`
+ `pb/` のブートストラップが既に実行済みであることも保証される。

`clients/python/conftest.py` を作成:

```python
import ssl_client  # noqa: F401  (runs the pb/ sys.path + env var bootstrap)
```

- [ ] **Step 10: 生成モジュールがパッケージ経由で正しく import できることをスモークテストする**

Run:
```bash
cd clients/python
./venv/bin/python -c "import ssl_client; import grSim_Packet_pb2; from state import ssl_gc_referee_message_pb2; print('imports OK')"
```
Expected: エラーなく `imports OK` と表示される。

- [ ] **Step 11: コミット**

```bash
git add clients/python/requirements.txt clients/python/pytest.ini \
  clients/python/scripts/generate_protos.sh clients/python/ssl_client/__init__.py \
  clients/python/ssl_client/net.py clients/python/.gitignore clients/python/conftest.py
git commit -m "feat(clients/python): scaffold project and protobuf codegen"
```

---

### Task 2: ワールドモデル

**Files:**
- Create: `clients/python/ssl_client/world.py`
- Test: `clients/python/tests/test_world.py`

**Interfaces:**
- Consumes: 生成された `ssl_vision_detection_pb2`、`ssl_vision_geometry_pb2`(Task 1)。
- Produces: `BallObservation`、`RobotObservation`、`FieldGeometry`、`WorldModel` の各 dataclass。`update_from_detection_frame(world: WorldModel, detection: SSL_DetectionFrame) -> None`。`update_from_geometry_data(world: WorldModel, geometry: SSL_GeometryData) -> None`。Task 3(`vision.py`)と Task 6(`strategy.py`)で使用。

- [ ] **Step 1: 失敗するテストを書く**

`clients/python/tests/test_world.py` を作成:

```python
import ssl_vision_detection_pb2
import ssl_vision_geometry_pb2

from ssl_client.world import WorldModel, update_from_detection_frame, update_from_geometry_data


def _make_detection_frame(ball_xy=None, blue=(), yellow=()):
    frame = ssl_vision_detection_pb2.SSL_DetectionFrame()
    frame.frame_number = 1
    frame.t_capture = 123.456
    frame.t_sent = 123.456
    frame.camera_id = 0
    if ball_xy is not None:
        ball = frame.balls.add()
        ball.confidence = 1.0
        ball.x, ball.y = ball_xy
        ball.pixel_x = 0.0
        ball.pixel_y = 0.0
    for robot_id, x, y, orientation in blue:
        robot = frame.robots_blue.add()
        robot.confidence = 1.0
        robot.robot_id = robot_id
        robot.x = x
        robot.y = y
        robot.orientation = orientation
        robot.pixel_x = 0.0
        robot.pixel_y = 0.0
    for robot_id, x, y, orientation in yellow:
        robot = frame.robots_yellow.add()
        robot.confidence = 1.0
        robot.robot_id = robot_id
        robot.x = x
        robot.y = y
        robot.orientation = orientation
        robot.pixel_x = 0.0
        robot.pixel_y = 0.0
    return frame


def test_update_from_detection_frame_converts_mm_to_meters_and_stores_ball():
    world = WorldModel()
    frame = _make_detection_frame(ball_xy=(1000.0, -500.0))

    update_from_detection_frame(world, frame)

    assert world.ball is not None
    assert world.ball.x == 1.0
    assert world.ball.y == -0.5


def test_update_from_detection_frame_stores_robots_by_id_and_color():
    world = WorldModel()
    frame = _make_detection_frame(
        blue=[(0, 2000.0, 0.0, 1.5707963267948966)],
        yellow=[(3, -2000.0, 0.0, 0.0)],
    )

    update_from_detection_frame(world, frame)

    assert 0 in world.blue_robots
    assert world.blue_robots[0].x == 2.0
    assert world.blue_robots[0].orientation == 1.5707963267948966
    assert 3 in world.yellow_robots
    assert world.yellow_robots[3].x == -2.0


def test_update_from_detection_frame_overwrites_previous_observation_for_same_id():
    world = WorldModel()
    update_from_detection_frame(world, _make_detection_frame(blue=[(0, 0.0, 0.0, 0.0)]))
    update_from_detection_frame(world, _make_detection_frame(blue=[(0, 1000.0, 0.0, 0.0)]))

    assert world.blue_robots[0].x == 1.0


def test_update_from_geometry_data_converts_mm_to_meters():
    world = WorldModel()
    geometry = ssl_vision_geometry_pb2.SSL_GeometryData()
    geometry.field.field_length = 9000
    geometry.field.field_width = 6000
    geometry.field.goal_width = 1000
    geometry.field.goal_depth = 180
    geometry.field.boundary_width = 300

    update_from_geometry_data(world, geometry)

    assert world.geometry is not None
    assert world.geometry.field_length == 9.0
    assert world.geometry.field_width == 6.0
    assert world.geometry.goal_width == 1.0
    assert world.geometry.goal_depth == 0.18
    assert world.geometry.boundary_width == 0.3
```

- [ ] **Step 2: テストを実行し、失敗することを確認する**

Run: `cd clients/python && ./venv/bin/pytest tests/test_world.py -v`
Expected: FAIL/ERROR — `ModuleNotFoundError: No module named 'ssl_client.world'`(モジュールがまだ存在しない)。

- [ ] **Step 3: `ssl_client/world.py` を書く**

```python
from dataclasses import dataclass, field
from typing import Dict, Optional

_MM_TO_M = 1.0 / 1000.0


@dataclass
class BallObservation:
    x: float  # meters, field frame
    y: float  # meters, field frame
    t_capture: float


@dataclass
class RobotObservation:
    robot_id: int
    x: float  # meters
    y: float  # meters
    orientation: float  # radians
    t_capture: float


@dataclass
class FieldGeometry:
    field_length: float  # meters
    field_width: float  # meters
    goal_width: float  # meters
    goal_depth: float  # meters
    boundary_width: float  # meters


@dataclass
class WorldModel:
    ball: Optional[BallObservation] = None
    blue_robots: Dict[int, RobotObservation] = field(default_factory=dict)
    yellow_robots: Dict[int, RobotObservation] = field(default_factory=dict)
    geometry: Optional[FieldGeometry] = None


def update_from_detection_frame(world: WorldModel, detection) -> None:
    if detection.balls:
        ball = max(detection.balls, key=lambda b: b.confidence)
        world.ball = BallObservation(
            x=ball.x * _MM_TO_M,
            y=ball.y * _MM_TO_M,
            t_capture=detection.t_capture,
        )

    for robot in detection.robots_blue:
        if not robot.HasField("robot_id"):
            continue
        world.blue_robots[robot.robot_id] = RobotObservation(
            robot_id=robot.robot_id,
            x=robot.x * _MM_TO_M,
            y=robot.y * _MM_TO_M,
            orientation=robot.orientation if robot.HasField("orientation") else 0.0,
            t_capture=detection.t_capture,
        )

    for robot in detection.robots_yellow:
        if not robot.HasField("robot_id"):
            continue
        world.yellow_robots[robot.robot_id] = RobotObservation(
            robot_id=robot.robot_id,
            x=robot.x * _MM_TO_M,
            y=robot.y * _MM_TO_M,
            orientation=robot.orientation if robot.HasField("orientation") else 0.0,
            t_capture=detection.t_capture,
        )


def update_from_geometry_data(world: WorldModel, geometry) -> None:
    f = geometry.field
    world.geometry = FieldGeometry(
        field_length=f.field_length * _MM_TO_M,
        field_width=f.field_width * _MM_TO_M,
        goal_width=f.goal_width * _MM_TO_M,
        goal_depth=f.goal_depth * _MM_TO_M,
        boundary_width=f.boundary_width * _MM_TO_M,
    )
```

- [ ] **Step 4: テストを実行し、成功することを確認する**

Run: `cd clients/python && ./venv/bin/pytest tests/test_world.py -v`
Expected: 4 passed。

- [ ] **Step 5: コミット**

```bash
git add clients/python/ssl_client/world.py clients/python/tests/test_world.py
git commit -m "feat(clients/python): add world model with mm-to-meter conversion"
```

---

### Task 3: Vision 受信

**Files:**
- Create: `clients/python/ssl_client/vision.py`
- Test: `clients/python/tests/test_vision.py`

**Interfaces:**
- Consumes: `WorldModel`、`update_from_detection_frame`、`update_from_geometry_data`(Task 2)。`ssl_client.net.open_multicast_socket`(Task 1)。
- Produces: `parse_wrapper_packet(raw: bytes) -> SSL_WrapperPacket`。`apply_wrapper_packet(world: WorldModel, packet: SSL_WrapperPacket) -> None`。`VisionReceiver(world, group="224.5.23.2", port=10020)`(`.handle_packet(raw: bytes) -> None` と `.run_forever() -> None` を持つ)。Task 7 のエントリポイントで使用。

- [ ] **Step 1: 失敗するテストを書く**

`clients/python/tests/test_vision.py` を作成:

```python
import ssl_vision_wrapper_pb2

from ssl_client.vision import VisionReceiver, apply_wrapper_packet, parse_wrapper_packet
from ssl_client.world import WorldModel


def _make_wrapper_with_ball(x, y):
    wrapper = ssl_vision_wrapper_pb2.SSL_WrapperPacket()
    wrapper.detection.frame_number = 1
    wrapper.detection.t_capture = 1.0
    wrapper.detection.t_sent = 1.0
    wrapper.detection.camera_id = 0
    ball = wrapper.detection.balls.add()
    ball.confidence = 1.0
    ball.x = x
    ball.y = y
    ball.pixel_x = 0.0
    ball.pixel_y = 0.0
    return wrapper


def test_parse_wrapper_packet_round_trips():
    original = _make_wrapper_with_ball(500.0, 250.0)
    data = original.SerializeToString()

    parsed = parse_wrapper_packet(data)

    assert parsed.detection.balls[0].x == 500.0


def test_apply_wrapper_packet_updates_world_ball():
    world = WorldModel()
    wrapper = _make_wrapper_with_ball(500.0, 250.0)

    apply_wrapper_packet(world, wrapper)

    assert world.ball is not None
    assert world.ball.x == 0.5
    assert world.ball.y == 0.25


def test_apply_wrapper_packet_ignores_absent_geometry():
    world = WorldModel()
    wrapper = _make_wrapper_with_ball(0.0, 0.0)

    apply_wrapper_packet(world, wrapper)

    assert world.geometry is None


def test_vision_receiver_handle_packet_updates_its_world():
    world = WorldModel()
    receiver = VisionReceiver(world)
    data = _make_wrapper_with_ball(100.0, 0.0).SerializeToString()

    receiver.handle_packet(data)

    assert world.ball.x == 0.1
```

- [ ] **Step 2: テストを実行し、失敗することを確認する**

Run: `cd clients/python && ./venv/bin/pytest tests/test_vision.py -v`
Expected: FAIL/ERROR — `ModuleNotFoundError: No module named 'ssl_client.vision'`。

- [ ] **Step 3: `ssl_client/vision.py` を書く**

```python
import ssl_vision_wrapper_pb2

from .net import open_multicast_socket
from .world import WorldModel, update_from_detection_frame, update_from_geometry_data


def parse_wrapper_packet(raw: bytes) -> ssl_vision_wrapper_pb2.SSL_WrapperPacket:
    packet = ssl_vision_wrapper_pb2.SSL_WrapperPacket()
    packet.ParseFromString(raw)
    return packet


def apply_wrapper_packet(world: WorldModel, packet: ssl_vision_wrapper_pb2.SSL_WrapperPacket) -> None:
    if packet.HasField("detection"):
        update_from_detection_frame(world, packet.detection)
    if packet.HasField("geometry"):
        update_from_geometry_data(world, packet.geometry)


class VisionReceiver:
    def __init__(self, world: WorldModel, group: str = "224.5.23.2", port: int = 10020):
        self.world = world
        self.group = group
        self.port = port

    def handle_packet(self, raw: bytes) -> None:
        apply_wrapper_packet(self.world, parse_wrapper_packet(raw))

    def run_forever(self) -> None:
        sock = open_multicast_socket(self.group, self.port)
        while True:
            raw, _ = sock.recvfrom(65536)
            self.handle_packet(raw)
```

- [ ] **Step 4: テストを実行し、成功することを確認する**

Run: `cd clients/python && ./venv/bin/pytest tests/test_vision.py -v`
Expected: 4 passed。

- [ ] **Step 5: コミット**

```bash
git add clients/python/ssl_client/vision.py clients/python/tests/test_vision.py
git commit -m "feat(clients/python): add Vision receiver"
```

---

### Task 4: Referee 受信

**Files:**
- Create: `clients/python/ssl_client/referee.py`
- Test: `clients/python/tests/test_referee.py`

**Interfaces:**
- Consumes: `ssl_client.net.open_multicast_socket`(Task 1)。生成された `state.ssl_gc_referee_message_pb2`(Task 1)。
- Produces: `parse_referee_packet(raw: bytes) -> Referee`。`is_match_running(msg: Referee) -> bool`。`RefereeReceiver(group="224.5.23.1", port=10003)`(`.handle_packet(raw: bytes) -> None`、`.is_running() -> bool`、`.run_forever() -> None` を持つ)。Task 7 のエントリポイントで使用。

- [ ] **Step 1: 失敗するテストを書く**

`clients/python/tests/test_referee.py` を作成:

```python
from state import ssl_gc_referee_message_pb2 as referee_pb2

from ssl_client.referee import RefereeReceiver, is_match_running, parse_referee_packet


def _make_referee(command):
    msg = referee_pb2.Referee()
    msg.packet_timestamp = 0
    msg.stage = referee_pb2.Referee.NORMAL_FIRST_HALF
    msg.command = command
    msg.command_counter = 0
    msg.command_timestamp = 0
    for team in (msg.yellow, msg.blue):
        team.name = ""
        team.score = 0
        team.red_cards = 0
        team.yellow_cards = 0
        team.timeouts = 0
        team.timeout_time = 0
        team.goalkeeper = 0
    return msg


def test_is_match_running_false_during_halt_and_stop():
    assert is_match_running(_make_referee(referee_pb2.Referee.HALT)) is False
    assert is_match_running(_make_referee(referee_pb2.Referee.STOP)) is False


def test_is_match_running_true_during_normal_start_and_force_start():
    assert is_match_running(_make_referee(referee_pb2.Referee.NORMAL_START)) is True
    assert is_match_running(_make_referee(referee_pb2.Referee.FORCE_START)) is True


def test_parse_referee_packet_round_trips():
    data = _make_referee(referee_pb2.Referee.FORCE_START).SerializeToString()

    parsed = parse_referee_packet(data)

    assert parsed.command == referee_pb2.Referee.FORCE_START


def test_referee_receiver_defaults_to_running_before_any_packet_seen():
    receiver = RefereeReceiver()
    assert receiver.is_running() is True


def test_referee_receiver_reflects_latest_packet():
    receiver = RefereeReceiver()
    receiver.handle_packet(_make_referee(referee_pb2.Referee.HALT).SerializeToString())
    assert receiver.is_running() is False

    receiver.handle_packet(_make_referee(referee_pb2.Referee.FORCE_START).SerializeToString())
    assert receiver.is_running() is True
```

- [ ] **Step 2: テストを実行し、失敗することを確認する**

Run: `cd clients/python && ./venv/bin/pytest tests/test_referee.py -v`
Expected: FAIL/ERROR — `ModuleNotFoundError: No module named 'ssl_client.referee'`。

- [ ] **Step 3: `ssl_client/referee.py` を書く**

```python
from typing import Optional

from state import ssl_gc_referee_message_pb2 as referee_pb2

from .net import open_multicast_socket

_STOPPED_COMMANDS = (referee_pb2.Referee.HALT, referee_pb2.Referee.STOP)


def parse_referee_packet(raw: bytes) -> referee_pb2.Referee:
    msg = referee_pb2.Referee()
    msg.ParseFromString(raw)
    return msg


def is_match_running(msg: referee_pb2.Referee) -> bool:
    return msg.command not in _STOPPED_COMMANDS


class RefereeReceiver:
    def __init__(self, group: str = "224.5.23.1", port: int = 10003):
        self.group = group
        self.port = port
        self.latest: Optional[referee_pb2.Referee] = None

    def handle_packet(self, raw: bytes) -> None:
        self.latest = parse_referee_packet(raw)

    def is_running(self) -> bool:
        # No referee integration seen yet: assume free play rather than
        # freezing forever if ssl-game-controller isn't running.
        if self.latest is None:
            return True
        return is_match_running(self.latest)

    def run_forever(self) -> None:
        sock = open_multicast_socket(self.group, self.port)
        while True:
            raw, _ = sock.recvfrom(65536)
            self.handle_packet(raw)
```

- [ ] **Step 4: テストを実行し、成功することを確認する**

Run: `cd clients/python && ./venv/bin/pytest tests/test_referee.py -v`
Expected: 5 passed。

- [ ] **Step 5: コミット**

```bash
git add clients/python/ssl_client/referee.py clients/python/tests/test_referee.py
git commit -m "feat(clients/python): add Referee receiver"
```

---

### Task 5: コマンド組み立てと送信

**Files:**
- Create: `clients/python/ssl_client/commands.py`
- Test: `clients/python/tests/test_commands.py`

**Interfaces:**
- Consumes: 生成された `grSim_Packet_pb2`(Task 1)。
- Produces: `RobotCommand` dataclass(`robot_id: int, vel_x: float, vel_y: float, vel_angular: float, robot_orientation: float, kick_speed: float = 0.0, chip_speed: float = 0.0, dribble: bool = False`)。`global_to_local(vel_x, vel_y, orientation) -> tuple[float, float]`。`build_packet(is_team_yellow: bool, commands: list[RobotCommand]) -> bytes`。`CommandSender(host="127.0.0.1", port=20011)`(`.send(is_team_yellow: bool, commands: list[RobotCommand]) -> None` を持つ)。`RobotCommand` と `build_packet` は Task 6(`strategy.py`)と Task 7(エントリポイント)で使用される。

- [ ] **Step 1: 失敗するテストを書く**

`clients/python/tests/test_commands.py` を作成:

```python
import math

import grSim_Packet_pb2

from ssl_client.commands import RobotCommand, build_packet, global_to_local


def test_global_to_local_no_rotation_is_identity():
    tangent, normal = global_to_local(1.0, 2.0, 0.0)
    assert math.isclose(tangent, 1.0)
    assert math.isclose(normal, 2.0)


def test_global_to_local_facing_positive_y_moving_global_x_goes_right():
    # Facing +Y (orientation=pi/2) and moving in global +X is "to the robot's
    # right": zero forward (tangent) motion, negative normal (normal is
    # positive to the left). Matches the rotation used in src/robot.cpp:482-483.
    tangent, normal = global_to_local(1.0, 0.0, math.pi / 2)
    assert math.isclose(tangent, 0.0, abs_tol=1e-9)
    assert math.isclose(normal, -1.0, abs_tol=1e-9)


def test_build_packet_sets_team_color_and_round_trips_all_fields():
    commands = [
        RobotCommand(
            robot_id=2,
            vel_x=1.0,
            vel_y=0.0,
            vel_angular=0.5,
            robot_orientation=0.0,
            kick_speed=3.0,
            chip_speed=0.0,
            dribble=True,
        ),
    ]

    data = build_packet(is_team_yellow=True, commands=commands)

    parsed = grSim_Packet_pb2.grSim_Packet()
    parsed.ParseFromString(data)
    assert parsed.commands.isteamyellow is True
    rc = parsed.commands.robot_commands[0]
    assert rc.id == 2
    assert math.isclose(rc.veltangent, 1.0)
    assert math.isclose(rc.velnormal, 0.0, abs_tol=1e-9)
    assert math.isclose(rc.velangular, 0.5)
    assert math.isclose(rc.kickspeedx, 3.0)
    assert math.isclose(rc.kickspeedz, 0.0)
    assert rc.spinner is True
    assert rc.wheelsspeed is False


def test_build_packet_with_no_commands_is_still_valid():
    data = build_packet(is_team_yellow=False, commands=[])

    parsed = grSim_Packet_pb2.grSim_Packet()
    parsed.ParseFromString(data)
    assert parsed.commands.isteamyellow is False
    assert len(parsed.commands.robot_commands) == 0
```

- [ ] **Step 2: テストを実行し、失敗することを確認する**

Run: `cd clients/python && ./venv/bin/pytest tests/test_commands.py -v`
Expected: FAIL/ERROR — `ModuleNotFoundError: No module named 'ssl_client.commands'`。

- [ ] **Step 3: `ssl_client/commands.py` を書く**

```python
import math
import socket
from dataclasses import dataclass

import grSim_Packet_pb2


@dataclass
class RobotCommand:
    robot_id: int
    vel_x: float  # desired velocity, field (global) frame, m/s
    vel_y: float  # desired velocity, field (global) frame, m/s
    vel_angular: float  # rad/s
    robot_orientation: float  # current measured robot orientation, radians
    kick_speed: float = 0.0  # m/s, straight kick
    chip_speed: float = 0.0  # m/s, chip kick
    dribble: bool = False


def global_to_local(vel_x: float, vel_y: float, orientation: float) -> tuple:
    """Rotate a field-frame velocity into the robot's local frame (tangent=
    forward, normal=left), matching Robot::setSpeed in src/robot.cpp."""
    tangent = vel_x * math.cos(orientation) + vel_y * math.sin(orientation)
    normal = -vel_x * math.sin(orientation) + vel_y * math.cos(orientation)
    return tangent, normal


def build_packet(is_team_yellow: bool, commands: list) -> bytes:
    packet = grSim_Packet_pb2.grSim_Packet()
    packet.commands.timestamp = 0.0
    packet.commands.isteamyellow = is_team_yellow
    for cmd in commands:
        rc = packet.commands.robot_commands.add()
        rc.id = cmd.robot_id
        tangent, normal = global_to_local(cmd.vel_x, cmd.vel_y, cmd.robot_orientation)
        rc.veltangent = tangent
        rc.velnormal = normal
        rc.velangular = cmd.vel_angular
        rc.kickspeedx = cmd.kick_speed
        rc.kickspeedz = cmd.chip_speed
        rc.spinner = cmd.dribble
        rc.wheelsspeed = False
    return packet.SerializeToString()


class CommandSender:
    def __init__(self, host: str = "127.0.0.1", port: int = 20011):
        self.host = host
        self.port = port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

    def send(self, is_team_yellow: bool, commands: list) -> None:
        data = build_packet(is_team_yellow, commands)
        self._sock.sendto(data, (self.host, self.port))
```

- [ ] **Step 4: テストを実行し、成功することを確認する**

Run: `cd clients/python && ./venv/bin/pytest tests/test_commands.py -v`
Expected: 4 passed。

- [ ] **Step 5: コミット**

```bash
git add clients/python/ssl_client/commands.py clients/python/tests/test_commands.py
git commit -m "feat(clients/python): add grSim command builder and UDP sender"
```

---

### Task 6: 戦略(ロール割当てとフォーメーション)

**Files:**
- Create: `clients/python/ssl_client/strategy.py`
- Test: `clients/python/tests/test_strategy.py`

**Interfaces:**
- Consumes: `WorldModel`、`BallObservation`、`RobotObservation`、`FieldGeometry`(Task 2)。`RobotCommand`(Task 5)。
- Produces: `TeamConfig` dataclass(`is_team_yellow: bool, defend_positive_x: bool`)。`TeamStrategy(config: TeamConfig)`(`.compute_commands(world: WorldModel, referee_running: bool) -> list[RobotCommand]` を持つ)。Task 7 のエントリポイントで使用。

- [ ] **Step 1: 失敗するテストを書く**

`clients/python/tests/test_strategy.py` を作成:

```python
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
```

- [ ] **Step 2: テストを実行し、失敗することを確認する**

Run: `cd clients/python && ./venv/bin/pytest tests/test_strategy.py -v`
Expected: FAIL/ERROR — `ModuleNotFoundError: No module named 'ssl_client.strategy'`。

- [ ] **Step 3: `ssl_client/strategy.py` を書く**

```python
import math
from dataclasses import dataclass
from typing import Dict, Optional

from .commands import RobotCommand
from .world import BallObservation, WorldModel

_SEEK_GAIN = 2.0
_MAX_SPEED = 2.0  # m/s, conservative vs. grSim's configured VelAbsoluteMax=5
_KICK_RANGE_M = 0.15
_KICK_SPEED_MPS = 4.0
_SEPARATION_DISTANCE = 0.4  # meters; own robots closer than this get pushed apart
_SEPARATION_GAIN = 1.5
_ANGLE_GAIN = 3.0
_MAX_ANGULAR_SPEED = 4.0  # rad/s, conservative vs. grSim's configured VelAngularMax=20
_KICK_ANGLE_TOLERANCE = 0.35  # radians (~20 degrees)

_FORMATION_PUSH_FORWARD = 0.5  # meters; extra forward shift when the ball is in the attacking half

# (forward_offset, lateral_offset) in meters, relative to the team's own goal
# line, for each non-keeper robot's fallback formation slot (assigned in
# ascending robot-id order, cycling if there are more robots than slots).
_FORMATION_SLOTS = [
    (1.0, 0.0),
    (1.0, 1.2),
    (1.0, -1.2),
    (2.5, 0.8),
    (2.5, -0.8),
]


@dataclass
class TeamConfig:
    is_team_yellow: bool
    defend_positive_x: bool  # True: this team's own goal is on the +X side


def _seek(current_x: float, current_y: float, target_x: float, target_y: float):
    vx = (target_x - current_x) * _SEEK_GAIN
    vy = (target_y - current_y) * _SEEK_GAIN
    speed = math.hypot(vx, vy)
    if speed > _MAX_SPEED:
        scale = _MAX_SPEED / speed
        vx *= scale
        vy *= scale
    return vx, vy


def _own_goal_x(world: WorldModel, defend_positive_x: bool) -> float:
    half_length = world.geometry.field_length / 2.0
    return half_length if defend_positive_x else -half_length


def _goalkeeper_target(world: WorldModel, defend_positive_x: bool):
    half_goal = world.geometry.goal_width / 2.0
    target_y = max(-half_goal, min(half_goal, world.ball.y))
    return _own_goal_x(world, defend_positive_x), target_y


def _formation_target(world: WorldModel, defend_positive_x: bool, slot_index: int, ball_x: float):
    """Fixed formation slot, relative to this team's own goal line, pushed
    _FORMATION_PUSH_FORWARD further forward when the ball is in the
    opponent's half of the field (a fixed, symmetric field split around
    field-center x=0 - not the same axis as the goalkeeper's lateral
    tracking, which follows the ball's Y)."""
    forward_offset, lateral_offset = _FORMATION_SLOTS[slot_index % len(_FORMATION_SLOTS)]
    forward_sign = -1.0 if defend_positive_x else 1.0
    own_goal_x = _own_goal_x(world, defend_positive_x)
    ball_in_attacking_half = (ball_x * forward_sign) > 0.0
    push_forward = _FORMATION_PUSH_FORWARD if ball_in_attacking_half else 0.0
    target_x = own_goal_x + forward_sign * (forward_offset + push_forward)
    return target_x, lateral_offset


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def _face(current_orientation: float, target_orientation: float):
    """Returns (vel_angular, angle_error) to rotate from current_orientation
    toward target_orientation, clamped to _MAX_ANGULAR_SPEED."""
    error = _normalize_angle(target_orientation - current_orientation)
    vel_angular = error * _ANGLE_GAIN
    if abs(vel_angular) > _MAX_ANGULAR_SPEED:
        vel_angular = math.copysign(_MAX_ANGULAR_SPEED, vel_angular)
    return vel_angular, error


def _apply_separation(rid, vx: float, vy: float, robot, own_robots: dict):
    """Push a robot's commanded velocity away from own teammates that are
    closer than _SEPARATION_DISTANCE, to avoid them stacking on top of each
    other. Not real path planning — just a simple repulsion term."""
    for other_id, other in own_robots.items():
        if other_id == rid:
            continue
        dx = robot.x - other.x
        dy = robot.y - other.y
        dist = math.hypot(dx, dy)
        if 0 < dist < _SEPARATION_DISTANCE:
            push = (_SEPARATION_DISTANCE - dist) * _SEPARATION_GAIN
            vx += (dx / dist) * push
            vy += (dy / dist) * push
    speed = math.hypot(vx, vy)
    if speed > _MAX_SPEED:
        scale = _MAX_SPEED / speed
        vx *= scale
        vy *= scale
    return vx, vy


class TeamStrategy:
    def __init__(self, config: TeamConfig):
        self._config = config
        self._goalkeeper_id: Optional[int] = None
        self._formation_slots: Dict[int, int] = {}

    def compute_commands(self, world: WorldModel, referee_running: bool):
        own_robots = world.yellow_robots if self._config.is_team_yellow else world.blue_robots
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
                target_x, target_y = _goalkeeper_target(world, self._config.defend_positive_x)
                vx, vy = _seek(robot.x, robot.y, target_x, target_y)
                vx, vy = _apply_separation(rid, vx, vy, robot, own_robots)
                commands.append(RobotCommand(rid, vx, vy, 0.0, robot.orientation))
            elif rid == attacker_id:
                vx, vy = _seek(robot.x, robot.y, world.ball.x, world.ball.y)
                vx, vy = _apply_separation(rid, vx, vy, robot, own_robots)
                opponent_goal_x = -_own_goal_x(world, self._config.defend_positive_x)
                target_orientation = math.atan2(0.0 - robot.y, opponent_goal_x - robot.x)
                vel_angular, angle_error = _face(robot.orientation, target_orientation)
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
                target_x, target_y = _formation_target(
                    world, self._config.defend_positive_x, slot_index, world.ball.x
                )
                vx, vy = _seek(robot.x, robot.y, target_x, target_y)
                vx, vy = _apply_separation(rid, vx, vy, robot, own_robots)
                commands.append(RobotCommand(rid, vx, vy, 0.0, robot.orientation))

        return commands
```

- [ ] **Step 4: テストを実行し、成功することを確認する**

Run: `cd clients/python && ./venv/bin/pytest tests/test_strategy.py -v`
Expected: 11 passed。

- [ ] **Step 5: コミット**

```bash
git add clients/python/ssl_client/strategy.py clients/python/tests/test_strategy.py
git commit -m "feat(clients/python): add role-assignment and formation strategy"
```

---

### Task 7: エントリポイント(`team_blue.py`、`team_yellow.py`)

**Files:**
- Create: `clients/python/team_blue.py`
- Create: `clients/python/team_yellow.py`

**Interfaces:**
- Consumes: `WorldModel`(Task 2)、`VisionReceiver`(Task 3)、`RefereeReceiver`(Task 4)、`CommandSender`(Task 5)、`TeamConfig`/`TeamStrategy`(Task 6)。
- Produces: 実行可能な2つのスクリプト。これ以上の消費者はいない(依存関係グラフの頂点)。単体テストの対象ではなく、Task 8 で手動確認する。

- [ ] **Step 1: `clients/python/team_blue.py` を書く**

```python
#!/usr/bin/env python3
import argparse
import threading
import time

import ssl_client  # noqa: F401  (runs the pb/ sys.path + env var bootstrap)
from ssl_client.commands import CommandSender
from ssl_client.referee import RefereeReceiver
from ssl_client.strategy import TeamConfig, TeamStrategy
from ssl_client.vision import VisionReceiver
from ssl_client.world import WorldModel

TEAM_IS_YELLOW = False
TEAM_NAME = "blue"


def main():
    parser = argparse.ArgumentParser(description="Simple blue-team SSL client for grSim")
    parser.add_argument(
        "--defend-positive-x",
        action="store_true",
        help="This team's own goal is on the +X side of the field (default: -X side)",
    )
    parser.add_argument("--grsim-host", default="127.0.0.1")
    parser.add_argument("--grsim-port", type=int, default=20011)
    parser.add_argument("--vision-group", default="224.5.23.2")
    parser.add_argument("--vision-port", type=int, default=10020)
    parser.add_argument("--referee-group", default="224.5.23.1")
    parser.add_argument("--referee-port", type=int, default=10003)
    parser.add_argument("--tick-hz", type=float, default=30.0)
    args = parser.parse_args()

    world = WorldModel()
    vision = VisionReceiver(world, group=args.vision_group, port=args.vision_port)
    referee = RefereeReceiver(group=args.referee_group, port=args.referee_port)
    sender = CommandSender(host=args.grsim_host, port=args.grsim_port)
    strategy = TeamStrategy(
        TeamConfig(is_team_yellow=TEAM_IS_YELLOW, defend_positive_x=args.defend_positive_x)
    )

    threading.Thread(target=vision.run_forever, daemon=True).start()
    threading.Thread(target=referee.run_forever, daemon=True).start()

    period = 1.0 / args.tick_hz
    print(f"[{TEAM_NAME}] sending commands to {args.grsim_host}:{args.grsim_port}", flush=True)
    try:
        while True:
            commands = strategy.compute_commands(world, referee.is_running())
            if commands:
                sender.send(TEAM_IS_YELLOW, commands)
            time.sleep(period)
    except KeyboardInterrupt:
        print(f"[{TEAM_NAME}] stopped", flush=True)


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: `clients/python/team_yellow.py` を書く**

```python
#!/usr/bin/env python3
import argparse
import threading
import time

import ssl_client  # noqa: F401  (runs the pb/ sys.path + env var bootstrap)
from ssl_client.commands import CommandSender
from ssl_client.referee import RefereeReceiver
from ssl_client.strategy import TeamConfig, TeamStrategy
from ssl_client.vision import VisionReceiver
from ssl_client.world import WorldModel

TEAM_IS_YELLOW = True
TEAM_NAME = "yellow"


def main():
    parser = argparse.ArgumentParser(description="Simple yellow-team SSL client for grSim")
    parser.add_argument(
        "--defend-negative-x",
        action="store_true",
        help="This team's own goal is on the -X side of the field (default: +X side)",
    )
    parser.add_argument("--grsim-host", default="127.0.0.1")
    parser.add_argument("--grsim-port", type=int, default=20011)
    parser.add_argument("--vision-group", default="224.5.23.2")
    parser.add_argument("--vision-port", type=int, default=10020)
    parser.add_argument("--referee-group", default="224.5.23.1")
    parser.add_argument("--referee-port", type=int, default=10003)
    parser.add_argument("--tick-hz", type=float, default=30.0)
    args = parser.parse_args()

    world = WorldModel()
    vision = VisionReceiver(world, group=args.vision_group, port=args.vision_port)
    referee = RefereeReceiver(group=args.referee_group, port=args.referee_port)
    sender = CommandSender(host=args.grsim_host, port=args.grsim_port)
    strategy = TeamStrategy(
        TeamConfig(is_team_yellow=TEAM_IS_YELLOW, defend_positive_x=not args.defend_negative_x)
    )

    threading.Thread(target=vision.run_forever, daemon=True).start()
    threading.Thread(target=referee.run_forever, daemon=True).start()

    period = 1.0 / args.tick_hz
    print(f"[{TEAM_NAME}] sending commands to {args.grsim_host}:{args.grsim_port}", flush=True)
    try:
        while True:
            commands = strategy.compute_commands(world, referee.is_running())
            if commands:
                sender.send(TEAM_IS_YELLOW, commands)
            time.sleep(period)
    except KeyboardInterrupt:
        print(f"[{TEAM_NAME}] stopped", flush=True)


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: 両エントリポイントが正常に起動・終了することをスモークテストする(grSim はまだ起動していなくてよい)**

Run:
```bash
cd clients/python
timeout 2 ./venv/bin/python team_blue.py || true
timeout 2 ./venv/bin/python team_yellow.py || true
```
Expected: それぞれ `[blue] sending commands to 127.0.0.1:20011` / `[yellow] ...` の行を出力し、`timeout` ラッパーによって約2秒後に終了する。トレースバックは出ないこと(vision/referee ポートを誰も listen していない場合にソケットエラーが見えるのは問題ない — `daemon=True` の受信スレッドは単に何も受信しないだけ。`ModuleNotFoundError`、`AttributeError` などの起動クラッシュがあってはならない)。

- [ ] **Step 4: コミット**

```bash
git add clients/python/team_blue.py clients/python/team_yellow.py
git commit -m "feat(clients/python): add team_blue and team_yellow entrypoints"
```

---

### Task 8: セットアップと手動検証の README

**Files:**
- Create: `clients/python/README.md`

**Interfaces:**
- Consumes: なし(ドキュメントのみ)。
- Produces: 他のタスクからは消費されない。これが最後のタスク。

- [ ] **Step 1: `clients/python/README.md` を書く**

```markdown
# grSim Python match clients

Two independent example clients (`team_blue.py`, `team_yellow.py`) that play a
simplified match against grSim: an attacker chases the ball, a goalkeeper
holds the goal line, the rest hold a loose formation, and everyone freezes on
HALT/STOP from the referee. See
`docs/superpowers/specs/2026-07-14-ssl-match-clients-design.md` for the full
design.

## One-time setup (inside WSL2)

```bash
cd clients/python
./scripts/generate_protos.sh   # requires `protoc` (apt: protobuf-compiler) and git
python3 -m venv venv
./venv/bin/pip install -r requirements.txt
```

Re-run `./scripts/generate_protos.sh` any time you need to regenerate the
`ssl_client/pb/` directory (e.g. after deleting it) — it is git-ignored.

## Running a match

You need three things running at once, each in its own terminal:

1. **grSim** — already built per the top-level `INSTALL.md`:
   ```bash
   cd ~/grSim/bin && ./grSim
   ```

2. **ssl-game-controller** — download the release binary for your platform
   from https://github.com/RoboCup-SSL/ssl-game-controller/releases, then:
   ```bash
   chmod +x ssl-game-controller
   ./ssl-game-controller
   ```
   Open http://localhost:8081 in a browser — this is where you drive the
   match state (Force Start, Stop, Halt).

3. **Both team clients**, from `clients/python/`:
   ```bash
   ./venv/bin/python team_blue.py
   ./venv/bin/python team_yellow.py
   ```

## Verifying it works

1. With all three running, open the ssl-game-controller web UI and click
   **Force Start**. Both teams' robots should start moving: one robot per
   team drives toward the ball, one stays on its own goal line and tracks the
   ball's sideways position, and the rest hold loose positions in front of
   their own goal.
2. Click **Stop** (or **Halt**) in the web UI. All robots on both teams
   should immediately stop moving.
3. Click **Force Start** again to resume.

If robots don't move at all, check:
- grSim's own message log (bottom panel) shows `Command listen port bound
  on: 20011` — if not, grSim isn't listening yet.
- The team script printed `sending commands to 127.0.0.1:20011` — if not, it
  crashed on startup; re-check the setup steps above.
- `PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python` is set (it's set
  automatically by `ssl_client/__init__.py`, but if you import a generated
  `_pb2` module *before* `import ssl_client` anywhere, you'll see `TypeError:
  Descriptors cannot be created directly.` instead).

## Running the unit tests

```bash
cd clients/python
./venv/bin/pytest -v
```

This exercises the pure logic (protobuf parsing, world-model updates, packet
building, strategy role assignment). It does not require grSim or
ssl-game-controller to be running.
```

(このステップで書いた `README.md` は、その後 Task 5(戦術的パスワーク)
のマージに伴って書き直され、現在は日本語版になっている — 上記は Task 8
実行時点の内容の記録。)

- [ ] **Step 2: テストスイート全体をもう一度実行し、すべて一緒にパスすることを確認する**

Run: `cd clients/python && ./venv/bin/pytest -v`
Expected: 28 passed(test_world.py の 4 + test_vision.py の 4 + test_referee.py の 5 + test_commands.py の 4 + test_strategy.py の 11)。

- [ ] **Step 3: コミット**

```bash
git add clients/python/README.md
git commit -m "docs(clients/python): add setup and manual verification instructions"
```
