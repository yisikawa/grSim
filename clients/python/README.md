# grSim Python 対戦クライアント

grSim と対戦する2つの独立したサンプルクライアント(`team_blue.py`、
`team_yellow.py`)。ボールを持ったロボットが最も評価の高い選択肢(パス/
シュート/ドリブル)を判断し、味方へパスを回しながらポゼッションを保つ
「戦術的パスワーク」を行う。パス飛行中は指定された受け手がボールの軌道
に入ってトラップし、ボールを持たない味方はパスコースが開ける位置へ動き
直す。ゴールキーパーはゴールライン上に留まる。

審判(ssl-game-controller)対応は単純な HALT/STOP の二値判定ではなく、
`Referee` メッセージをチーム視点の状態(`GameState`)に変換し、状態に応
じた目標位置を戦略層が選び、それでも規則に反するコマンドをルール制約
フィルタが強制修正する二段構えになっている。キックオフ・フリーキック・
PK・ボールプレースメントなど主要な審判状態に準拠する(詳細は下記の
「対応する審判状態」および設計書を参照)。設計の詳細は
`docs/superpowers/specs/2026-07-14-ssl-match-clients-design.md` と
`docs/superpowers/specs/2026-07-15-passing-strategy-design.md`、
`docs/superpowers/specs/2026-07-21-referee-compliance-design.md` を参照。

## 初回セットアップ(WSL2 内で実行)

```bash
cd clients/python
./scripts/generate_protos.sh   # protoc(apt: protobuf-compiler)と git が必要
python3 -m venv venv
./venv/bin/pip install -r requirements.txt
```

`ssl_client/pb/` ディレクトリ(git 管理外)を再生成したい場合は、いつでも
`./scripts/generate_protos.sh` を再実行すればよい。

## 対戦の起動

以下の3つを、それぞれ別のターミナルで同時に起動する必要がある:

1. **grSim** — トップレベルの `INSTALL.md` の手順でビルド済みのもの:
   ```bash
   cd ~/grSim/bin && ./grSim
   ```

2. **ssl-game-controller** — 使用環境向けのリリースバイナリを
   https://github.com/RoboCup-SSL/ssl-game-controller/releases から
   ダウンロードし、以下を実行:
   ```bash
   chmod +x ssl-game-controller
   ./ssl-game-controller
   ```
   ブラウザで http://localhost:8081 を開く — ここで試合状態
   (Force Start、Stop、Halt)を操作する。

3. **両チームのクライアント**、`clients/python/` から:
   ```bash
   ./venv/bin/python team_blue.py
   ./venv/bin/python team_yellow.py
   ```

## 動作確認方法

1. 3つすべてが起動した状態で、ssl-game-controller の Web UI を開き
   **Force Start** をクリックする。両チームともポゼッション重視のパス
   ワークを行うようになる: 最初にボールへ到達したロボットが保持し
   (ドリブラー ON)、最も評価の高い選択肢の方を向いて、空いている味方へ
   パスする — シュートはゴールに近く、シュートコースが開いている場合の
   み行う。パスが飛んでいる間、指定された受け手がボールの軌道に入って
   トラップする。ボールを持たない味方は、ボールからのパスコースが開け
   る位置へフォーメーションスロット周辺で位置を調整する。
2. Web UI で **Halt** をクリックすると両チームの全ロボットが即座に停止
   する。**Stop** の場合は即停止ではなく、全ロボットがボールから0.5m
   以上離れつつ、低速(1.5m/s以下)でフォーメーション位置へ移動する。
3. 再度 **Force Start** をクリックして再開する。

ロボットが全く動かない場合は、以下を確認する:
- grSim 自身のメッセージログ(下部パネル)に `Command listen port bound
  on: 20011` と表示されているか — 表示されていなければ grSim がまだ
  待ち受けていない。
- チームスクリプトが `sending commands to 127.0.0.1:20011` と出力して
  いるか — 出力されていなければ起動時にクラッシュしている。上記のセ
  ットアップ手順を再確認する。
- `PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python` が設定されているか
  (`ssl_client/__init__.py` によって自動的に設定されるが、生成された
  `_pb2` モジュールを `import ssl_client` より前にどこかで import する
  と、代わりに `TypeError: Descriptors cannot be created directly.` が
  発生する)。

## 審判状態への対応

`ssl_client/referee.py` は最新の `Referee` メッセージをそのまま公開する
だけで、解釈は行わない。解釈と制約適用は次の2モジュールが担う:

- **`ssl_client/game_state.py`** — `Referee` メッセージ + 自チーム色から、
  チーム視点の `GameState`(`Phase` enum と、プレースメント指定地点など)
  を導出する純粋関数群。`NORMAL_START` が直前の `PREPARE_*` 状態を引き継
  ぐ処理や、キックオフ/PK/フリーキックで「ボールが動いたら通常プレーへ
  遷移する」判定もここに集約されている。
- **`ssl_client/rules.py`** — `apply_rule_constraints(commands, game_state,
  own_robots=..., ball=..., geometry=..., defend_positive_x=..., keeper_id=...,
  exempt_ids=...)` という制約フィルタ。戦略層が選んだ目標位置が結果的に
  規則違反(速度超過・ボールへの接近しすぎ・プレースメント線分侵入・
  ディフェンスエリアや場外への進入)になっていた場合、これを強制的に
  修正する安全網。

戦略層(`ssl_client/strategy.py`)は `GameState` を見て状態別の目標位置
(キックオフ配置、フリーキック退避、PK配置など)を選び、その出力を
`rules.py` のフィルタに通してから送信する、という二段構えになっている。

### 対応する審判状態(概要)

| Refereeコマンド | 自チームの挙動 |
|---|---|
| `HALT` | 全ロボット即時停止 |
| `STOP` | 速度上限1.5m/s、ボールから0.5m退避、フォーメーション位置へ移動 |
| `PREPARE_KICKOFF_*` | 自陣ハーフへ整列。自チームのキックオフならキッカー1台をボール脇に配置(蹴った後は他ロボットが触るまで再接触しない=2度触り禁止)、敵のキックオフならセンターサークル外0.5mを維持 |
| `NORMAL_START` | 直前のPREPARE状態を引き継ぎ、キッカーが蹴って通常プレーへ移行 |
| `DIRECT_FREE_*` / `INDIRECT_FREE_*` | 自チームならキッカー1台がボールへ(2度触り禁止)、敵チームなら全員0.5m退避 |
| `PREPARE_PENALTY_*` | 自チームのPKならキッカーがボール脇・他はボール後方1mへ退避、敵のPKならキーパーはゴールライン上・他は後方1mへ退避。NORMAL_STARTで実行 |
| `BALL_PLACEMENT_*` | ボール→指定地点の線分から0.5m以上退避(能動配置は未対応) |
| `TIMEOUT_*` | STOP相当 |
| `FORCE_START` | 通常プレー |
| 未知・未対応コマンド | 安全側に倒してSTOP相当 |

**常時適用されるルール**(審判状態によらず): キーパー以外は自陣ディフェ
ンスエリアに進入しない、全ロボットは敵ディフェンスエリアに進入しない、
フィールド境界外へ出ない。

詳細な設計は
`docs/superpowers/specs/2026-07-21-referee-compliance-design.md` を参照。
grSim + ssl-game-controller を使った手動E2E検証手順は
[`docs/referee-compliance-verification.md`](docs/referee-compliance-verification.md)
を参照。

## 単体テストの実行

```bash
cd clients/python
./venv/bin/pytest -v
```

純粋なロジック(protobuf のパース、ワールドモデルの更新、パケット組み
立て、戦略のロール割当て、GameState導出、ルール制約フィルタ)を検証す
る。grSim や ssl-game-controller が起動している必要はない。手動での
E2E検証手順は
[`docs/referee-compliance-verification.md`](docs/referee-compliance-verification.md)
を参照。
