# 審判状態への完全準拠 — 設計書

日付: 2026-07-21
ステータス: 承認済み(計画フェーズへ)

## 目的

`clients/python/` の対戦クライアントを、ssl-game-controller が発行する主要な審判状態すべてに準拠させる。現状は `HALT`/`STOP` の二値判定([referee.py](../../../clients/python/ssl_client/referee.py) の `is_running()`)のみで、キックオフ・フリーキック・PK・ボールプレースメントの配置ルールに未対応のため、公式ルール運用の試合では反則で頻繁に停止する。本フェーズ完了後は、ssl-game-controller で通常の試合進行(キックオフ→通常プレー→各種セットプレー)が反則なく成立する。

これは「フル機能のサッカーAIシステム」化ロードマップの第1フェーズ(全6フェーズ中)。

## スコープ決定事項

- **BALL_PLACEMENT**: 退避のみ。自チーム指名時も能動的なボール運搬は実装しない(実SSLでも能力申告制であり未対応チームは許容される)。
- **PK (PREPARE_PENALTY)**: 対応する。配置ルールの一種として実装。
- **アーキテクチャ**: GameState層+制約フィルタの二段構え(下記)。

## 対応する審判状態と適用ルール

| Refereeコマンド | 自チームの挙動 |
|---|---|
| `HALT` | 全ロボット即時停止(現状どおり) |
| `STOP` | 速度上限1.5m/s、ボールから0.5m退避、フォーメーション位置へ移動 |
| `PREPARE_KICKOFF_(自)` | 全員自陣ハーフへ。キッカー1台をボール脇(自陣側)に配置 |
| `PREPARE_KICKOFF_(敵)` | 全員自陣ハーフ、センターサークル外0.5m維持 |
| `NORMAL_START` | 直前のPREPARE状態に応じてキッカーが蹴る→以降通常プレー |
| `DIRECT_FREE_(自)` / `INDIRECT_FREE_(自)` | キッカー1台がボールへ、他は通常フォーメーション。キッカーは蹴った後、他ロボットが触るまで再接触しない(2度触り禁止) |
| `DIRECT_FREE_(敵)` / `INDIRECT_FREE_(敵)` | 全員ボールから0.5m以上退避しつつ守備配置 |
| `PREPARE_PENALTY_(自)` | キッカー1台がボール脇、他はボール後方1mへ退避。NORMAL_STARTでシュート |
| `PREPARE_PENALTY_(敵)` | キーパーはゴールライン上、他はボール後方1mへ退避 |
| `BALL_PLACEMENT_(敵)` | ボール→指定地点(designated_position)の線分から0.5m以上退避 |
| `BALL_PLACEMENT_(自)` | 退避のみ(能動配置しない) |
| `TIMEOUT_*` | STOP相当(停止) |
| `FORCE_START` | 通常プレー(現状どおり) |
| 未知・未対応コマンド | 安全側に倒してSTOP相当 |

**常時適用ルール**(審判状態に関係なく):

- キーパー以外は自陣ディフェンスエリアに進入しない
- 全ロボットは敵ディフェンスエリアに進入しない
- フィールド境界外へ出ない

## アーキテクチャ

検討した代替案: (B) 状態別Strategyクラス分離 — 共通処理の重複と大規模な作り直しが発生、制約の強制適用点が分散する。(C) 既存 `compute_commands` へのif分岐追加 — strategy.py が肥大化しテスト困難、違反見落としリスクが高い。いずれも不採用。

採用案は「戦略層はルールに沿った目標を選ぶ」「フィルタ層はそれでも違反するコマンドを強制修正する」の二段構え。ルール違反が構造的に起きにくい。

```
RefereeReceiver ──最新Referee msg──▶ game_state.derive(msg, our_color, prev)
                                        │ GameState
VisionReceiver ──▶ WorldModel ──▶ TeamStrategy.compute_commands(world, game_state)
                                        │ commands(状態別目標位置ベース)
                                  rules.apply_rule_constraints(...)  ← 安全網
                                        │ 制約適用済みcommands
                                  CommandSender
```

## コンポーネント

- **`ssl_client/game_state.py`(新規)** — Refereeメッセージ+自チーム色を入力に、チーム視点の `GameState` を導出する純粋関数群。
  - `GameState` dataclass: `phase`(enum: HALT / STOP / RUNNING / KICKOFF_OURS / KICKOFF_THEIRS / FREE_KICK_OURS / FREE_KICK_THEIRS / PENALTY_OURS / PENALTY_THEIRS / BALL_PLACEMENT_OURS / BALL_PLACEMENT_THEIRS)、`designated_position`(プレースメント指定地点、無ければNone)。
  - `NORMAL_START` は直前のPREPARE状態を引き継ぐ必要があるため(メッセージ自体はチーム情報を持たない)、直前コマンドを保持する小さなトラッカーを含む。キックオフ/PK/フリーキックの「キック実行後は通常プレーへ遷移」の判定(ボールが所定距離以上動いたら遷移)もここが持つ。
- **`ssl_client/rules.py`(新規)** — 制約フィルタ。`apply_rule_constraints(commands, game_state, world, config) -> commands` の純粋関数。
  - 速度クランプ(1.5m/s上限。適用状態: STOP / TIMEOUT / BALL_PLACEMENT_*)
  - ボール退避(退避が必要な状態で0.5m以内に居る場合、退避方向の速度に上書き)
  - プレースメント線分からの退避
  - ディフェンスエリア・場外へ向かうコマンドの抑止(常時)
- **`ssl_client/strategy.py`(拡張)** — シグネチャを `compute_commands(world, game_state)` に変更(現在の `referee_running: bool` を置換)。状態別の目標位置選択(キックオフ配置、フリーキック退避配置、PK配置)を追加。通常プレー(RUNNING)のロジック(CHASE/POSSESS/PASS_IN_FLIGHT状態機械)は現状維持。フリーキックの2度触り禁止はstrategy側で「直近キッカーIDの記憶」により実現。
- **`ssl_client/world.py`(小拡張)** — `FieldGeometry` に `penalty_area_depth` / `penalty_area_width` を追加。geometryパケットに該当フィールドが無い場合はDivision B標準値(depth 1.0m × width 2.0m)にフォールバック。
- **`ssl_client/referee.py`(小拡張)** — `is_running()` の二値APIを廃止し、最新Refereeメッセージをそのまま公開する。解釈は game_state.py に一元化。
- **`team_blue.py` / `team_yellow.py`(小修正)** — 配線変更のみ(RefereeReceiver → game_state → strategy → rules → sender)。

## エラー処理

- Refereeメッセージ未受信: フリープレー扱い(現状どおり。game-controllerなしでも動作する)
- 未知のRefereeコマンド: STOP相当(安全側)
- geometry未受信: コマンド送信しない(現状どおり)
- designated_position欠落時のBALL_PLACEMENT: STOP相当として扱う

## テスト方針

- **TDD(pytest)**: `game_state.py`(コマンド→状態導出、NORMAL_STARTの引き継ぎ、キック後遷移)と `rules.py`(速度クランプ、ボール退避、エリア進入禁止、プレースメント線分退避)は純粋関数として全パスを先行テスト。strategyの状態別配置はモック観測値でテスト。既存の `tests/` 7ファイル体制を踏襲。
- **E2E(手動)**: ssl-game-controller + grSim で全状態遷移を確認: Halt→Stop→キックオフ(両側)→通常プレー→フリーキック(両側)→PK(両側)→ボールプレースメント(両側)。実装完了後に検証手順書を提示する。

## スコープ外

- 能動的なボールプレースメント(ドリブル運搬)
- 本格的な経路計画(フェーズ2)、Visionフィルタリング(フェーズ3)、動的役割割当(フェーズ4)、制御品質改善(フェーズ5)、可視化(フェーズ6)
- INDIRECT_FREE と DIRECT_FREE の挙動区別(どちらも同じ配置ルールとして扱う。得点判定は審判側の責務)
