# SSL対戦クライアント（Python）— 設計書

日付: 2026-07-14
ステータス: 計画フェーズへ承認済み

## 目的

grSimに対して「最小限だが実際に試合をする」2チーム対戦を構築する。競技レベルのAIを作ることが目的ではなく、SSLの一連のアーキテクチャ（Vision受信 → 戦略判断 → Command送信 → 審判状態）を一通り体験・把握することが目的。稼働中のgrSimと公式`ssl-game-controller`に対して、チームごとに独立した2つのPythonクライアントを接続し、「ロボットがボールへ向かう」「ゴールキーパーがゴールラインを守る」「`HALT`/`STOP`で全チームが停止する」といった、見た目にまとまりのあるプレーを実現する。

## アーキテクチャ

```
[grSim] (既存・起動済み)
   │ Vision: multicast 224.5.23.2:10020 (ssl_vision_wrapper.proto)
   │ Command: unicast 127.0.0.1:20011 (grSim_Packet.proto, isteamyellowでチーム区別)
   │
[ssl-game-controller] (新規・公式リリースバイナリ — ビルド不要)
   │ Referee: multicast 224.5.23.1:10003 (SSL_Referee message)
   │ Web UI: localhost:8081 (人間がForce Start / Halt / Stopを操作)
   │
   ├── team_blue.py   ── Vision + Referee → 戦略判断 → Command送信 (Blue)
   └── team_yellow.py ── Vision + Referee → 戦略判断 → Command送信 (Yellow)
```

`team_blue.py`と`team_yellow.py`は互いに直接通信しない独立したOSプロセスであり、共有のVision/Refereeマルチキャスト配信だけを見て、それぞれ独自に判断する。これは実際のSSLチーム同士の関係と同じ構造。

## 配置場所

- コード: 本grSimリポジトリ内の`clients/python/`（既存の`clients/qt`, `clients/java`サンプルと並ぶ位置）
- 実行: WSL2から、Windows側リポジトリのパス（`/mnt/d/LLMprojects/grSim/clients/python`としてマウントされている）に対して直接実行する。WSLネイティブのファイルシステム下へコピーする必要はない。Pythonスクリプトには、grSim本体のビルドを遅くしていたような大量の小ファイルI/Oが発生しないため。

## コンポーネント（共有パッケージ + 2つのエントリポイント）

- `ssl_client/vision.py` — Visionマルチキャストグループに参加し、`ssl_vision_wrapper.proto`をデコードして、ワールドモデル（ボール位置、両チーム全ロボットの位置。ロボットIDごとに最新フレームを採用）を保持する。
- `ssl_client/referee.py` — Refereeマルチキャストグループに参加し、`SSL_Referee`をデコードする。本リポジトリには同梱されていない`ssl_gc_referee_message.proto`をもとに、実装時に`RoboCup-SSL/ssl-game-controller`（または`ssl-vision`）リポジトリからPython用protobufバインディングを生成する必要がある。
- `ssl_client/commands.py` — 本リポジトリ既存の`grSim_Commands.proto`/`grSim_Packet.proto`を再利用して`grSim_Packet`を組み立て、grSimのコマンドポート(20011)へ送信する。1ティックにつき1パケット、制御下の各ロボット分の`robot_command`を含める。
- `ssl_client/strategy.py` — 役割分担と各ロボットの目標速度計算（下記参照）。
- `team_blue.py` / `team_yellow.py` — 上記を組み合わせる薄いエントリポイント。違いはどちらのチームカラーを担当するかのみ。

## 戦略ロジック（フォーメーション + 役割分担）

毎ティック、自チームのロボットに対して:
- ボールに一番近いロボットを**アタッカー**に割り当てる：ボールに向かって移動し、キック可能な距離内かつおおむね相手ゴールを向いていればキック(`kickspeedx`)する。
- 1台を**ゴールキーパー**として固定的に割り当てる：自陣ゴールライン上に留まり、ボールの横方向(Y座標)をゴール幅の範囲内で追従する。
- 残りのロボットは単純な固定**フォーメーション位置**（例: ディフェンダー2、サポートアタッカー2 — フィールド中央からのオフセットとして定義）を保持し、ボールがある側のハーフへ少し寄せる。
- 簡易的な分離処理：ロボット同士が一定距離より近づいたら反発力を加える。本格的な経路計画ではない。

## 審判状態の扱い（簡略化）

- `HALT`または`STOP`: 自チームの全ロボットに毎ティック速度ゼロを送信する。
- それ以外の状態(`NORMAL_START`、`RUNNING`、`FORCE_START`等): 上記の戦略ロジックを無条件に実行する。
- 細かい状態(`PREPARE_KICKOFF`、ボールプレースメント、フリーキックの配置ルール等)には対応しない — 状態遷移はssl-game-controllerのWeb UIから人間が操作する。

## スコープ外

- 本格的な経路計画・衝突回避のない移動
- パス回しの判断、シュート力の調整、ドリブル判断
- 全審判状態・配置ルールへの完全準拠
- 2つのチームプロセス間の通信

## テスト・検証方法

自動テストは計画せず、手動・対話的な検証を行う（探索的なプロジェクトのため）:
1. grSimを起動する（既存のセットアップ通り、動作確認済み）。
2. `ssl-game-controller`のリリースバイナリを起動し、Web UI(`localhost:8081`)を開く。
3. 別々のWSLターミナルで`team_blue.py`と`team_yellow.py`を起動する。
4. Web UIから`Force Start`を実行し、両チームのロボットがボールへ向かう、ゴールキーパーがゴールラインに沿ってボールを追従する、フォーメーション位置が保たれることを目視確認する。
5. Web UIから`Stop`/`Halt`を実行し、全ロボットが停止することを目視確認する。

## 実装時に解決が必要な依存関係

- `ssl_gc_referee_message.proto`（およびそれがimportするファイル）は`RoboCup-SSL/ssl-game-controller`のGitHubリポジトリから取得し、本リポジトリ既存の`grSim_Commands.proto` / `grSim_Packet.proto` / `ssl_vision_wrapper.proto` / `ssl_vision_detection.proto`と合わせて`protoc --python_out=...`でコンパイルする必要がある。
