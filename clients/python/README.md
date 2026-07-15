# grSim Python 対戦クライアント

grSim と対戦する2つの独立したサンプルクライアント(`team_blue.py`、
`team_yellow.py`)。ボールを持ったロボットが最も評価の高い選択肢(パス/
シュート/ドリブル)を判断し、味方へパスを回しながらポゼッションを保つ
「戦術的パスワーク」を行う。パス飛行中は指定された受け手がボールの軌道
に入ってトラップし、ボールを持たない味方はパスコースが開ける位置へ動き
直す。ゴールキーパーはゴールライン上に留まり、審判からの HALT/STOP で
全ロボットが停止する。設計の詳細は
`docs/superpowers/specs/2026-07-14-ssl-match-clients-design.md` と
`docs/superpowers/specs/2026-07-15-passing-strategy-design.md` を参照。

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
2. Web UI で **Stop**(または **Halt**)をクリックする。両チームの全
   ロボットが即座に停止するはずである。
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

## 単体テストの実行

```bash
cd clients/python
./venv/bin/pytest -v
```

純粋なロジック(protobuf のパース、ワールドモデルの更新、パケット組み
立て、戦略のロール割当て)を検証する。grSim や ssl-game-controller が
起動している必要はない。
