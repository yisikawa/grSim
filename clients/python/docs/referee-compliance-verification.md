# 審判準拠 手動E2E検証手順

前提: grSim起動済み(Vision 224.5.23.2:10020 / Command 20011)、
ssl-game-controller起動済み(Referee 224.5.23.1:10003, Web UI http://localhost:8081)。
WSL2の2ターミナルで `python3 team_yellow.py` と `python3 team_blue.py` を起動
(デフォルトで互いに反対側のゴールを守備する。yellow=+X側、blue=-X側)。

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
- [ ] **game-controllerなしで起動**: game-controllerを一度も起動していない状態でクライアントを起動すると、フリープレーで動作する(いったんRefereeメッセージを受信した後は、GCを停止しても最後の状態が維持される)
