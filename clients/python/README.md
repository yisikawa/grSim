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
   **Force Start**. Both teams now play a possession-first passing game:
   the robot that reaches the ball first takes possession (dribbler on),
   turns toward the best-scoring option, and passes to an open teammate —
   shooting only when close to the goal with an open shot lane. While a
   pass is in flight, the designated receiver moves onto the ball's path
   to trap it. Robots without the ball shift around their formation slots
   to keep an open passing lane from the ball.
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
