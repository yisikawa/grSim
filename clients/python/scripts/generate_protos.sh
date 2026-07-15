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
