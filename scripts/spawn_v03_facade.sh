#!/usr/bin/env bash

set -e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

WORLD="${1:-default}"
SDF="$REPO_ROOT/models/skyscrub_facade/model.sdf"

echo "Spawning SkyScrub V0.3 test facade..."
echo "World : $WORLD"
echo "SDF   : $SDF"

gz service \
  -s "/world/${WORLD}/create" \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req "sdf_filename: \"$SDF\", name: \"skyscrub_facade\""
