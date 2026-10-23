#!/bin/bash
# Build the Scout images and tag them <date>-<git commit>[-dirty] plus :latest (old tags are kept for rollback:
# TAG=<old tag> ~/robot_up.sh).
#   docker/build.sh [safety|base|perception ...]     (default: all three)
# First build: perception ~1.5-2 h (librealsense + torchvision from source), base ~30 min, safety ~5 min.
set -euo pipefail
cd "$(dirname "$0")"
rev=$(git -C .. rev-parse --short HEAD 2>/dev/null || echo nogit)
[ -n "$(git -C .. status --porcelain 2>/dev/null)" ] && rev=$rev-dirty
export TAG=$(date +%Y%m%d)-$rev
./sync_third_party.sh >/dev/null    # images must match the third-party commits/patches on disk
if [ -n "$(git -C .. status --porcelain docker/third_party.repos docker/patches 2>/dev/null)" ]; then
  echo "note: docker/third_party.repos or patches changed; commit them so $TAG is reproducible"
fi
services=${*:-safety base perception}
echo "building $services as :$TAG"
docker compose -f compose.yaml build $services
for s in $services; do docker tag scout-$s:$TAG scout-$s:latest; done
docker images --format '{{.Repository}}:{{.Tag}}  {{.Size}}' | grep "^scout-" | grep -E ":$TAG|:latest"
