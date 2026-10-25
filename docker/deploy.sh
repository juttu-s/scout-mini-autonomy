#!/bin/bash
# Deploy a CI release (git tag v*, images built by .github/workflows/ci.yml and pushed to GHCR) onto the robot.
#   docker/deploy.sh <tag> [--skip-scenarios] [--nav <map> [x y yaw]]
# One-time: docker login ghcr.io -u <github user>   (password: a token with read:packages)
#
#  1 preflight     Docker, can0, a clean checkout, the tag's commit
#  2 pull, verify  ghcr.io/<owner>/scout-{safety,base}:<tag>: arm64, and built from the tag's commit (OCI revision
#                  label); retagged locally as scout-{safety,base}:<tag>. Perception isn't built in CI: it keeps the
#                  image it runs now.
#  3 scenarios     the release's 15 safety scenarios against the release's safety image, on this Orin, on their own
#                  ROS master (safe next to the live stack). Any failure: stop here, nothing changed.
#  4 stationary    if the stack is up: no recording, wheels still, no command source, no move_base goal
#                  (deploy_check.py). A running navigation stack is restarted with its map, at the robot's pose.
#  5 rollback pt   the running images are tagged scout-{safety,base}:pre-<time>
#  6 apply         robot_down.sh, then robot_up.sh on the new tag (a full restart: master runs the safety image)
#  7 smoke         deploy_check.py smoke: topic rates, safety out of FAULT, localization converged (nav)
#  8 rollback      if 6 or 7 fails: back to pre-<time>, smoke again; exit 1 (2 if the rollback fails too)
# Log: ~/.scout_deploy/deploy-<time>.log; one line per deploy in ~/.scout_deploy/history.csv.
# Env: SCOUT_REGISTRY (default ghcr.io/juttu-s), SMOKE_MIN_*_HZ / SMOKE_MIN_INLIER (see deploy_check.py),
#      DEPLOY_ALLOW_DIRTY=1 (deploy from a checkout with local changes).
set -euo pipefail
usage() { echo "usage: $0 <tag> [--skip-scenarios] [--nav <map> [x y yaw]]" >&2; exit 2; }
[ $# -ge 1 ] || usage
here=$(dirname "$(readlink -f "$0")")
repo=$(dirname "$here")
C="docker compose -f $here/compose.yaml"
registry=${SCOUT_REGISTRY:-ghcr.io/juttu-s}
tag=$1; shift
skip_scenarios=0 nav_args=()
while [ $# -gt 0 ]; do
  case $1 in
    --skip-scenarios) skip_scenarios=1; shift ;;
    --nav) [ $# -ge 2 ] || usage; nav_args=("$@"); break ;;
    *) usage ;;
  esac
done

ts=$(date +%Y%m%d-%H%M%S)
state=$HOME/.scout_deploy
mkdir -p "$state"
exec > >(tee -a "$state/deploy-$ts.log") 2>&1
sha="" scenarios="not run"
[ -f "$state/history.csv" ] || echo "time,tag,commit,result,scenarios" > "$state/history.csv"
record() { echo "$ts,$tag,$sha,${1//,/;},${scenarios//,/;}" >> "$state/history.csv"; }
step() { echo; echo "== $*"; }
abort() { echo "DEPLOY ABORTED (nothing changed): $*" >&2; record "aborted: $*"; exit 1; }

check() {   # deploy_check.py <args> inside the base container, SMOKE_* env passed through
  local env=()
  for v in $(compgen -e | grep '^SMOKE_' || true); do env+=(-e "$v=${!v}"); done
  $C exec -T "${env[@]}" base bash -c 'source /opt/scout/setup.bash && exec python3 - "$@"' -- "$@" \
    < "$here/deploy_check.py"
}

step "1 preflight: $tag"
docker info >/dev/null 2>&1 || abort "no access to Docker"
ip link show can0 2>/dev/null | grep -q "state UP" || abort "can0 is not up: run ~/can_up.sh first"
if [ -n "$(git -C "$repo" status --porcelain)" ] && [ "${DEPLOY_ALLOW_DIRTY:-0}" != 1 ]; then
  abort "$repo has local changes (compose.yaml and the robot scripts come from it); commit them or DEPLOY_ALLOW_DIRTY=1"
fi
git -C "$repo" fetch -q --tags origin 2>/dev/null || echo "warning: git fetch failed, using the local tags"
sha=$(git -C "$repo" rev-list -n1 "$tag" 2>/dev/null) || abort "no git tag $tag (git tag -l 'v*')"
echo "commit $sha"
grep -q "\"${registry%%/*}\"" ~/.docker/config.json 2>/dev/null ||
  abort "not logged in to ${registry%%/*}: docker login ${registry%%/*} -u <github user>  (token with read:packages)"

step "2 pull and verify the images"
for s in safety base; do
  img=$registry/scout-$s:$tag
  docker pull -q "$img" >/dev/null || abort "docker pull $img failed (has CI finished the tag's images job?)"
  arch=$(docker image inspect -f '{{.Architecture}}' "$img")
  rev=$(docker image inspect -f '{{index .Config.Labels "org.opencontainers.image.revision"}}' "$img")
  [ "$arch" = arm64 ] || abort "$img is $arch, not arm64"
  [ "$rev" = "$sha" ] || abort "$img was built from ${rev:-an unknown commit}, not $tag ($sha)"
  docker tag "$img" "scout-$s:$tag"
  echo "scout-$s:$tag  <- $img  (arm64, $rev)"
done

if [ "$skip_scenarios" = 1 ]; then
  scenarios=skipped; echo; echo "== 3 safety scenarios: skipped (--skip-scenarios)"
else
  step "3 safety scenarios of $tag on this Orin (own ROS master)"
  tests=$(mktemp -d)
  out=$state/scenarios-$ts
  mkdir -p "$out"
  git -C "$repo" archive "$tag" detection_ws/src/scout_safety/test | tar -x -C "$tests"
  if docker run --rm --entrypoint bash --user "$(id -u):$(id -g)" \
       -v "$tests/detection_ws/src/scout_safety/test:/test:ro" -v "$out:/out" \
       "scout-safety:$tag" /test/run_scenario.sh --all /out > "$out/run.log" 2>&1; then
    scenarios="$(cat "$out"/*/stdout.log | grep -c ': PASS (' || true)/$(find "$out" -mindepth 1 -maxdepth 1 -type d | wc -l) passed"
  else
    scenarios="failed: $(grep '^failed:' "$out/run.log" | cut -d' ' -f2- || echo "see $out/run.log")"
  fi
  rm -rf "$tests"
  cat "$out"/*/stdout.log 2>/dev/null | grep -E ': (PASS|FAIL) \(|^FAIL|detector stalls' || true
  echo "scenarios: $scenarios  (results: $out)"
  case $scenarios in failed*) abort "safety scenarios $scenarios" ;; esac
fi

step "4 stationary check"
up_args=()
perception_tag=latest
running=0
[ -n "$($C ps -q base 2>/dev/null)" ] && running=1
if [ "$running" = 1 ]; then
  if $C exec -T base pgrep -f '[r]ecord_(nav|session)\.sh' >/dev/null 2>&1; then
    abort "a recording is running in base (record_nav.sh / record_session.sh); stop it first"
  fi
  if ! res=$(check stationary); then
    echo "$res"; abort "the robot is not idle"
  fi
  echo "$res"
  cmd=$(docker inspect -f '{{json .Config.Cmd}}' "$($C ps -q base)")
  if [ ${#nav_args[@]} -eq 0 ] && echo "$cmd" | grep -q '"nav:=true"'; then
    map=$(echo "$cmd" | grep -o '"map:=[^"]*"' | cut -d= -f2 | tr -d '"')
    read -r x y yaw < <(echo "$res" | awk '/^POSE /{print $2, $3, $4}')
    [ -n "${yaw:-}" ] || abort "navigation is running but localization reported no pose: pass --nav <map> x y yaw"
    nav_args=(--nav "$map" "$x" "$y" "$yaw")
    echo "navigation on $map: restarting at the current pose $x $y $yaw"
  fi
  if [ -n "$($C ps -q perception 2>/dev/null)" ]; then
    perception_tag=$(docker inspect -f '{{.Config.Image}}' "$($C ps -q perception)"); perception_tag=${perception_tag##*:}
  fi
else
  echo "stack is down: nothing to check"
fi
up_args=("${nav_args[@]}")
smoke_args=(smoke)
[ ${#up_args[@]} -gt 0 ] && smoke_args+=(--nav)

step "5 rollback point"
prev=pre-$ts
for s in safety base; do
  if [ "$running" = 1 ]; then
    id=$(docker inspect -f '{{.Image}}' "$($C ps -q "$s")")
  else
    id=$(docker image inspect -f '{{.Id}}' "scout-$s:latest" 2>/dev/null) || id=""
  fi
  if [ -n "$id" ]; then docker tag "$id" "scout-$s:$prev"; else prev=""; fi
done
printf 'time %s\nfrom %s\nperception %s\nup_args %s\n' "$ts" "${prev:-none}" "$perception_tag" "${up_args[*]:-}" \
  > "$state/previous"
echo "rollback: ${prev:-none (no previous images)}, perception stays on scout-perception:$perception_tag"

rollback() {
  echo; echo "!! $1"
  if [ -z "$prev" ]; then
    echo "no previous images to roll back to; stopping the stack" >&2
    "$here/robot_down.sh" || true; record "failed, stopped"; exit 2
  fi
  step "8 rolling back to $prev"
  "$here/robot_down.sh" || true
  if TAG=$prev PERCEPTION_TAG=$perception_tag "$here/robot_up.sh" "${up_args[@]}" && check "${smoke_args[@]}"; then
    echo "ROLLED BACK to $prev: $1"; record "rolled back: $1"; exit 1
  fi
  echo "!!!! ROLLBACK FAILED TOO: the robot stack is not healthy. Check: docker compose -f $here/compose.yaml ps" >&2
  record "rollback failed: $1"; exit 2
}

step "6 apply: restart on $tag (perception $perception_tag)"
[ "$running" = 1 ] && "$here/robot_down.sh"
TAG=$tag PERCEPTION_TAG=$perception_tag "$here/robot_up.sh" "${up_args[@]}" || rollback "the stack did not come up healthy"

step "7 smoke test"
check "${smoke_args[@]}" || rollback "smoke test failed"

record deployed
for s in safety base; do   # keep the last three rollback points
  docker images --format '{{.Tag}}' "scout-$s" | grep '^pre-' | sort -r | tail -n +4 |
    while read -r t; do docker rmi -f "scout-$s:$t" >/dev/null; done
done
echo; echo "DEPLOYED $tag ($sha). Rollback: TAG=$prev PERCEPTION_TAG=$perception_tag ~/robot_up.sh ${up_args[*]:-}"
