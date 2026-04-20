#!/bin/bash
# Cross-branch render-speed benchmark.
#
# For every example/*/, times the binary at BASE and HEAD revs and prints
# a per-example speedup ratio. Intended to quantify the cumulative perf
# impact of a branch against a baseline (e.g. deadsy master).
#
# Usage:  run.sh [BASE_REF] [HEAD_REF] [N_REPEATS]
#   BASE_REF   default: c4b81ba (last pre-perf deadsy commit)
#   HEAD_REF   default: HEAD
#   N_REPEATS  default: 3 (median of N reported)
#
# Env:
#   SDFX_BENCHCMP_TIMEOUT  per-run wall-clock cap in seconds (default: 240)
#   SDFX_BENCHCMP_SKIP     space-separated example names to skip
#   SDFX_BENCHCMP_ONLY     space-separated example names — if set, ONLY these

set -u

BASE_REF="${1:-c4b81ba}"
HEAD_REF="${2:-HEAD}"
N_REPEATS="${3:-3}"
TIMEOUT="${SDFX_BENCHCMP_TIMEOUT:-240}"
SKIP="${SDFX_BENCHCMP_SKIP:-}"
ONLY="${SDFX_BENCHCMP_ONLY:-}"

REPO_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
BASE_TREE=$(mktemp -d -t sdfx-benchcmp-base-XXXXXX)
HEAD_TREE=$(mktemp -d -t sdfx-benchcmp-head-XXXXXX)
LOG=$(mktemp -t sdfx-benchcmp-results-XXXXXX.txt)

emit() {
  # Write to stderr (line-buffered by default) for live visibility and to the
  # log file so the final summary is persisted. stdout is block-buffered when
  # the script runs as a background task, which was hiding progress entirely.
  echo "$1" >&2
  echo "$1" >> "$LOG"
}

cleanup() {
  git -C "$REPO_ROOT" worktree remove --force "$BASE_TREE" 2>/dev/null
  git -C "$REPO_ROOT" worktree remove --force "$HEAD_TREE" 2>/dev/null
}
trap cleanup EXIT

emit ">>> preparing worktrees BASE=$BASE_REF HEAD=$HEAD_REF N=$N_REPEATS timeout=${TIMEOUT}s"
git -C "$REPO_ROOT" worktree add --detach "$BASE_TREE" "$BASE_REF" >/dev/null
git -C "$REPO_ROOT" worktree add --detach "$HEAD_TREE" "$HEAD_REF" >/dev/null

# Time a single run using perl's alarm() for a portable hard timeout.
# Echoes real-seconds, or one of FAIL / TIMEOUT.
time_one() {
  local d=$1 n=$2
  local tmp
  tmp=$(mktemp -t sdfx-benchcmp-time-XXXXXX)
  (
    cd "$d" || exit 99
    find . -maxdepth 1 -name '*.stl' -delete 2>/dev/null
    exec perl -e '
      my $t = shift;
      my $pid = fork();
      die "fork: $!" unless defined $pid;
      if ($pid == 0) { exec @ARGV; exit 127; }
      local $SIG{ALRM} = sub { kill 9, $pid; waitpid $pid, 0; exit 124; };
      alarm($t);
      waitpid $pid, 0;
      exit ($? >> 8);
    ' "$TIMEOUT" /usr/bin/time -p "./$n" >/dev/null 2>"$tmp"
  )
  local rc=$?
  if [ "$rc" -eq 124 ]; then
    rm -f "$tmp"
    echo "TIMEOUT"
    return
  fi
  if [ "$rc" -ne 0 ]; then
    rm -f "$tmp"
    echo "FAIL"
    return
  fi
  local t
  t=$(awk '/^real /{print $2; exit}' "$tmp")
  rm -f "$tmp"
  if [ -z "$t" ]; then
    echo "FAIL"
    return
  fi
  echo "$t"
}

# Median of N numeric samples; prints FAIL if none are numeric.
median() {
  python3 -c '
import sys
xs = [float(x) for x in sys.argv[1:] if x.replace(".","",1).replace("-","",1).isdigit()]
if not xs:
    print("FAIL")
else:
    xs.sort()
    print(f"{xs[len(xs)//2]:.3f}")
' "$@"
}

skipme() {
  local name=$1
  for s in $SKIP; do [ "$name" = "$s" ] && return 0; done
  if [ -n "$ONLY" ]; then
    for o in $ONLY; do [ "$name" = "$o" ] && return 1; done
    return 0
  fi
  return 1
}

: > "$LOG"
header=$(printf "%-24s %10s %10s %10s %8s" "example" "base(s)" "head(s)" "speedup" "stls")
emit "$header"
emit "$(printf -- '---------------------------------------------------------------------')"

total_base=0
total_head=0
compared=0
skipped=0
log_sum=0

for dir in "$HEAD_TREE"/examples/*/; do
  name=$(basename "$dir")
  if skipme "$name"; then
    emit "$(printf "%-24s %10s %10s %10s %8s" "$name" "-" "-" "SKIP" "-")"
    skipped=$((skipped+1))
    continue
  fi
  basedir="$BASE_TREE/examples/$name"
  headdir="$HEAD_TREE/examples/$name"
  if [ ! -d "$basedir" ] || [ ! -f "$basedir/main.go" ] || [ ! -f "$headdir/main.go" ]; then
    emit "$(printf "%-24s %10s %10s %10s %8s" "$name" "-" "-" "SKIP" "-")"
    skipped=$((skipped+1))
    continue
  fi
  emit ">>> $name: building"
  ( cd "$basedir" && go build -o "$name" . ) >/dev/null 2>&1 || {
    emit "$(printf "%-24s %10s %10s %10s %8s" "$name" "BUILD?" "-" "-" "-")"
    skipped=$((skipped+1))
    continue
  }
  ( cd "$headdir" && go build -o "$name" . ) >/dev/null 2>&1 || {
    emit "$(printf "%-24s %10s %10s %10s %8s" "$name" "-" "BUILD?" "-" "-")"
    skipped=$((skipped+1))
    continue
  }

  bsamples=()
  hsamples=()
  emit ">>> $name: timing base ($N_REPEATS runs)"
  for i in $(seq 1 "$N_REPEATS"); do
    t=$(time_one "$basedir" "$name"); bsamples+=("$t")
    emit ">>>   base run $i: $t"
  done
  emit ">>> $name: timing head ($N_REPEATS runs)"
  for i in $(seq 1 "$N_REPEATS"); do
    t=$(time_one "$headdir" "$name"); hsamples+=("$t")
    emit ">>>   head run $i: $t"
  done
  bmed=$(median "${bsamples[@]}")
  hmed=$(median "${hsamples[@]}")

  if [ "$bmed" = "FAIL" ] || [ "$hmed" = "FAIL" ]; then
    emit "$(printf "%-24s %10s %10s %10s %8s" "$name" "$bmed" "$hmed" "-" "-")"
    skipped=$((skipped+1))
    continue
  fi

  stlcount=$(find "$headdir" -maxdepth 1 -name '*.stl' 2>/dev/null | wc -l | tr -d ' ')
  ratio=$(python3 -c "print(f'{$bmed/$hmed:.2f}x')")
  emit "$(printf "%-24s %10s %10s %10s %8s" "$name" "$bmed" "$hmed" "$ratio" "$stlcount")"

  total_base=$(python3 -c "print($total_base + $bmed)")
  total_head=$(python3 -c "print($total_head + $hmed)")
  log_sum=$(python3 -c "import math; print($log_sum + math.log($bmed/$hmed))")
  compared=$((compared+1))
done

emit "---"
emit "Summary (base=$BASE_REF head=$HEAD_REF, median of $N_REPEATS runs):"
emit "  compared: $compared   skipped: $skipped"
if [ "$compared" -gt 0 ]; then
  geomean=$(python3 -c "import math; print(f'{math.exp($log_sum / $compared):.2f}x')")
  totratio=$(python3 -c "print(f'{$total_base/$total_head:.2f}x')")
  emit "  total base: ${total_base}s   total head: ${total_head}s   total speedup: $totratio"
  emit "  geomean per-example speedup: $geomean"
fi
emit "  results log: $LOG"
