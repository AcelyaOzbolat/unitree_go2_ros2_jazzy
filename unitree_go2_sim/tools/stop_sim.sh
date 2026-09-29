#!/bin/bash
# Stop every simulation process belonging to this workspace.
#
# Matching is done on the executable path, and this script's own process tree is
# excluded explicitly. Doing the same thing inline with `pkill -f "ros2 launch"`
# kills the very shell running the cleanup - the caller's command line contains
# the pattern too - so the loop dies halfway and leaves half the stack running.
# Stale nodes then publish alongside the fresh ones and quietly corrupt every
# measurement that follows.
SELF=$$
PARENT=$PPID
NAME=$(basename "$0")

collect() {
  ps -eo pid=,args= | awk -v self="$SELF" -v parent="$PARENT" -v name="$NAME" '
    $1 == self || $1 == parent { next }
    index($0, name) { next }
    /\/opt\/ros\/[^ ]*\/lib\// ||
    /ros2_ws\/install\/[^ ]*\/lib\// ||
    /gz_tools_vendor/ ||
    /[ \/]gz sim/ ||
    /[ \/]rviz2/ { print $1 }
  '
}

PIDS=$(collect)
if [ -n "$PIDS" ]; then
  kill -9 $PIDS 2>/dev/null
fi

for _ in $(seq 1 40); do
  LEFT=$(collect | wc -l)
  [ "$LEFT" -eq 0 ] && break
done

echo "kalan simulasyon sureci: $LEFT"
