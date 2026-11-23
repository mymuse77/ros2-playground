#!/usr/bin/env bash
# Verify the ROS 2 + Gazebo + GPU environment. Run as the ROS user (NOT root):
#   bash verify.sh          # quick checks (~20 s)
#   bash verify.sh --tb3    # also run a headless TurtleBot3 Gazebo simulation (~1 min)
# Exit code = number of FAILED checks.
FAIL=0; WARN=0
pass() { printf '  [PASS] %s\n' "$*"; }
fail() { printf '  [FAIL] %s\n' "$*"; FAIL=$((FAIL+1)); }
warn() { printf '  [WARN] %s\n' "$*"; WARN=$((WARN+1)); }

# shellcheck disable=SC1090
source "$HOME/.ros-wsl-env" 2>/dev/null || { echo "~/.ros-wsl-env missing - run setup.sh first"; exit 99; }

echo "== ROS 2"
[ "${ROS_DISTRO:-}" = "jazzy" ] && pass "ROS_DISTRO=jazzy" || fail "ROS_DISTRO is '${ROS_DISTRO:-unset}'"
command -v ros2 >/dev/null && pass "ros2 CLI found" || fail "ros2 not found"

TMP=$(mktemp -d); trap 'kill $(jobs -p) 2>/dev/null; rm -rf "$TMP"' EXIT
ros2 run demo_nodes_cpp listener > "$TMP/l.log" 2>&1 &
sleep 2
timeout 8 ros2 run demo_nodes_cpp talker > "$TMP/t.log" 2>&1
grep -q "I heard" "$TMP/l.log" && pass "talker -> listener message delivery" || fail "listener heard nothing"

echo "== Gazebo"
GZV=$(gz sim --versions 2>/dev/null | head -n 1)
[ -n "$GZV" ] && pass "gz sim $GZV" || fail "gz sim not working"

echo "== GPU rendering (WSLg)"
if [ -e /dev/dxg ]; then pass "/dev/dxg present (GPU passthrough available)"; else warn "/dev/dxg missing: no GPU passthrough, will use CPU rendering"; fi
if [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
  warn "no DISPLAY/WAYLAND_DISPLAY (WSLg not active?) - skipping renderer check"
else
  R=$(glxinfo -B 2>/dev/null | sed -n 's/^OpenGL renderer string: //p')
  case "$R" in
    D3D12*)   pass "OpenGL renderer: $R" ;;
    llvmpipe*) warn "OpenGL renderer is $R (CPU). Update the Windows GPU driver, then 'wsl --shutdown'" ;;
    *)        warn "OpenGL renderer: '${R:-unknown}'" ;;
  esac
fi

if [ "${1:-}" = "--tb3" ]; then
  echo "== TurtleBot3 headless simulation"
  if ! dpkg -s ros-jazzy-turtlebot3-gazebo >/dev/null 2>&1; then
    warn "turtlebot3 not installed (installed with --no-robots?) - skipped"
  else
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py gui:=false > "$TMP/tb3.log" 2>&1 &
    TB3=$!
    sleep 40
    timeout 15 ros2 topic echo /scan --once >/dev/null 2>&1 && pass "/scan lidar data received" || fail "no /scan data"
    X0=$(timeout 8 ros2 topic echo /odom --once 2>/dev/null | awk '/position:/{f=1} f&&/x:/{print $2; exit}')
    timeout 4 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: base_link}, twist: {linear: {x: 0.2}}}" >/dev/null 2>&1
    X1=$(timeout 8 ros2 topic echo /odom --once 2>/dev/null | awk '/position:/{f=1} f&&/x:/{print $2; exit}')
    if [ -n "$X0" ] && [ -n "$X1" ] && awk -v a="$X0" -v b="$X1" 'BEGIN{exit !((b-a)>0.2)}'; then
      pass "robot drove forward (odom x: $X0 -> $X1)"
    else
      fail "robot did not move (odom x: '${X0}' -> '${X1}')"
    fi
    kill "$TB3" 2>/dev/null; sleep 2; pkill -f 'gz sim' 2>/dev/null; pkill -f ruby 2>/dev/null
  fi
fi

echo
echo "Result: $FAIL failed, $WARN warnings"
exit "$FAIL"
