#!/usr/bin/env bash
# ROS 2 Jazzy + Gazebo Harmonic setup for Ubuntu 24.04 (WSL2). Runs as root INSIDE the distro.
# Normally invoked by install.ps1; can also be run by hand:
#   sudo bash setup.sh --user ros --mirror auto
# Idempotent: safe to re-run.
set -Eeuo pipefail

ROS_USER="ros"
MIRROR="auto"          # auto | huawei | aliyun | tuna | ustc | official
ROBOTS=1               # 1 = also install Nav2 / SLAM Toolbox / TurtleBot3
SIMULATE=0             # 1 = apt-get -s (no real package install, for testing)
ROS_KEY_FILE=""        # optional local ros.key for offline / blocked networks
ROS_KEY_FPR="C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654"   # Open Robotics signing key

while [ $# -gt 0 ]; do
  case "$1" in
    --user)         ROS_USER="$2"; shift 2 ;;
    --mirror)       MIRROR="$2"; shift 2 ;;
    --no-robots)    ROBOTS=0; shift ;;
    --simulate)     SIMULATE=1; shift ;;
    --ros-key-file) ROS_KEY_FILE="$2"; shift 2 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

log()  { printf '\n[%s] === %s\n' "$(date +%H:%M:%S)" "$*"; }
warn() { printf '[WARN] %s\n' "$*" >&2; }
die()  { printf '[ERROR] %s\n' "$*" >&2; exit 1; }
trap 'die "failed at line $LINENO (exit $?)"' ERR

[ "$(id -u)" -eq 0 ] || die "must run as root"
# shellcheck disable=SC1091
. /etc/os-release
[ "${VERSION_CODENAME:-}" = "noble" ] || die "Ubuntu 24.04 (noble) required, found: ${PRETTY_NAME:-unknown}"
export DEBIAN_FRONTEND=noninteractive
APT=(apt-get -y -q -o Acquire::Retries=5 -o Dpkg::Use-Pty=0 -o APT::Color=0)
INSTALL=("${APT[@]}" install --no-install-recommends)
[ "$SIMULATE" -eq 1 ] && INSTALL=("${APT[@]}" -s install --no-install-recommends)

# ---------------------------------------------------------------- 1. user + wsl.conf
log "1/7 user '$ROS_USER' and /etc/wsl.conf"
if ! id "$ROS_USER" >/dev/null 2>&1; then
  groups=""
  for g in sudo video render; do getent group "$g" >/dev/null && groups="${groups:+$groups,}$g"; done
  useradd -m -s /bin/bash ${groups:+-G "$groups"} "$ROS_USER"
fi
echo "$ROS_USER ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/90-$ROS_USER"
chmod 440 "/etc/sudoers.d/90-$ROS_USER"
cat > /etc/wsl.conf <<EOF
[boot]
systemd=true
[user]
default=$ROS_USER
EOF

# ---------------------------------------------------------------- 2. mirror selection
log "2/7 choose apt mirrors (MIRROR=$MIRROR)"
declare -A UBU=(
  [huawei]="https://repo.huaweicloud.com/ubuntu"
  [aliyun]="https://mirrors.aliyun.com/ubuntu"
  [tuna]="https://mirrors.tuna.tsinghua.edu.cn/ubuntu"
  [ustc]="https://mirrors.ustc.edu.cn/ubuntu"
  [official]="http://archive.ubuntu.com/ubuntu"
)
declare -A ROSM=(
  [huawei]="https://repo.huaweicloud.com/ros2/ubuntu"
  [aliyun]="https://mirrors.aliyun.com/ros2/ubuntu"
  [tuna]="https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu"
  [ustc]="https://mirrors.ustc.edu.cn/ros2/ubuntu"
  [official]="https://packages.ros.org/ros2/ubuntu"
)
ORDER=(huawei aliyun tuna ustc official)

# prints download speed in bytes/s (0 if unreachable or TLS invalid)
probe() {
  curl -sS -L -o /dev/null -m 15 -r 0-1500000 -w '%{http_code} %{speed_download}' "$1" 2>/dev/null \
    | awk '($1==200||$1==206){printf "%d",$2; ok=1} END{if(!ok)printf "0"}' || true
}
pick_best() {  # $1 = name of assoc array, $2 = path suffix to probe
  local -n arr="$1"; local best="" bestspeed=0 k s
  for k in "${ORDER[@]}"; do
    s=$(probe "${arr[$k]}$2")
    printf '    %-9s %10s B/s  %s\n' "$k" "$s" "${arr[$k]}" >&2
    if [ "$s" -gt "$bestspeed" ]; then best="$k"; bestspeed="$s"; fi
  done
  echo "$best"
}

if [ "$MIRROR" = "auto" ]; then
  echo "  probing Ubuntu mirrors..."
  UBU_SEL=$(pick_best UBU "/dists/noble/main/binary-amd64/Packages.gz")
  echo "  probing ROS 2 mirrors..."
  ROS_SEL=$(pick_best ROSM "/dists/noble/main/binary-amd64/Packages.gz")
  [ -n "$UBU_SEL" ] || { warn "no Ubuntu mirror reachable, keeping image defaults"; UBU_SEL=official; }
  [ -n "$ROS_SEL" ] || { warn "no ROS mirror reachable over HTTPS, falling back to official over http"; ROS_SEL=official; ROSM[official]="http://packages.ros.org/ros2/ubuntu"; }
else
  [ -n "${UBU[$MIRROR]:-}" ] || die "unknown mirror '$MIRROR'"
  UBU_SEL="$MIRROR"; ROS_SEL="$MIRROR"
fi
echo "  -> Ubuntu: $UBU_SEL (${UBU[$UBU_SEL]})"
echo "  -> ROS 2 : $ROS_SEL (${ROSM[$ROS_SEL]})"

if [ "$UBU_SEL" != "official" ]; then
  if [ -f /etc/apt/sources.list.d/ubuntu.sources ]; then
    [ -f /etc/apt/sources.list.d/ubuntu.sources.orig ] || cp /etc/apt/sources.list.d/ubuntu.sources /etc/apt/sources.list.d/ubuntu.sources.orig
    sed -i -E "s#^URIs:.*#URIs: ${UBU[$UBU_SEL]}/#" /etc/apt/sources.list.d/ubuntu.sources
  elif [ -f /etc/apt/sources.list ]; then
    sed -i -E "s#https?://(archive|security)\.ubuntu\.com/ubuntu#${UBU[$UBU_SEL]}#g" /etc/apt/sources.list
  fi
fi

# ---------------------------------------------------------------- 3. base tools + ROS key + ROS source
log "3/7 base tools, ROS 2 signing key, ROS 2 apt source"
"${APT[@]}" update
"${APT[@]}" install --no-install-recommends curl ca-certificates gnupg

KEYTMP=$(mktemp)
if [ -n "$ROS_KEY_FILE" ]; then
  cp "$ROS_KEY_FILE" "$KEYTMP"
else
  ok=0
  # GitHub raw is the canonical location (the key is fingerprint-checked below, so the source need not be trusted)
  for attempt in 1 2 3 4; do
    # the file may be binary or ASCII-armored; let gpg decide whether it is a valid key
    if curl -fsSL -m 90 --retry 2 --retry-all-errors "https://raw.githubusercontent.com/ros/rosdistro/master/ros.key" -o "$KEYTMP" 2>/dev/null \
       && gpg --show-keys --with-colons "$KEYTMP" >/dev/null 2>&1; then ok=1; break; fi
    echo "  ros.key download attempt $attempt failed, retrying..."; sleep 3
  done
  [ "$ok" -eq 1 ] || die "could not download ros.key from GitHub; download https://raw.githubusercontent.com/ros/rosdistro/master/ros.key manually and pass --ros-key-file"
fi
FPR=$(gpg --show-keys --with-colons "$KEYTMP" 2>/dev/null | awk -F: '$1=="fpr"{print $10; exit}')
[ "$FPR" = "$ROS_KEY_FPR" ] || die "ROS key fingerprint mismatch (got '$FPR', expected '$ROS_KEY_FPR') - refusing to continue"
install -d -m 0755 /usr/share/keyrings
if grep -q 'BEGIN PGP' "$KEYTMP"; then gpg --dearmor < "$KEYTMP" > /usr/share/keyrings/ros-archive-keyring.gpg; else cp "$KEYTMP" /usr/share/keyrings/ros-archive-keyring.gpg; fi
chmod 644 /usr/share/keyrings/ros-archive-keyring.gpg
rm -f "$KEYTMP"
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] ${ROSM[$ROS_SEL]} noble main" > /etc/apt/sources.list.d/ros2.list
"${APT[@]}" update

# ---------------------------------------------------------------- 4. system upgrade
log "4/7 system upgrade"
if [ "$SIMULATE" -eq 1 ]; then echo "  (simulate) skipped"; else "${APT[@]}" upgrade; fi

# ---------------------------------------------------------------- 5. ROS 2 + Gazebo
log "5/7 ROS 2 Jazzy desktop + Gazebo Harmonic + ros2_control (large download)"
"${INSTALL[@]}" \
  build-essential git python3-pip mesa-utils vulkan-tools mesa-vulkan-drivers \
  ros-dev-tools ros-jazzy-desktop ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control ros-jazzy-ros2-control ros-jazzy-ros2-controllers

if [ "$ROBOTS" -eq 1 ]; then
  log "6/7 Nav2, SLAM Toolbox, TurtleBot3"
  "${INSTALL[@]}" \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
    ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-simulations \
    ros-jazzy-teleop-twist-keyboard ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui
else
  log "6/7 robot packages skipped (--no-robots)"
fi

# ---------------------------------------------------------------- 7. user environment
log "7/7 user environment for '$ROS_USER'"
HOME_DIR=$(getent passwd "$ROS_USER" | cut -d: -f6)
ENVF="$HOME_DIR/.ros-wsl-env"
cat > "$ENVF" <<'EOF'
# Managed by ros-wsl installer. Sourced from ~/.bashrc.
source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
export TURTLEBOT3_MODEL=waffle
# WSLg: render through the Windows GPU (AMD/Intel/NVIDIA) via Mesa D3D12 instead of CPU llvmpipe
if [ -e /dev/dxg ]; then
  export GALLIUM_DRIVER=d3d12
  export MESA_LOADER_DRIVER_OVERRIDE=d3d12
  export LD_LIBRARY_PATH=/usr/lib/wsl/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
fi
EOF
touch "$HOME_DIR/.bashrc"
sed -i '/# >>> ros-wsl >>>/,/# <<< ros-wsl <<</d' "$HOME_DIR/.bashrc"
cat >> "$HOME_DIR/.bashrc" <<'EOF'
# >>> ros-wsl >>>
[ -f "$HOME/.ros-wsl-env" ] && source "$HOME/.ros-wsl-env"
# <<< ros-wsl <<<
EOF
mkdir -p "$HOME_DIR/ros2_ws/src" "$HOME_DIR/sim_worlds"
chown -R "$ROS_USER:$ROS_USER" "$HOME_DIR/.ros-wsl-env" "$HOME_DIR/.bashrc" "$HOME_DIR/ros2_ws" "$HOME_DIR/sim_worlds"

log "SETUP COMPLETE (user=$ROS_USER, ubuntu mirror=$UBU_SEL, ros mirror=$ROS_SEL, simulate=$SIMULATE)"
