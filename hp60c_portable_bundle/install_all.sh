#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-${SCRIPT_DIR}/workspace}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ASCAM_FILE_ID="1bLGx735xdgjjUNUWo6T_JeqfXyzV_gtI"
ROS_SETUP_FILE="/opt/ros/${ROS_DISTRO}/setup.bash"
VENV_DIR="${VENV_DIR:-${SCRIPT_DIR}/.venv}"

ensure_viewer_venv() {
  echo "[5/6] Creating isolated Python environment at ${VENV_DIR}..."
  rm -rf "${VENV_DIR}"
  python3 -m venv --system-site-packages "${VENV_DIR}"

  export PYTHONNOUSERSITE=1
  # Keep ROS Python packages importable while blocking ~/.local NumPy 2.x overrides.
  set +u
  source "${ROS_SETUP_FILE}"
  if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
    source "${WORKSPACE_DIR}/install/setup.bash"
  fi
  source "${VENV_DIR}/bin/activate"
  set -u

  python - <<'PY'
import flask
import cv2
import cv_bridge
import numpy
import rclpy

print("Viewer venv ready")
print("  python:", __import__("sys").executable)
print("  numpy:", numpy.__version__, numpy.__file__)
print("  cv_bridge:", cv_bridge.__file__)
PY
}

run_sudo() {
  if [[ "${EUID}" -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

ensure_ros_installed() {
  if [[ -f "${ROS_SETUP_FILE}" ]]; then
    return 0
  fi

  if [[ ! -r /etc/os-release ]]; then
    echo "ERROR: /etc/os-release not found"
    echo "Cannot determine whether ROS 2 ${ROS_DISTRO} can be installed automatically."
    exit 1
  fi

  # Auto-bootstrap ROS only on the officially supported Ubuntu release.
  . /etc/os-release
  local os_id="${ID:-unknown}"
  local version_id="${VERSION_ID:-unknown}"
  local codename="${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}"

  if [[ "${os_id}" != "ubuntu" || "${version_id}" != "24.04" ]]; then
    echo "ERROR: ${ROS_SETUP_FILE} not found"
    echo "Automatic ROS 2 ${ROS_DISTRO} install is supported only on Ubuntu 24.04."
    echo "Detected: ${PRETTY_NAME:-${os_id} ${version_id}}"
    echo "Follow the official ROS install guide, then rerun this script:"
    echo "  https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html"
    exit 1
  fi

  echo "ROS 2 ${ROS_DISTRO} not found. Installing ROS apt source and ros-base..."
  run_sudo apt update
  run_sudo apt install -y locales curl software-properties-common
  run_sudo add-apt-repository -y universe

  if ! locale | grep -qi 'utf-8'; then
    run_sudo locale-gen en_US en_US.UTF-8
    run_sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
  fi

  local ros_apt_source_version
  local ros_apt_source_json
  ros_apt_source_json="$(
    curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest
  )"
  ros_apt_source_version="$(
    printf '%s\n' "${ros_apt_source_json}" | awk -F '"' '/tag_name/ {print $4; exit}'
  )"
  if [[ -z "${ros_apt_source_version}" ]]; then
    echo "ERROR: Failed to determine the latest ros2-apt-source release"
    exit 1
  fi

  local ros_apt_deb="/tmp/ros2-apt-source_${ros_apt_source_version}.${codename}_all.deb"
  curl -fL -o "${ros_apt_deb}" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_source_version}/ros2-apt-source_${ros_apt_source_version}.${codename}_all.deb"
  run_sudo dpkg -i "${ros_apt_deb}"
  rm -f "${ros_apt_deb}"

  run_sudo apt update
  run_sudo apt install -y "ros-${ROS_DISTRO}-ros-base"

  if [[ ! -f "${ROS_SETUP_FILE}" ]]; then
    echo "ERROR: ROS 2 ${ROS_DISTRO} installation completed, but ${ROS_SETUP_FILE} is still missing"
    exit 1
  fi
}

ensure_ros_installed

echo "[1/6] Installing system dependencies..."
run_sudo apt update
mapfile -t COMMON_DEPS < <(grep -Ev '^\s*(#|$)' "${SCRIPT_DIR}/dependencies_apt.txt")
run_sudo apt install -y \
  "${COMMON_DEPS[@]}" \
  "ros-${ROS_DISTRO}-cv-bridge" \
  "ros-${ROS_DISTRO}-image-transport" \
  "ros-${ROS_DISTRO}-image-publisher" \
  "ros-${ROS_DISTRO}-camera-info-manager" \
  "ros-${ROS_DISTRO}-pcl-conversions"

echo "[2/6] Preparing workspace at ${WORKSPACE_DIR}..."
mkdir -p "${WORKSPACE_DIR}/src"

if [[ -d "${WORKSPACE_DIR}/build" || -d "${WORKSPACE_DIR}/install" || -d "${WORKSPACE_DIR}/log" ]]; then
  echo "Removing stale colcon build artifacts from copied workspace..."
  rm -rf "${WORKSPACE_DIR}/build" "${WORKSPACE_DIR}/install" "${WORKSPACE_DIR}/log"
fi

LOCAL_DRIVER_DIR="${SCRIPT_DIR}/drivers/ascamera"
if [[ -d "${LOCAL_DRIVER_DIR}" ]]; then
  echo "Using bundled local driver source from ${LOCAL_DRIVER_DIR}"
  rsync -a --delete "${LOCAL_DRIVER_DIR}/" "${WORKSPACE_DIR}/src/ascamera/"
else
  echo "Bundled driver not found, downloading ascamera package..."
  TMP_DIR="$(mktemp -d)"
  trap 'rm -rf "${TMP_DIR}"' EXIT

  python3 - <<PY
import os, re, requests
file_id='${ASCAM_FILE_ID}'
out='${TMP_DIR}/ascam_ros2_ws.zip'

s=requests.Session()
page=s.get(f'https://drive.google.com/uc?export=download&id={file_id}', timeout=60)
text=page.text
m=re.search(r'name="uuid" value="([^"]+)"', text)
params={'id':file_id,'export':'download'}
if m:
    params.update({'confirm':'t','uuid':m.group(1)})

resp=s.get('https://drive.usercontent.google.com/download', params=params, stream=True, timeout=120)
resp.raise_for_status()

os.makedirs(os.path.dirname(out), exist_ok=True)
with open(out,'wb') as f:
    for chunk in resp.iter_content(chunk_size=1024*1024):
        if chunk:
            f.write(chunk)
print('Downloaded', out, os.path.getsize(out))
PY

  unzip -q "${TMP_DIR}/ascam_ros2_ws.zip" -d "${TMP_DIR}/ascam_ros2_ws"
  unzip -q "${TMP_DIR}/ascam_ros2_ws/src.zip" -d "${TMP_DIR}/unpacked"

  ASCAM_DIR="$(find "${TMP_DIR}/unpacked" -type d -name ascamera | head -n 1 || true)"
  if [[ -z "${ASCAM_DIR}" ]]; then
    echo "ERROR: Could not find ascamera package in downloaded archive"
    exit 1
  fi

  rsync -a --delete "${ASCAM_DIR}/" "${WORKSPACE_DIR}/src/ascamera/"
fi

echo "[3/6] Installing udev camera rule..."
RULE_SRC="${SCRIPT_DIR}/angstrong-camera.rules"
if [[ ! -f "${RULE_SRC}" ]]; then
  echo "ERROR: Missing ${RULE_SRC}"
  exit 1
fi

run_sudo install -m 0644 "${RULE_SRC}" /etc/udev/rules.d/angstrong-camera.rules
run_sudo udevadm control --reload-rules
run_sudo udevadm trigger

CURRENT_USER="${SUDO_USER:-$USER}"
if id -nG "${CURRENT_USER}" | grep -qw video; then
  echo "User ${CURRENT_USER} already in video group"
else
  run_sudo usermod -aG video "${CURRENT_USER}"
  echo "Added ${CURRENT_USER} to video group"
fi

echo "[4/6] Building ascamera package..."
cd "${WORKSPACE_DIR}"
set +u
source "${ROS_SETUP_FILE}"
set -u
colcon build --packages-select ascamera --symlink-install

ensure_viewer_venv

echo "[6/6] Done"
echo "Next steps:"
echo "  1) Log out/in (or reboot) to apply video-group permissions"
echo "  2) Reconnect the HP60C camera"
echo "  3) Run: ${SCRIPT_DIR}/run_all.sh"
