#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-${SCRIPT_DIR}/workspace}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
LOCK_FILE="/tmp/hp60c_portable_bundle.lock"
VENV_DIR="${VENV_DIR:-${SCRIPT_DIR}/.venv}"

WEB_HOST="${WEB_HOST:-127.0.0.1}"
WEB_PORT="${WEB_PORT:-5000}"
V4L2_DEVICE="${V4L2_DEVICE:-/dev/video2}"
LAUNCH_CAMERA="${LAUNCH_CAMERA:-1}"
ENABLE_OPENCV_FALLBACK="${ENABLE_OPENCV_FALLBACK:-0}"
AUTO_STOP_CONFLICTS="${AUTO_STOP_CONFLICTS:-1}"

RGB_TOPIC="${RGB_TOPIC:-/ascamera/camera_publisher/rgb0/image}"
DEPTH_TOPIC="${DEPTH_TOPIC:-/ascamera/camera_publisher/depth0/image_raw}"
ASCAMERA_CONFIG_PATH="${ASCAMERA_CONFIG_PATH:-${WORKSPACE_DIR}/src/ascamera/configurationfiles}"

ensure_viewer_venv() {
  if [[ -x "${VENV_DIR}/bin/python" ]]; then
    return 0
  fi

  echo "Creating viewer virtual environment at ${VENV_DIR}..."
  rm -rf "${VENV_DIR}"
  python3 -m venv --system-site-packages "${VENV_DIR}"
}

if command -v flock >/dev/null 2>&1; then
  exec 9>"${LOCK_FILE}"
  if ! flock -n 9; then
    echo "Another viewer instance is already running."
    echo "Use ${SCRIPT_DIR}/stop_all.sh first."
    exit 1
  fi
fi

if [[ "${AUTO_STOP_CONFLICTS}" == "1" ]] && [[ -x "${SCRIPT_DIR}/stop_all.sh" ]]; then
  echo "Cleaning up conflicting old camera/viewer processes..."
  HP60C_SKIP_RUNNER_KILL=1 "${SCRIPT_DIR}/stop_all.sh" >/dev/null 2>&1 || true
fi

cleanup() {
  echo ""
  echo "Stopping processes..."
  [[ -n "${CAM_PID:-}" ]] && kill "${CAM_PID}" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ERROR: ROS setup file not found: /opt/ros/${ROS_DISTRO}/setup.bash"
  exit 1
fi

ensure_viewer_venv

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  source "${WORKSPACE_DIR}/install/setup.bash"
fi
source "${VENV_DIR}/bin/activate"
set -u

export PYTHONNOUSERSITE=1
PYTHON_BIN="${VENV_DIR}/bin/python"

if [[ ! -d "${WORKSPACE_DIR}/src/ascamera" ]]; then
  echo "ERROR: Driver source missing at ${WORKSPACE_DIR}/src/ascamera"
  echo "Run installer first: ${SCRIPT_DIR}/install_all.sh"
  exit 1
fi

missing_mods=()
for m in flask cv2 rclpy cv_bridge; do
  if ! "${PYTHON_BIN}" -c "import ${m}" >/dev/null 2>&1; then
    missing_mods+=("${m}")
  fi
done
if (( ${#missing_mods[@]} > 0 )); then
  echo "Missing python modules: ${missing_mods[*]}"
  echo "Run installer first: ${SCRIPT_DIR}/install_all.sh"
  exit 1
fi

if ! ros2 pkg prefix ascamera >/dev/null 2>&1; then
  echo "ascamera package is not built yet. Building now..."
  ( cd "${WORKSPACE_DIR}" && colcon build --packages-select ascamera --symlink-install )
  set +u
  source "${WORKSPACE_DIR}/install/setup.bash"
  set -u
fi

if command -v ss >/dev/null 2>&1; then
  while ss -ltn "( sport = :${WEB_PORT} )" | tail -n +2 | grep -q .; do
    WEB_PORT=$((WEB_PORT + 1))
  done
fi

if [[ -e "${V4L2_DEVICE}" ]] && command -v fuser >/dev/null 2>&1; then
  busy_pids="$(fuser "${V4L2_DEVICE}" 2>/dev/null | xargs || true)"
  if [[ -n "${busy_pids}" ]]; then
    echo "WARNING: ${V4L2_DEVICE} is currently busy (PIDs: ${busy_pids})"
    ps -fp ${busy_pids} || true
  fi
fi

CAM_PID=""
if [[ "${LAUNCH_CAMERA}" == "1" ]]; then
  if [[ ! -d "${ASCAMERA_CONFIG_PATH}" ]]; then
    echo "ERROR: ASCAMERA_CONFIG_PATH not found: ${ASCAMERA_CONFIG_PATH}"
    exit 1
  fi

  echo "Starting HP60C camera node..."
  ros2 run ascamera ascamera_node --ros-args \
    -r __ns:=/ascamera \
    -p usb_bus_no:=-1 \
    -p usb_path:=null \
    -p confiPath:="${ASCAMERA_CONFIG_PATH}" \
    -p color_pcl:=true \
    -p pub_tfTree:=true \
    -p depth_width:=640 \
    -p depth_height:=480 \
    -p rgb_width:=640 \
    -p rgb_height:=480 \
    -p fps:=15 &
  CAM_PID=$!
  sleep 2
else
  echo "Skipping camera launch because LAUNCH_CAMERA=${LAUNCH_CAMERA}"
fi

echo "Starting web viewer at http://${WEB_HOST}:${WEB_PORT}"
extra_args=()
if [[ "${ENABLE_OPENCV_FALLBACK}" == "1" ]]; then
  extra_args+=(--enable-opencv-fallback)
fi

"${PYTHON_BIN}" "${SCRIPT_DIR}/depth_web_ui.py" \
  --host "${WEB_HOST}" \
  --port "${WEB_PORT}" \
  --rgb-topic "${RGB_TOPIC}" \
  --depth-topic "${DEPTH_TOPIC}" \
  --v4l2-device "${V4L2_DEVICE}" \
  "${extra_args[@]}"
