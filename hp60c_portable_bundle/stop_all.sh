#!/usr/bin/env bash
set -euo pipefail

kill_by_pattern() {
  local label="$1"
  local pattern="$2"
  local pids
  pids="$(ps -eo pid,args | awk -v p="${pattern}" -v self="$$" -v parent="${PPID}" '$0 ~ p && $1 != self && $1 != parent {print $1}')"
  if [[ -n "${pids}" ]]; then
    echo "Stopping ${label} PIDs: ${pids}"
    kill ${pids} 2>/dev/null || true
    sleep 1
    kill -9 ${pids} 2>/dev/null || true
  fi
}

# Stop any viewer instance from old/new folders.
kill_by_pattern "depth_web_ui.py" "(python|python3|.*/[.]venv/bin/python) .*depth_web_ui[.]py"
# Stop bundle runner wrappers unless the current launcher requested a safe cleanup pass.
if [[ "${HP60C_SKIP_RUNNER_KILL:-0}" != "1" ]]; then
  kill_by_pattern "run_all.sh" "(/| )run_all[.]sh($| )"
  kill_by_pattern "run_depth_web_ui.sh" "(/| )run_depth_web_ui[.]sh($| )"
  kill_by_pattern "run_hp60c_web_ui.sh" "(/| )run_hp60c_web_ui[.]sh($| )"
fi
# Stop direct camera nodes and old launch entrypoints.
kill_by_pattern "ascamera_node" "ascamera_node"
kill_by_pattern "depth_camera.launch.py" "ros2 launch .*depth_camera[.]launch[.]py"

rm -f /tmp/hp60c_portable_bundle.lock

echo "Done."
