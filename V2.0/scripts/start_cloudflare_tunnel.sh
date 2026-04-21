#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOCAL_HOST="${TUNNEL_LOCAL_HOST:-127.0.0.1}"
LOCAL_PORT="${FLASK_PORT:-5050}"
LOCAL_URL="${TUNNEL_URL:-http://${LOCAL_HOST}:${LOCAL_PORT}}"
LOCAL_BINARY="${ROOT_DIR}/.tools/cloudflared/cloudflared"

if command -v cloudflared >/dev/null 2>&1; then
  CLOUDFLARED_BIN="$(command -v cloudflared)"
elif [[ -x "${LOCAL_BINARY}" ]]; then
  CLOUDFLARED_BIN="${LOCAL_BINARY}"
else
  echo "cloudflared no esta instalado. Lo instalo localmente en ${ROOT_DIR}/.tools..." >&2
  "${SCRIPT_DIR}/install_cloudflared.sh" >&2
  CLOUDFLARED_BIN="${LOCAL_BINARY}"
fi

python3 - "${LOCAL_HOST}" "${LOCAL_PORT}" <<'PY'
import socket
import sys

host = sys.argv[1]
port = int(sys.argv[2])
sock = socket.socket()
sock.settimeout(1.0)
try:
    sock.connect((host, port))
except OSError as exc:
    raise SystemExit(
        f"La app V2 no responde en http://{host}:{port}. Arrancala antes de abrir el tunel. Error: {exc}"
    )
finally:
    sock.close()
PY

echo "Publicando ${LOCAL_URL} con Cloudflare Quick Tunnel..."
exec "${CLOUDFLARED_BIN}" tunnel --url "${LOCAL_URL}"
