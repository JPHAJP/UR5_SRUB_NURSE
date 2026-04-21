#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
TOOLS_DIR="${ROOT_DIR}/.tools/cloudflared"
BINARY_PATH="${TOOLS_DIR}/cloudflared"

ARCH="$(uname -m)"
case "${ARCH}" in
  x86_64|amd64)
    ASSET_NAME="cloudflared-linux-amd64"
    ;;
  aarch64|arm64)
    ASSET_NAME="cloudflared-linux-arm64"
    ;;
  armv7l|armv6l)
    ASSET_NAME="cloudflared-linux-arm"
    ;;
  i386|i686)
    ASSET_NAME="cloudflared-linux-386"
    ;;
  *)
    echo "Arquitectura no soportada para instalacion automatica de cloudflared: ${ARCH}" >&2
    exit 1
    ;;
esac

mkdir -p "${TOOLS_DIR}"
DOWNLOAD_URL="https://github.com/cloudflare/cloudflared/releases/latest/download/${ASSET_NAME}"

echo "Descargando cloudflared desde ${DOWNLOAD_URL}"
curl -fsSL "${DOWNLOAD_URL}" -o "${BINARY_PATH}"
chmod +x "${BINARY_PATH}"

echo "cloudflared instalado en ${BINARY_PATH}"
"${BINARY_PATH}" --version
