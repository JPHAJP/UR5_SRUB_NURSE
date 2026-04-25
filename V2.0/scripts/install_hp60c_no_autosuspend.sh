#!/usr/bin/env bash
set -euo pipefail

VENDOR_ID="${HP60C_USB_VENDOR_ID:-3482}"
PRODUCT_ID="${HP60C_USB_PRODUCT_ID:-6723}"
RULE_PATH="${HP60C_UDEV_RULE_PATH:-/etc/udev/rules.d/99-hp60c-no-autosuspend.rules}"
RULE_CONTENT_LINES=(
  "ACTION==\"add\", SUBSYSTEM==\"usb\", ATTR{idVendor}==\"${VENDOR_ID}\", ATTR{idProduct}==\"${PRODUCT_ID}\", TEST==\"power/control\", ATTR{power/control}=\"on\""
  "ACTION==\"add\", SUBSYSTEM==\"usb\", ATTR{idVendor}==\"${VENDOR_ID}\", ATTR{idProduct}==\"${PRODUCT_ID}\", TEST==\"power/autosuspend\", ATTR{power/autosuspend}=\"-1\""
)

if [[ "${EUID:-$(id -u)}" -ne 0 ]]; then
  echo "Ejecuta este script con sudo para instalar la regla udev de la HP60C." >&2
  exit 1
fi

printf '%s\n' "${RULE_CONTENT_LINES[@]}" > "${RULE_PATH}"
chmod 644 "${RULE_PATH}"

udevadm control --reload-rules
udevadm trigger --attr-match=idVendor="${VENDOR_ID}" --attr-match=idProduct="${PRODUCT_ID}" || true

found_device=0
for device in /sys/bus/usb/devices/*; do
  [[ -f "${device}/idVendor" && -f "${device}/idProduct" ]] || continue
  vendor="$(cat "${device}/idVendor" 2>/dev/null || true)"
  product="$(cat "${device}/idProduct" 2>/dev/null || true)"
  if [[ "${vendor}" != "${VENDOR_ID}" || "${product}" != "${PRODUCT_ID}" ]]; then
    continue
  fi
  found_device=1
  if [[ -w "${device}/power/control" ]]; then
    echo on > "${device}/power/control"
  fi
  if [[ -w "${device}/power/autosuspend" ]]; then
    echo -1 > "${device}/power/autosuspend"
  fi
  echo "HP60C detectada en ${device}"
  if [[ -f "${device}/power/control" ]]; then
    echo "power/control=$(cat "${device}/power/control")"
  fi
  if [[ -f "${device}/power/autosuspend" ]]; then
    echo "power/autosuspend=$(cat "${device}/power/autosuspend")"
  fi
done

echo "Regla udev instalada en ${RULE_PATH}"
if [[ "${found_device}" -eq 0 ]]; then
  echo "No se detecto una HP60C ${VENDOR_ID}:${PRODUCT_ID} conectada ahora mismo."
fi

echo "Verifica con:"
echo "  lsusb -t"
echo "  cat /sys/bus/usb/devices/<ruta-hp60c>/power/control"
echo "  cat /sys/bus/usb/devices/<ruta-hp60c>/power/autosuspend"
