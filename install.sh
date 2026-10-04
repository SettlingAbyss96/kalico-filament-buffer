#!/usr/bin/env bash
# Install kalico-filament-buffer into a Kalico installation by symlinking.
# Safe to re-run. Usage:  ./install.sh [--with-tests]
#   --with-tests  also link config/buffer-test.cfg into the printer config
#                 directory (then add [include buffer-test.cfg] to printer.cfg)
# Environment overrides: KLIPPER_DIR (default ~/klipper),
#                        PRINTER_CONFIG_DIR (default ~/printer_data/config)
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KLIPPER_DIR="${KLIPPER_DIR:-$HOME/klipper}"
PRINTER_CONFIG_DIR="${PRINTER_CONFIG_DIR:-$HOME/printer_data/config}"

if [ ! -d "$KLIPPER_DIR/klippy/plugins" ]; then
    echo "error: $KLIPPER_DIR/klippy/plugins not found." >&2
    echo "This plugin needs Kalico (mainline Klipper has no plugin directory)." >&2
    exit 1
fi

ln -sfn "$REPO/filament_buffer.py" "$KLIPPER_DIR/klippy/plugins/filament_buffer.py"
echo "linked $KLIPPER_DIR/klippy/plugins/filament_buffer.py -> $REPO/filament_buffer.py"

if [ "${1:-}" = "--with-tests" ]; then
    ln -sfn "$REPO/config/buffer-test.cfg" "$PRINTER_CONFIG_DIR/buffer-test.cfg"
    echo "linked $PRINTER_CONFIG_DIR/buffer-test.cfg (add [include buffer-test.cfg] to printer.cfg)"
fi

echo "Run the offline tests:  (cd \"$REPO\" && python3 -m unittest discover -s tests)"
echo "Then restart Klipper:   sudo systemctl restart klipper"
