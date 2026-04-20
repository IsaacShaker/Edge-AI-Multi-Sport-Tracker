#!/bin/bash
# Sync project source code to Raspberry Pi over SSH using rsync.
#
# Usage:
#   ./5-sync-to-pi.sh                        # sync to saved PI_HOST
#   ./5-sync-to-pi.sh pi@raspberrypi.local   # sync to explicit host
#   ./5-sync-to-pi.sh --from-pi              # pull changes back from pi
#
# On first run, set PI_HOST below or pass the address as an argument.
# SSH key-based auth is strongly recommended (no password prompts):
#   ssh-keygen -t ed25519        # if you don't have a key yet
#   ssh-copy-id <PI_HOST>        # copy key to the pi once

set -e

# ── Configuration ────────────────────────────────────────────────────────────
PI_HOST="${PI_HOST:-compute@10.5.60.97}"  # e.g. pi@192.168.1.42  or  pi@raspberrypi.local
PI_DEST="${PI_DEST:-~/Edge-AI-Multi-Sport-Tracker}"  # destination path on the pi
# ─────────────────────────────────────────────────────────────────────────────

FROM_PI=false

# Parse arguments
for arg in "$@"; do
    case "$arg" in
        --from-pi)
            FROM_PI=true
            ;;
        --help|-h)
            sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *@*|*raspberrypi*|*pi*)
            PI_HOST="$arg"
            ;;
        *)
            echo "Unknown argument: $arg"
            echo "Run '$0 --help' for usage."
            exit 1
            ;;
    esac
done

if [ -z "$PI_HOST" ]; then
    echo "Error: No Raspberry Pi host specified."
    echo ""
    echo "Either:"
    echo "  1. Edit PI_HOST in this script"
    echo "  2. Export it: export PI_HOST=pi@raspberrypi.local"
    echo "  3. Pass it as an argument: $0 pi@raspberrypi.local"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Files/dirs to exclude from sync
EXCLUDES=(
    --exclude='.git/'
    --exclude='.venv/'
    --exclude='server/build/'
    --exclude='__pycache__/'
    --exclude='*.pyc'
    --exclude='*.o'
    --exclude='*.a'
    --exclude='*.so'
    --exclude='.DS_Store'
    --exclude='*.log'
    --exclude='prediction_log.csv'
)

RSYNC_OPTS=(-avz --progress "${EXCLUDES[@]}")

if [ "$FROM_PI" = true ]; then
    echo "=== Pulling changes FROM Pi ($PI_HOST:$PI_DEST) ==="
    echo "    -> $SCRIPT_DIR"
    echo ""
    rsync "${RSYNC_OPTS[@]}" "${PI_HOST}:${PI_DEST}/" "$SCRIPT_DIR/"
    echo ""
    echo "=== Pull complete ==="
else
    echo "=== Syncing TO Pi ($PI_HOST:$PI_DEST) ==="
    echo "    <- $SCRIPT_DIR"
    echo ""
    # Create destination directory on pi if it doesn't exist
    ssh "$PI_HOST" "mkdir -p $PI_DEST"
    rsync "${RSYNC_OPTS[@]}" "$SCRIPT_DIR/" "${PI_HOST}:${PI_DEST}/"
    echo ""
    echo "=== Sync complete ==="
    echo ""
    echo "To build on the Pi, run:"
    echo "  ssh $PI_HOST 'cd $PI_DEST && ./pi-1-install-dependencies.sh'"
    echo "  ssh $PI_HOST 'cd $PI_DEST && ./pi-3-build.sh'"
    echo ""
    echo "To run on the Pi:"
    echo "  ssh $PI_HOST 'cd $PI_DEST && ./pi-4-run.sh --no-display'"
fi
