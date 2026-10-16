#!/bin/bash
# Shortcut wrapper for provision_pi.sh
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCRIPT_DIR/provision_pi.sh" "$@"
