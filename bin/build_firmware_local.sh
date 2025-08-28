# Used to built firmware on local machine.
# Run install_requirements_local before building firmware.

#!/bin/bash
set -e  # exit on error

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Paths relative to script
PROJECT_DIR="$SCRIPT_DIR/../BTStation_ESP32"
CONFIG_PATH="$PROJECT_DIR/arduino_cli.yaml"
PROFILE="default"

EXTRA_FLAGS=""
if [ "$DEBUG" = "1" ]; then
    EXTRA_FLAGS="$EXTRA_FLAGS -DDEBUG"
fi

if [ "$DEBUG" = "1" ]; then
    EXTRA_FLAGS="$EXTRA_FLAGS -DUSE_PN532"
fi

export PROJECT_DIR PROFILE CONFIG_PATH EXTRA_FLAGS

rm -Rf "$PROJECT_DIR/build"

"$SCRIPT_DIR/common/build_firmware.sh"