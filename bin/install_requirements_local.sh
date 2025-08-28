# Used to install all the required project dependencies.

#!/bin/bash
set -e  # exit on error

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Paths relative to script
PROJECT_DIR="$SCRIPT_DIR/../BTStation_ESP32"
CONFIG_PATH="$PROJECT_DIR/arduino_cli.yaml"
PLATFORM="esp32:esp32"

export PROJECT_DIR CONFIG_PATH PLATFORM 

"$SCRIPT_DIR/common/install_requirements.sh"