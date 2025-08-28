# Used to install all the required project dependencies.

#!/bin/bash
set -e  # exit on error

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Paths relative to script
PROJECT_DIR="$SCRIPT_DIR/../BTStation_ESP32"
CONFIG_PATH="$PROJECT_DIR/arduino_cli.yaml"
LIBRARIES_DESCRIPTION_PATH="$PROJECT_DIR/libs/description.txt"
ZIP_LIBRARIES_DIR="$PROJECT_DIR/libs_custom/"
ZIP_LIBRARIES_DESCRIPTION_PATH="$ZIP_LIBRARIES_DIR/description.txt"
PLATFORM="esp32:esp32"

export PROJECT_DIR CONFIG_PATH PLATFORM 
export LIBRARIES_DESCRIPTION_PATH 
export ZIP_LIBRARIES_DESCRIPTION_PATH ZIP_LIBRARIES_DIR

"$SCRIPT_DIR/common/install_requirements.sh"