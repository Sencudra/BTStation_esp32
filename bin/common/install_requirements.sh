#!/bin/bash
set -e  # Exit on any error

if ! command -v arduino-cli >/dev/null 2>&1; then
    echo "Error: arduino-cli not found in PATH. Please install Arduino CLI first."
    exit 1
fi

echo "Updating index from $CONFIG_PATH"
arduino-cli core update-index --config-file "$CONFIG_PATH"

echo "Installing core for $PLATFORM"
arduino-cli core install --config-file "$CONFIG_PATH" "$PLATFORM"

if [ ! -f "$LIBRARIES_DESCRIPTION_PATH" ]; then
    echo "No file $LIBRARIES_DESCRIPTION_PATH found"
    exit 1
fi
echo "Installing 3rd party dependencies from manager."
xargs -a "$LIBRARIES_DESCRIPTION_PATH" -r -I {} sh -c 'arduino-cli lib install --config-file '"$CONFIG_PATH"' "{}"'

if [ ! -f "$ZIP_LIBRARIES_DESCRIPTION_PATH" ]; then
    echo "No $ZIP_LIBRARIES_DESCRIPTION_PATH for zip libraries found"
    exit 1
fi
echo "Installing 3rd party dependencies from repository"
xargs -a "$ZIP_LIBRARIES_DESCRIPTION_PATH" -r -I {} sh -c 'arduino-cli lib install --config-file '"$CONFIG_PATH"' --zip-path '"$ZIP_LIBRARIES_DIR/{}.zip"''

echo "All requirements are installed."