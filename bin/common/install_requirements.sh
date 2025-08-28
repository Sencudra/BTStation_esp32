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

echo "All requirements are installed."