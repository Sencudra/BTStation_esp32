#!/bin/bash
set -e  # Exit on any error

echo "Compiling firmware with settings:"
echo "PROJECT_DIR=$PROJECT_DIR"
echo "PROFILE=$PROFILE"
echo "CONFIG_PATH=$CONFIG_PATH"
echo "EXTRA_FLAGS=$EXTRA_FLAGS"

arduino-cli compile \
    --config-file "$CONFIG_PATH" \
    --warnings more \
    --profile "$PROFILE" \
    --build-property "compiler.cpp.extra_flags=$EXTRA_FLAGS" \
    "$PROJECT_DIR" 
