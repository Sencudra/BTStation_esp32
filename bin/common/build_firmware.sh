echo "Compiling firmware with settings:"
echo "FQBN=$FQBN"
echo "PROJECT_DIR=$PROJECT_DIR"
echo "CONFIG_PATH=$CONFIG_PATH"
echo "Flags=$EXTRA_FLAGS"

arduino-cli compile \
    --config-file "$CONFIG_PATH" \
    --warnings more \
    --fqbn "$FQBN" \
    --profile default \
    --build-property "build.extra_flags=$EXTRA_FLAGS" \
    "$PROJECT_DIR" 
