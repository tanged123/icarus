#!/usr/bin/env bash
set -e

# Ensure we are in a Nix environment
if [ -z "$IN_NIX_SHELL" ]; then
    echo "Not in Nix environment. Re-running inside Nix..."
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    "$SCRIPT_DIR/dev.sh" "$0" "$@"
    exit $?
fi

# Get script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Check if an argument is provided
if [ -z "$1" ]; then
    echo "Usage: $0 <example_source_file_or_target_name>"
    echo "Example: $0 examples/simulation/drag_coefficient.cpp"
    echo "Example: $0 drag_coefficient"
    exit 1
fi



INPUT_NAME="$1"

# Extract target name (basename without extension)
TARGET_NAME=$(basename "$INPUT_NAME")
TARGET_NAME="${TARGET_NAME%.*}"

echo "Targeting example: $TARGET_NAME"

# Build the specific target
cd "$PROJECT_ROOT"

# Ensure build directory exists
if [ ! -d "build" ]; then
    echo "Build directory not found. Running full build first..."
    ./scripts/build.sh
fi

echo "Building target '$TARGET_NAME'..."
cmake --build build --target "$TARGET_NAME"

# Find the executable
# CMake might put it in build/examples/subdir/target or just build/target depending on configuration
# We use 'find' to locate it reliably.
EXEC_PATH=$(find build -type f -name "$TARGET_NAME" -executable | head -n 1)

if [ -n "$EXEC_PATH" ]; then
    echo "Running '$TARGET_NAME'..."
    echo "---------------------------------------------------"
    "$EXEC_PATH"
    echo "---------------------------------------------------"
else
    echo "Error: Could not find executable for target '$TARGET_NAME' in build directory."
    exit 1
fi
