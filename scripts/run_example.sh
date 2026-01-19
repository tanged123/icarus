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
shift  # Remove first arg, rest are passed to the example

# Extract target name and extension
TARGET_NAME=$(basename "$INPUT_NAME")
EXTENSION="${TARGET_NAME##*.}"
TARGET_NAME="${TARGET_NAME%.*}"

cd "$PROJECT_ROOT"

# Handle Python files
if [[ "$EXTENSION" == "py" || "$INPUT_NAME" == *.py ]]; then
    echo "Running Python example: $INPUT_NAME"

    # Check for the icarus Python package in build/python/
    PYTHON_PKG_DIR="$PROJECT_ROOT/build/python"
    if [ ! -d "$PYTHON_PKG_DIR/icarus" ]; then
        echo "Python bindings not found. Building with --python..."
        ./scripts/build.sh --python
    fi

    if [ ! -d "$PYTHON_PKG_DIR/icarus" ]; then
        echo "Error: Could not find Python bindings. Build with --python or --all-interfaces."
        exit 1
    fi

    # Resolve the input path
    if [[ "$INPUT_NAME" == /* ]]; then
        SCRIPT_PATH="$INPUT_NAME"
    elif [ -f "$INPUT_NAME" ]; then
        SCRIPT_PATH="$INPUT_NAME"
    elif [ -f "examples/python/$INPUT_NAME" ]; then
        SCRIPT_PATH="examples/python/$INPUT_NAME"
    elif [ -f "examples/python/${TARGET_NAME}.py" ]; then
        SCRIPT_PATH="examples/python/${TARGET_NAME}.py"
    elif [ -f "tests/python/$INPUT_NAME" ]; then
        SCRIPT_PATH="tests/python/$INPUT_NAME"
    else
        echo "Error: Could not find Python file: $INPUT_NAME"
        exit 1
    fi

    echo "---------------------------------------------------"
    PYTHONPATH="$PYTHON_PKG_DIR:$PYTHONPATH" python "$SCRIPT_PATH" "$@"
    echo "---------------------------------------------------"
    exit 0
fi

# Handle C++ examples
echo "Targeting C++ example: $TARGET_NAME"

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
    "$EXEC_PATH" "$@"
    echo "---------------------------------------------------"
else
    echo "Error: Could not find executable for target '$TARGET_NAME' in build directory."
    exit 1
fi
