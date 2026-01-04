#!/usr/bin/env bash
set -e

# Ensure we are in a Nix environment
if [ -z "$IN_NIX_SHELL" ]; then
    echo "Not in Nix environment. Re-running inside Nix..."
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    "$SCRIPT_DIR/dev.sh" "$0" "$@"
    exit $?
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "Running examples..."

# Ensure output directory exists for examples that record data
mkdir -p "$PROJECT_ROOT/output"

# Find and run all examples in the build directory
EXAMPLES_DIR="$PROJECT_ROOT/build/examples"

if [ -d "$EXAMPLES_DIR" ]; then
    # Find all executable files in the examples directory (recursive)
    # Sort them to ensure deterministic run order
    # Exclude stage_separation as it's deprecated/removed
    for exe in $(find "$EXAMPLES_DIR" -type f -executable | grep -v "stage_separation" | grep -v "CMakeFiles" | sort); do
        echo ""
        echo "=== Running $(basename "$exe") ==="
        "$exe"
    done
else
    echo "Examples directory not found at $EXAMPLES_DIR. Did you build the project?"
fi

echo ""
echo "All examples completed."