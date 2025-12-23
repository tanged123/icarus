#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$PROJECT_ROOT/build"

# Build if needed
if [ ! -d "$BUILD_DIR" ]; then
    echo "Build directory not found. Building..."
    "$SCRIPT_DIR/build.sh"
fi

echo "=== Running all examples ==="
shopt -s nullglob
for example in "$BUILD_DIR"/examples/*; do
    if [ -x "$example" ] && [ -f "$example" ]; then
        name=$(basename "$example")
        echo "--- Running $name ---"
        "$example" || echo "FAILED: $name"
        echo ""
    fi
done
shopt -u nullglob
echo "=== All examples completed ==="
