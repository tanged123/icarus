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
BUILD_DIR="$PROJECT_ROOT/build"

echo "=== Cleaning and rebuilding with coverage ==="
rm -rf "$BUILD_DIR"
cmake -B "$BUILD_DIR" -G Ninja -DENABLE_COVERAGE=ON
ninja -C "$BUILD_DIR"

echo "=== Running tests ==="
ctest --test-dir "$BUILD_DIR" --output-on-failure

echo "=== Generating coverage report ==="
mkdir -p "$BUILD_DIR/coverage"

# Capture coverage data
lcov --capture \
    --directory "$BUILD_DIR" \
    --output-file "$BUILD_DIR/coverage/coverage.info" \
    --ignore-errors mismatch

# Remove external dependencies from coverage
lcov --remove "$BUILD_DIR/coverage/coverage.info" \
    '/nix/*' \
    '*/tests/*' \
    '*/examples/*' \
    --output-file "$BUILD_DIR/coverage/coverage_clean.info"

# Generate HTML report
genhtml "$BUILD_DIR/coverage/coverage_clean.info" \
    --output-directory "$BUILD_DIR/coverage/html"

echo "=== Coverage report generated ==="
echo "Open $BUILD_DIR/coverage/html/index.html in a browser"
