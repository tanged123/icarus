#!/usr/bin/env bash
set -e

# Ensure we are in a Nix environment
if [ -z "$IN_NIX_SHELL" ]; then
    echo "Not in Nix environment. Re-running inside Nix..."
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    "$SCRIPT_DIR/dev.sh" "$0" "$@"
    exit $?
fi

# Ensure we have a build (forward build type args to build.sh)
if [ ! -d "build" ]; then
    echo "Build directory not found. Building..."
    ./scripts/build.sh "$@"
else
    # Rebuild to ensure latest changes (forward build type args)
    ./scripts/build.sh "$@"
fi

# Run tests
mkdir -p logs
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="logs/tests_${TIMESTAMP}.log"

ctest --test-dir build -VV 2>&1 | tee "$LOG_FILE"

# Run Python tests if Python bindings were built
if [ -d "build/python/icarus" ]; then
    echo ""
    echo "=== Python Tests ==="
    PYTHONPATH=build/python pytest tests/python -v 2>&1 | tee -a "$LOG_FILE"
fi

# Create symlink to latest
ln -sf "tests_${TIMESTAMP}.log" logs/tests.log
