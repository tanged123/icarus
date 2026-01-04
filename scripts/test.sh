#!/usr/bin/env bash
set -e

# Ensure we are in a Nix environment
if [ -z "$IN_NIX_SHELL" ]; then
    echo "Not in Nix environment. Re-running inside Nix..."
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    "$SCRIPT_DIR/dev.sh" "$0" "$@"
    exit $?
fi

# Parse arguments to track interface options
BUILD_ARGS=()
RUN_PYTHON_TESTS=false

show_help() {
    cat << EOF
Usage: $(basename "$0") [OPTIONS]

Run tests for the Icarus project.

Build Type Options:
  --debug           Build with debug symbols (default)
  --release         Build with optimizations
  --relwithdebinfo  Build with optimizations and debug info

Interface Options:
  --c-api           Build and test C API interface
  --python          Build and test Python bindings
  --all-interfaces  Build and test all interfaces
  --interfaces      Alias for --c-api (deprecated)

Other Options:
  -h, --help        Show this help message

Examples:
  ./scripts/test.sh                    # Run C++ tests only
  ./scripts/test.sh --python           # Include Python tests
  ./scripts/test.sh --all-interfaces   # Test all interfaces
EOF
    exit 0
}

for arg in "$@"; do
    case $arg in
        -h|--help)
            show_help
            ;;
        --python)
            RUN_PYTHON_TESTS=true
            BUILD_ARGS+=("$arg")
            ;;
        --all-interfaces)
            RUN_PYTHON_TESTS=true
            BUILD_ARGS+=("$arg")
            ;;
        *)
            BUILD_ARGS+=("$arg")
            ;;
    esac
done

# Ensure we have a build (forward build type args to build.sh)
if [ ! -d "build" ]; then
    echo "Build directory not found. Building..."
    ./scripts/build.sh "${BUILD_ARGS[@]}"
else
    # Rebuild to ensure latest changes (forward build type args)
    ./scripts/build.sh "${BUILD_ARGS[@]}"
fi

# Run tests
mkdir -p logs
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="logs/tests_${TIMESTAMP}.log"

echo "=== C++ Tests ==="
ctest --test-dir build -VV 2>&1 | tee "$LOG_FILE"

# Run Python tests if requested and bindings were built
if [ "$RUN_PYTHON_TESTS" = true ]; then
    if [ -d "build/python/icarus" ]; then
        echo ""
        echo "=== Python Tests ==="
        PYTHONPATH=build/python pytest tests/python -v 2>&1 | tee -a "$LOG_FILE"
    else
        echo ""
        echo "Warning: Python tests requested but build/python/icarus not found." >&2
        echo "Did the Python bindings build successfully?" >&2
    fi
fi

# Create symlink to latest
ln -sf "tests_${TIMESTAMP}.log" logs/tests.log
