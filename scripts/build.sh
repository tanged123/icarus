#!/usr/bin/env bash
set -e

# Ensure we are in a Nix environment
if [ -z "$IN_NIX_SHELL" ]; then
    echo "Not in Nix environment. Re-running inside Nix..."
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    "$SCRIPT_DIR/dev.sh" "$0" "$@"
    exit $?
fi

# Handle arguments
CLEAN=false
BUILD_TYPE="${BUILD_TYPE:-Debug}"  # Default to Debug for local development
BUILD_INTERFACES="${BUILD_INTERFACES:-OFF}"  # C API bindings
BUILD_PYTHON="${BUILD_PYTHON:-OFF}"  # Python bindings (requires pybind11)

# Default jobs: half of available cores, minimum 2
# This prevents OOM on 32GB systems with heavy template code (CasADi/Janus)
DEFAULT_JOBS=$(( $(nproc) / 2 ))
[ "$DEFAULT_JOBS" -lt 2 ] && DEFAULT_JOBS=2
JOBS="${JOBS:-$DEFAULT_JOBS}"

# Argument parsing state
NEXT_IS_JOBS=false

for arg in "$@"; do
    # If previous arg was -j/--jobs, this arg should be the job count
    if [ "$NEXT_IS_JOBS" = true ]; then
        if [[ "$arg" =~ ^[0-9]+$ ]]; then
            JOBS="$arg"
            NEXT_IS_JOBS=false
            continue
        else
            echo "Error: --jobs/-j requires a numeric argument, got: '$arg'" >&2
            exit 1
        fi
    fi

    case $arg in
        --clean)
            CLEAN=true
            ;;
        --debug)
            BUILD_TYPE="Debug"
            ;;
        --release)
            BUILD_TYPE="Release"
            ;;
        --relwithdebinfo)
            BUILD_TYPE="RelWithDebInfo"
            ;;
        --interfaces)
            BUILD_INTERFACES="ON"
            ;;
        --python)
            BUILD_PYTHON="ON"
            ;;
        --jobs=*|-j=*)
            JOBS="${arg#*=}"
            ;;
        --jobs|-j)
            # Next argument will be the job count
            NEXT_IS_JOBS=true
            ;;
        *)
            echo "Warning: Unknown argument ignored: $arg" >&2
            ;;
    esac
done

# Check if -j/--jobs was the last argument without a value
if [ "$NEXT_IS_JOBS" = true ]; then
    echo "Error: --jobs/-j requires a numeric argument" >&2
    exit 1
fi

# Validate JOBS is numeric
if ! [[ "$JOBS" =~ ^[0-9]+$ ]]; then
    echo "Error: Invalid job count: '$JOBS' (must be numeric)" >&2
    exit 1
fi


if [ "$CLEAN" = true ]; then
    echo "Clean build requested."
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    "$SCRIPT_DIR/clean.sh"
fi

echo "Building with CMAKE_BUILD_TYPE=$BUILD_TYPE (jobs: $JOBS, interfaces: $BUILD_INTERFACES, python: $BUILD_PYTHON)"

# Show ccache stats before build
if command -v ccache &> /dev/null; then
    echo ""
    echo "=== ccache stats (before build) ==="
    ccache -s | grep -E "(Hits|Misses|Hit rate|Cache size)" || ccache -s | head -10
fi

# Create build directory if it doesn't exist or reconfigure
cmake -B build -G Ninja \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DBUILD_INTERFACES="$BUILD_INTERFACES" \
    -DBUILD_PYTHON="$BUILD_PYTHON"

# Build the project with limited parallelism to prevent OOM
ninja -C build -j "$JOBS"

# Show ccache stats after build
if command -v ccache &> /dev/null; then
    echo ""
    echo "=== ccache stats (after build) ==="
    ccache -s | grep -E "(Hits|Misses|Hit rate|Cache size)" || ccache -s | head -10
fi
