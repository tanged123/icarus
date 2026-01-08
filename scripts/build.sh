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
BUILD_C_API="${BUILD_C_API:-OFF}"  # C API bindings
BUILD_PYTHON="${BUILD_PYTHON:-OFF}"  # Python bindings (requires pybind11)

# Default jobs: half of available cores, capped at 6 (minimum 2)
# This prevents OOM on 32GB systems with heavy template code (CasADi/Janus)
DEFAULT_JOBS=$(( $(nproc) / 2 ))
[ "$DEFAULT_JOBS" -lt 2 ] && DEFAULT_JOBS=2
[ "$DEFAULT_JOBS" -gt 6 ] && DEFAULT_JOBS=6
JOBS="${JOBS:-$DEFAULT_JOBS}"

# Argument parsing state
NEXT_IS_JOBS=false

show_help() {
    cat << EOF
Usage: $(basename "$0") [OPTIONS]

Build the Icarus project.

Build Type Options:
  --debug           Build with debug symbols (default)
  --release         Build with optimizations
  --relwithdebinfo  Build with optimizations and debug info
  --clean           Clean build directory before building

Interface Options:
  --c-api           Build C API interface
  --python          Build Python bindings (requires pybind11)
  --all-interfaces  Build all interfaces (C API + Python)
  --interfaces      Alias for --c-api (deprecated)

Other Options:
  -j, --jobs N      Number of parallel build jobs (default: half of cores)
  -h, --help        Show this help message

Environment Variables:
  BUILD_TYPE        Override build type
  BUILD_C_API       Set to ON to enable C API
  BUILD_PYTHON      Set to ON to enable Python bindings
  JOBS              Override parallel job count

Examples:
  ./scripts/build.sh                    # Debug build, no interfaces
  ./scripts/build.sh --release          # Release build
  ./scripts/build.sh --python           # Debug build with Python bindings
  ./scripts/build.sh --all-interfaces   # Build all interfaces
  ./scripts/build.sh --release --c-api  # Release build with C API
EOF
    exit 0
}

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
        -h|--help)
            show_help
            ;;
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
        --c-api|--interfaces)
            BUILD_C_API="ON"
            ;;
        --python)
            BUILD_PYTHON="ON"
            ;;
        --all-interfaces)
            BUILD_C_API="ON"
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

# Build interface summary
INTERFACE_SUMMARY=""
[ "$BUILD_C_API" = "ON" ] && INTERFACE_SUMMARY="${INTERFACE_SUMMARY}c-api "
[ "$BUILD_PYTHON" = "ON" ] && INTERFACE_SUMMARY="${INTERFACE_SUMMARY}python "
[ -z "$INTERFACE_SUMMARY" ] && INTERFACE_SUMMARY="none"

echo "Building with CMAKE_BUILD_TYPE=$BUILD_TYPE (jobs: $JOBS, interfaces: $INTERFACE_SUMMARY)"

# Check if we need to reconfigure due to interface flag changes
# CMake caches variables, so changing flags requires explicit reconfigure
NEED_RECONFIGURE=false
if [ -f "build/CMakeCache.txt" ]; then
    CACHED_PYTHON=$(grep -E "^BUILD_PYTHON:" build/CMakeCache.txt 2>/dev/null | cut -d= -f2 || echo "OFF")
    CACHED_C_API=$(grep -E "^BUILD_INTERFACES:" build/CMakeCache.txt 2>/dev/null | cut -d= -f2 || echo "OFF")

    if [ "$BUILD_PYTHON" = "ON" ] && [ "$CACHED_PYTHON" != "ON" ]; then
        echo "Python bindings requested but not in current build config - forcing reconfigure..."
        NEED_RECONFIGURE=true
    fi
    if [ "$BUILD_C_API" = "ON" ] && [ "$CACHED_C_API" != "ON" ]; then
        echo "C API requested but not in current build config - forcing reconfigure..."
        NEED_RECONFIGURE=true
    fi

    if [ "$NEED_RECONFIGURE" = true ]; then
        rm -f build/CMakeCache.txt
    fi
fi

# Show ccache stats before build
if command -v ccache &> /dev/null; then
    echo ""
    echo "=== ccache stats (before build) ==="
    ccache -s | grep -E "(Hits|Misses|Hit rate|Cache size)" || ccache -s | head -10
fi

# Create build directory if it doesn't exist or reconfigure
cmake -B build -G Ninja \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DBUILD_INTERFACES="$BUILD_C_API" \
    -DBUILD_PYTHON="$BUILD_PYTHON"

# Build the project with limited parallelism to prevent OOM
ninja -C build -j "$JOBS"

# Show ccache stats after build
if command -v ccache &> /dev/null; then
    echo ""
    echo "=== ccache stats (after build) ==="
    ccache -s | grep -E "(Hits|Misses|Hit rate|Cache size)" || ccache -s | head -10
fi
