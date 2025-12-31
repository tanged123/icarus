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

# Default jobs: half of available cores, minimum 2
# This prevents OOM on 32GB systems with heavy template code (CasADi/Janus)
DEFAULT_JOBS=$(( $(nproc) / 2 ))
[ "$DEFAULT_JOBS" -lt 2 ] && DEFAULT_JOBS=2
JOBS="${JOBS:-$DEFAULT_JOBS}"

for arg in "$@"; do
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
        --jobs=*|-j=*)
            JOBS="${arg#*=}"
            ;;
        --jobs|-j)
            # Next argument will be the job count (handled below)
            NEXT_IS_JOBS=true
            ;;
        *)
            if [ "$NEXT_IS_JOBS" = true ]; then
                JOBS="$arg"
                NEXT_IS_JOBS=false
            fi
            ;;
    esac
done

if [ "$CLEAN" = true ]; then
    echo "Clean build requested."
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    "$SCRIPT_DIR/clean.sh"
fi

echo "Building with CMAKE_BUILD_TYPE=$BUILD_TYPE (jobs: $JOBS)"

# Create build directory if it doesn't exist or reconfigure
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE="$BUILD_TYPE"

# Build the project with limited parallelism to prevent OOM
ninja -C build -j "$JOBS"
