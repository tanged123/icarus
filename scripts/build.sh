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
    esac
done

if [ "$CLEAN" = true ]; then
    echo "Clean build requested."
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    "$SCRIPT_DIR/clean.sh"
fi

echo "Building with CMAKE_BUILD_TYPE=$BUILD_TYPE"

# Create build directory if it doesn't exist or reconfigure
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE="$BUILD_TYPE"

# Build the project
ninja -C build
