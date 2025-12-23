#!/bin/bash
set -e

# Ensure we are in a Nix environment
if [ -z "$IN_NIX_SHELL" ]; then
    echo "Not in Nix environment. Re-running inside Nix..."
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    "$SCRIPT_DIR/dev.sh" "$0" "$@"
    exit $?
fi

# Ensure doxygen is installed
if ! command -v doxygen &> /dev/null; then
    echo "Doxygen could not be found. Please install it."
    exit 1
fi

echo "Generating documentation..."
mkdir -p build/docs
doxygen Doxyfile

# Copy interactive HTML examples and user guides
echo "Copying interactive examples..."
cp -r docs/examples build/docs/html/examples 2>/dev/null || true
cp -r docs/user_guides build/docs/html/user_guides 2>/dev/null || true

echo "Documentation generated in build/docs/html"
