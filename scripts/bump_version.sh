#!/usr/bin/env bash
set -e

# Usage: ./scripts/bump_version.sh [major|minor|patch]
# Defaults to patch if no argument provided.

CMAKE_FILE="CMakeLists.txt"

if [ ! -f "$CMAKE_FILE" ]; then
    echo "Error: $CMAKE_FILE not found in current directory."
    exit 1
fi

# 1. Get current version
# grep for "VERSION X.Y.Z"
CURRENT_VERSION=$(grep -oP 'VERSION \K[0-9]+\.[0-9]+\.[0-9]+' "$CMAKE_FILE")

if [ -z "$CURRENT_VERSION" ]; then
    echo "Error: Could not find VERSION in $CMAKE_FILE"
    exit 1
fi

IFS='.' read -r MAJOR MINOR PATCH <<< "$CURRENT_VERSION"

echo "Current Version: $MAJOR.$MINOR.$PATCH"

# 2. Increment based on argument
MODE=${1:-patch}

if [ "$MODE" == "major" ]; then
    MAJOR=$((MAJOR + 1))
    MINOR=0
    PATCH=0
elif [ "$MODE" == "minor" ]; then
    MINOR=$((MINOR + 1))
    PATCH=0
elif [ "$MODE" == "patch" ]; then
    PATCH=$((PATCH + 1))
else
    echo "Usage: $0 [major|minor|patch]"
    exit 1
fi

NEW_VERSION="$MAJOR.$MINOR.$PATCH"

# 3. Update CMakeLists.txt
# Check if using GNU sed or BSD sed (macOS)
if sed --version 2>/dev/null | grep -q GNU; then
    SED="sed -i"
else
    SED="sed -i ''"
fi

$SED "s/VERSION $CURRENT_VERSION/VERSION $NEW_VERSION/" "$CMAKE_FILE"
echo "Updated $CMAKE_FILE"

# 4. Update flake.nix (if exists)
FLAKE_FILE="flake.nix"
if [ -f "$FLAKE_FILE" ]; then
    # Matches version = "..."; and replaces content inside quotes
    $SED "s/version = \".*\";/version = \"$NEW_VERSION\";/" "$FLAKE_FILE"
    echo "Updated $FLAKE_FILE"
fi

# 5. Update CoreTypes.hpp version macros (if exists)
TYPES_FILE="include/icarus/core/CoreTypes.hpp"
if [ -f "$TYPES_FILE" ]; then
    $SED "s/#define ICARUS_VERSION_MAJOR [0-9]*/#define ICARUS_VERSION_MAJOR $MAJOR/" "$TYPES_FILE"
    $SED "s/#define ICARUS_VERSION_MINOR [0-9]*/#define ICARUS_VERSION_MINOR $MINOR/" "$TYPES_FILE"
    $SED "s/#define ICARUS_VERSION_PATCH [0-9]*/#define ICARUS_VERSION_PATCH $PATCH/" "$TYPES_FILE"
    echo "Updated $TYPES_FILE"
fi

# 6. Update C API fallback version (icarus_c.cpp)
C_API_FILE="interfaces/c_api/icarus_c.cpp"
if [ -f "$C_API_FILE" ]; then
    $SED "s/#define ICARUS_VERSION_STRING \"[0-9]*\.[0-9]*\.[0-9]*\"/#define ICARUS_VERSION_STRING \"$NEW_VERSION\"/" "$C_API_FILE"
    $SED "s/#define ICARUS_VERSION_MAJOR [0-9]*/#define ICARUS_VERSION_MAJOR $MAJOR/" "$C_API_FILE"
    $SED "s/#define ICARUS_VERSION_MINOR [0-9]*/#define ICARUS_VERSION_MINOR $MINOR/" "$C_API_FILE"
    $SED "s/#define ICARUS_VERSION_PATCH [0-9]*/#define ICARUS_VERSION_PATCH $PATCH/" "$C_API_FILE"
    echo "Updated $C_API_FILE"
fi

# 7. Update Python bindings fallback version (icarus_python.cpp)
PYTHON_API_FILE="interfaces/python/icarus_python.cpp"
if [ -f "$PYTHON_API_FILE" ]; then
    $SED "s/#define ICARUS_VERSION_STRING \"[0-9]*\.[0-9]*\.[0-9]*\"/#define ICARUS_VERSION_STRING \"$NEW_VERSION\"/" "$PYTHON_API_FILE"
    $SED "s/#define ICARUS_VERSION_MAJOR [0-9]*/#define ICARUS_VERSION_MAJOR $MAJOR/" "$PYTHON_API_FILE"
    $SED "s/#define ICARUS_VERSION_MINOR [0-9]*/#define ICARUS_VERSION_MINOR $MINOR/" "$PYTHON_API_FILE"
    $SED "s/#define ICARUS_VERSION_PATCH [0-9]*/#define ICARUS_VERSION_PATCH $PATCH/" "$PYTHON_API_FILE"
    echo "Updated $PYTHON_API_FILE"
fi

# 8. Update Python __init__.py docstring example
PYTHON_INIT_FILE="interfaces/python/icarus/__init__.py"
if [ -f "$PYTHON_INIT_FILE" ]; then
    $SED "s/print(icarus.__version__).*# \"[0-9]*\.[0-9]*\.[0-9]*\"/print(icarus.__version__)      # \"$NEW_VERSION\"/" "$PYTHON_INIT_FILE"
    $SED "s/print(icarus.version_info).*# ([0-9]*, [0-9]*, [0-9]*)/print(icarus.version_info)     # ($MAJOR, $MINOR, $PATCH)/" "$PYTHON_INIT_FILE"
    echo "Updated $PYTHON_INIT_FILE"
fi

echo "Bumped to: $NEW_VERSION"

# Optional: Git tag suggestion
echo ""
echo "Don't forget to commit and tag:"
echo "  git add -u"
echo "  git commit -m \"chore: bump version to $NEW_VERSION\""
echo "  git tag v$NEW_VERSION"
