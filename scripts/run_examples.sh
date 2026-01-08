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

# Parse arguments
RUN_PYTHON=false
for arg in "$@"; do
    case $arg in
        --python|--all-interfaces) RUN_PYTHON=true ;;
    esac
done

echo "Running examples..."

# Ensure output directory exists for examples that record data
mkdir -p "$PROJECT_ROOT/output"

# =============================================================================
# C++ Examples
# =============================================================================
EXAMPLES_DIR="$PROJECT_ROOT/build/examples"

if [ -d "$EXAMPLES_DIR" ]; then
    # Find all executable files in the examples directory (recursive)
    # Sort them to ensure deterministic run order
    # Exclude stage_separation as it's deprecated/removed
    for exe in $(find "$EXAMPLES_DIR" -type f -executable | grep -v "stage_separation" | grep -v "CMakeFiles" | sort); do
        echo ""
        echo "=== Running $(basename "$exe") ==="
        "$exe"
    done
else
    echo "Examples directory not found at $EXAMPLES_DIR. Did you build the project?"
fi

# =============================================================================
# Python Examples (when --python or --all-interfaces specified)
# =============================================================================
PYTHON_EXAMPLES_DIR="$PROJECT_ROOT/examples/python"
PYTHON_PKG_DIR="$PROJECT_ROOT/build/python"

if [ "$RUN_PYTHON" = true ] && [ -d "$PYTHON_EXAMPLES_DIR" ]; then
    if [ -d "$PYTHON_PKG_DIR/icarus" ]; then
        export PYTHONPATH="$PYTHON_PKG_DIR:$PYTHONPATH"

        echo ""
        echo "=== Python Examples ==="

        for py_example in $(find "$PYTHON_EXAMPLES_DIR" -name "*.py" | sort); do
            echo ""
            echo "=== Running $(basename "$py_example") ==="
            python "$py_example"
        done
    else
        echo ""
        echo "Warning: Python bindings not found, skipping Python examples."
        echo "Build with --python or --all-interfaces to enable."
    fi
elif [ "$RUN_PYTHON" = true ]; then
    echo ""
    echo "No Python examples found in $PYTHON_EXAMPLES_DIR"
fi

echo ""
echo "All examples completed."