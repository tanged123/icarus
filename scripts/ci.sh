#!/usr/bin/env bash
set -e

# Ensure we are in a Nix environment
if [ -z "$IN_NIX_SHELL" ]; then
    echo "Not in Nix environment. Re-running inside Nix..."
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    "$SCRIPT_DIR/dev.sh" "$0" "$@"
    exit $?
fi

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Parse arguments
INTERFACE_ARGS=()

show_help() {
    cat << EOF
Usage: $(basename "$0") [OPTIONS]

Run CI (clean release build + tests) for the Icarus project.

Interface Options:
  --c-api           Build and test C API interface
  --python          Build and test Python bindings
  --all-interfaces  Build and test all interfaces
  --interfaces      Alias for --c-api (deprecated)

Other Options:
  -h, --help        Show this help message

Examples:
  ./scripts/ci.sh                    # CI without interfaces
  ./scripts/ci.sh --python           # CI with Python bindings
  ./scripts/ci.sh --all-interfaces   # CI with all interfaces
EOF
    exit 0
}

for arg in "$@"; do
    case $arg in
        -h|--help)
            show_help
            ;;
        --c-api|--interfaces|--python|--all-interfaces)
            INTERFACE_ARGS+=("$arg")
            ;;
        *)
            echo "Warning: Unknown argument ignored: $arg" >&2
            ;;
    esac
done

# Build interface summary for display
INTERFACE_SUMMARY="none"
for arg in "${INTERFACE_ARGS[@]}"; do
    case $arg in
        --c-api|--interfaces) INTERFACE_SUMMARY="c-api" ;;
        --python) INTERFACE_SUMMARY="python" ;;
        --all-interfaces) INTERFACE_SUMMARY="all" ;;
    esac
done

# Ensure logs directory exists
mkdir -p "$PROJECT_ROOT/logs"

# Create timestamp
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="$PROJECT_ROOT/logs/ci_${TIMESTAMP}.log"

# Run build and test scripts (CI uses Release by default for performance)
echo "Running CI (Release build, interfaces: $INTERFACE_SUMMARY)..."
cd "$PROJECT_ROOT"
(./scripts/build.sh --clean --release "${INTERFACE_ARGS[@]}" && ./scripts/test.sh --release "${INTERFACE_ARGS[@]}") 2>&1 | tee "$LOG_FILE"

# Create symlink to latest
ln -sf "ci_${TIMESTAMP}.log" "$PROJECT_ROOT/logs/ci.log"

echo "CI Complete. Logs available at logs/ci_${TIMESTAMP}.log (symlinked to logs/ci.log)"
