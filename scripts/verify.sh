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

MODE="all"
INTERFACE_ARGS=()

show_help() {
    cat << EOF
Usage: $(basename "$0") [OPTIONS]

Run full verification (debug + release builds, tests, examples) for the Icarus project.

Build Mode Options:
  --debug           Only run debug build verification
  --release         Only run release build verification
  --all             Run both debug and release (default)

Interface Options:
  --c-api           Build and test C API interface
  --python          Build and test Python bindings
  --all-interfaces  Build and test all interfaces
  --interfaces      Alias for --c-api (deprecated)

Other Options:
  -h, --help        Show this help message

Examples:
  ./scripts/verify.sh                      # Full verification, no interfaces
  ./scripts/verify.sh --release            # Release only
  ./scripts/verify.sh --all-interfaces     # Full verification with all interfaces
  ./scripts/verify.sh --debug --python     # Debug only with Python
EOF
    exit 0
}

while [[ "$#" -gt 0 ]]; do
    case $1 in
        -h|--help) show_help ;;
        --debug) MODE="debug" ;;
        --release) MODE="release" ;;
        --all) MODE="all" ;;
        --c-api|--interfaces|--python|--all-interfaces) INTERFACE_ARGS+=("$1") ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
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

mkdir -p "$PROJECT_ROOT/logs"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="$PROJECT_ROOT/logs/verify_${TIMESTAMP}.log"

echo "Starting verification (Mode: $MODE, interfaces: $INTERFACE_SUMMARY)..." | tee "$LOG_FILE"

cd "$PROJECT_ROOT"
(
    if [[ "$MODE" == "debug" || "$MODE" == "all" ]]; then
        echo '=== Debug Build ==='
        ./scripts/build.sh --clean --debug "${INTERFACE_ARGS[@]}"
        echo '=== Debug Tests ==='
        ./scripts/test.sh --debug "${INTERFACE_ARGS[@]}"
        echo '=== Debug Examples ==='
        ./scripts/run_examples.sh
    fi

    if [[ "$MODE" == "all" ]]; then
        echo ''
    fi

    if [[ "$MODE" == "release" || "$MODE" == "all" ]]; then
        echo '=== Release Build ==='
        ./scripts/build.sh --clean --release "${INTERFACE_ARGS[@]}"
        echo '=== Release Tests ==='
        ./scripts/test.sh --release "${INTERFACE_ARGS[@]}"
        echo '=== Release Examples ==='
        ./scripts/run_examples.sh
    fi
) 2>&1 | tee -a "$LOG_FILE"

ln -sf "verify_${TIMESTAMP}.log" "$PROJECT_ROOT/logs/verify.log"
echo "Verification complete! Logs at logs/verify_${TIMESTAMP}.log"
