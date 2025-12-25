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

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --debug) MODE="debug" ;;
        --release) MODE="release" ;;
        --all) MODE="all" ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

mkdir -p "$PROJECT_ROOT/logs"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="$PROJECT_ROOT/logs/verify_${TIMESTAMP}.log"

echo "Starting verification (Mode: $MODE)..." | tee "$LOG_FILE"

cd "$PROJECT_ROOT"
(
    if [[ "$MODE" == "debug" || "$MODE" == "all" ]]; then
        echo '=== Debug Build ==='
        ./scripts/build.sh --clean --debug
        echo '=== Debug Tests ==='
        ./scripts/test.sh --debug
        echo '=== Debug Examples ==='
        ./scripts/run_examples.sh
    fi

    if [[ "$MODE" == "all" ]]; then
        echo ''
    fi

    if [[ "$MODE" == "release" || "$MODE" == "all" ]]; then
        echo '=== Release Build ==='
        ./scripts/build.sh --clean --release
        echo '=== Release Tests ==='
        ./scripts/test.sh --release
        echo '=== Release Examples ==='
        ./scripts/run_examples.sh
    fi
) 2>&1 | tee -a "$LOG_FILE"

ln -sf "verify_${TIMESTAMP}.log" "$PROJECT_ROOT/logs/verify.log"
echo "Verification complete! Logs at logs/verify_${TIMESTAMP}.log"
