#!/bin/bash
# =============================================================================
# janus_audit.sh - Janus Compliance Audit for Icarus
# =============================================================================
#
# Scans the Icarus codebase for potential Janus violations:
# 1. std:: math functions (should use janus:: versions)
# 2. if/else branching on Scalar types (should use janus::where)
# 3. while loops with dynamic bounds (should use structural for loops)
# 4. Hardcoded 'double' in template functions
#
# Usage: ./scripts/janus_audit.sh
#
# Exit codes:
#   0 - No violations found
#   1 - Violations found
#
# =============================================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT"

VIOLATIONS_FOUND=0

echo "=============================================="
echo "       Janus Compliance Audit for Icarus"
echo "=============================================="

# =============================================================================
# Check 1: std:: math functions
# =============================================================================
echo -e "\n${YELLOW}[1/4] Checking for std:: math usage...${NC}"
echo "      (Should use janus:: equivalents for Scalar compatibility)"

# List of math functions that should use janus:: versions
MATH_FUNCS="sin|cos|tan|asin|acos|atan|atan2|sinh|cosh|tanh|asinh|acosh|atanh|sqrt|pow|exp|log|log10|log2|abs|fabs|fmod|floor|ceil|round|fmax|fmin|hypot"

# Exclude test files and known safe locations
EXCLUDE_PATTERN="--exclude-dir=.git --exclude-dir=build --exclude-dir=references --exclude=*_test.cpp"

if grep -rn $EXCLUDE_PATTERN --include="*.hpp" --include="*.cpp" \
    -E "std::($MATH_FUNCS)\s*\(" \
    include/ src/ components/ 2>/dev/null; then
    echo -e "${RED}  FAIL: Found std:: math functions${NC}"
    VIOLATIONS_FOUND=1
else
    echo -e "${GREEN}  PASS: No std:: math functions found${NC}"
fi

# =============================================================================
# Check 2: if/else on Scalar types (heuristic)
# =============================================================================
echo -e "\n${YELLOW}[2/4] Checking for potential if/else on Scalar...${NC}"
echo "      (Should use janus::where for branching on Scalar values)"

# This is a heuristic - look for patterns like "if (x > 0)" where x could be Scalar
# We look for comparisons involving common variable patterns
if grep -rn $EXCLUDE_PATTERN --include="*.hpp" --include="*.cpp" \
    -E "if\s*\(\s*[a-zA-Z_][a-zA-Z0-9_]*\s*(>|<|>=|<=|==|!=)\s*(0|0\.0|Scalar)" \
    include/ src/ components/ 2>/dev/null | \
    grep -v "size\|count\|index\|phase\|mode\|flag\|state_\|Phase::\|if constexpr" | head -20; then
    echo -e "${YELLOW}  WARNING: Potential if/else on Scalar (false positives possible)${NC}"
    echo "           Review the above lines manually for Janus compliance"
else
    echo -e "${GREEN}  PASS: No obvious if/Scalar patterns found${NC}"
fi

# =============================================================================
# Check 3: while loops with dynamic bounds
# =============================================================================
echo -e "\n${YELLOW}[3/4] Checking for while loops with dynamic bounds...${NC}"
echo "      (Loops must have structural bounds - use for loops with int bounds)"

# Look for while loops that compare against variables (not constants)
# Pattern: while (variable comparison something)
if grep -rn $EXCLUDE_PATTERN --include="*.hpp" --include="*.cpp" \
    -E "while\s*\(\s*[a-zA-Z_][a-zA-Z0-9_]*\s*(>|<|>=|<=|!=)" \
    include/ src/ components/ 2>/dev/null | \
    grep -v "iterator\|it\|ptr\|pos\|cursor\|nullptr\|true\|false" | head -10; then
    echo -e "${YELLOW}  WARNING: Potential while loops with dynamic bounds (false positives possible)${NC}"
    echo "           Review the above lines - while loops break symbolic tracing"
else
    echo -e "${GREEN}  PASS: No obvious while loops with dynamic bounds${NC}"
fi

# =============================================================================
# Check 4: Hardcoded double in templates
# =============================================================================
echo -e "\n${YELLOW}[4/4] Checking for hardcoded double in template functions...${NC}"
echo "      (Template functions should use Scalar, not double)"

# Look for function returns or parameters using 'double' in template contexts
if grep -rn $EXCLUDE_PATTERN --include="*.hpp" --include="*.cpp" \
    -B2 "template.*Scalar" | \
    grep -E "^\s*(double|float)\s+[a-zA-Z_]" | head -10; then
    echo -e "${YELLOW}  WARNING: Found hardcoded double near template declarations${NC}"
    echo "           Review the above lines manually"
else
    echo -e "${GREEN}  PASS: No obvious hardcoded double in templates${NC}"
fi

# =============================================================================
# Summary
# =============================================================================
echo -e "\n=============================================="
if [ $VIOLATIONS_FOUND -eq 0 ]; then
    echo -e "${GREEN}Audit Complete: No violations found${NC}"
    exit 0
else
    echo -e "${RED}Audit Complete: Violations found - please fix before merge${NC}"
    exit 1
fi

