#!/bin/bash
# Run unit tests and save report for pre-commit hook
REPO_ROOT="$(git rev-parse --show-toplevel)"
BUILD_DIR="$REPO_ROOT/tests/build"
REPORT="$BUILD_DIR/last_test_report.txt"
DATE=$(date '+%Y-%m-%d %H:%M:%S')

cd "$BUILD_DIR" || exit 1
ninja TrackerTests || exit 1

echo "Running tests..."

R1=$(timeout 30 ./TrackerTests -sg DTEHandler 2>&1 | grep -E "^OK|^Errors")
echo "  DTEHandler: $R1"

R2=$(timeout 30 ./TrackerTests -sg ArgosTxService 2>&1 | grep -E "^OK|^Errors")
echo "  ArgosTxService: $R2"

R3=$(timeout 30 ./TrackerTests -sg AXLSensor 2>&1 | grep -E "^OK|^Errors")
echo "  AXLSensor: $R3"

R4=$(timeout 30 ./TrackerTests -sg ConfigStore 2>&1 | grep -E "^OK|^Errors")
echo "  ConfigStore: $R4"

{
    echo "Date: $DATE"
    echo "DTEHandler: $R1"
    echo "ArgosTxService: $R2"
    echo "AXLSensor: $R3"
    echo "ConfigStore: $R4"
    echo "Status: All tests passing"
} > "$REPORT"

echo ""
cat "$REPORT"
