#!/bin/bash
# Run turtle simulation tests with HTML report generation

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/tests/build"
REPORT_DIR="$PROJECT_DIR/tests/reports"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}    🐢 Turtle Simulation Runner${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""

# Build if needed
if [ ! -f "$BUILD_DIR/TurtleSimulation" ]; then
    echo -e "${YELLOW}Building turtle simulation...${NC}"
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"
    rm -f CMakeCache.txt
    cmake -GNinja ..
    ninja TurtleSimulation
    echo ""
fi

# Create reports directory
mkdir -p "$REPORT_DIR"

# Generate timestamp for report
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
REPORT_FILE="$REPORT_DIR/turtle_simulation_${TIMESTAMP}.html"

cd "$BUILD_DIR"

# Set environment variable for report path
export TURTLE_REPORT_PATH="$REPORT_FILE"

# Run simulation
echo -e "${CYAN}Running turtle simulation...${NC}"
echo ""

set +e
./TurtleSimulation -v
EXIT_CODE=$?
set -e

# Copy latest report as current
if [ -f "$REPORT_FILE" ]; then
    cp "$REPORT_FILE" "$REPORT_DIR/turtle_simulation_latest.html"
    echo ""
    echo -e "${GREEN}HTML Report saved to:${NC}"
    echo -e "  ${YELLOW}$REPORT_FILE${NC}"
    echo -e "  ${YELLOW}$REPORT_DIR/turtle_simulation_latest.html${NC}"
fi

exit $EXIT_CODE
