#!/bin/bash
# =============================================================================
# VSCode Tasks Path Generator
# =============================================================================
# This script generates the PATH string for VSCode tasks.json
# Run this script and copy the output to tasks.json
# =============================================================================

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Load build configuration
if [ -f "$PROJECT_ROOT/build_config.sh" ]; then
    source "$PROJECT_ROOT/build_config.sh"
else
    echo "ERROR: build_config.sh not found!"
    exit 1
fi

# Generate PATH string for tasks.json
TASKS_PATH=""
if [ -n "$ARM_TOOLCHAIN_PATH" ]; then
    TASKS_PATH="$ARM_TOOLCHAIN_PATH"
fi
if [ -n "$NRFUTIL_PATH" ]; then
    if [ -n "$TASKS_PATH" ]; then
        TASKS_PATH="$TASKS_PATH:$NRFUTIL_PATH"
    else
        TASKS_PATH="$NRFUTIL_PATH"
    fi
fi

echo "========================================================================="
echo "VSCode tasks.json PATH Configuration"
echo "========================================================================="
echo ""
echo "Copy the following line to .vscode/tasks.json:"
echo ""
echo "\"PATH\": \"$TASKS_PATH:\${env:PATH}\""
echo ""
echo "Full configuration block:"
echo "{"
echo "    \"version\": \"2.0.0\","
echo "    \"options\": {"
echo "        \"env\": {"
echo "            \"PATH\": \"$TASKS_PATH:\${env:PATH}\""
echo "        }"
echo "    },"
echo "    \"tasks\": ["
echo "        ..."
echo "    ]"
echo "}"
echo ""
echo "========================================================================="
