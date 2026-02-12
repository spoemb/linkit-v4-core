#!/bin/bash
# =============================================================================
# LinkIt Build Environment Setup Script
# =============================================================================
# This script helps new users configure their build environment.
# It will:
#   1. Detect installed tools
#   2. Generate build_config.sh
#   3. Update .vscode/tasks.json
#   4. Verify the configuration
# =============================================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo -e "${BLUE}=========================================================================${NC}"
echo -e "${BLUE}LinkIt Build Environment Setup${NC}"
echo -e "${BLUE}=========================================================================${NC}"
echo ""

# -----------------------------------------------------------------------------
# Function to find ARM GCC toolchain
# -----------------------------------------------------------------------------
find_arm_gcc() {
    echo -e "${YELLOW}Searching for ARM GCC toolchain...${NC}"

    # Check if already in PATH
    if command -v arm-none-eabi-gcc &> /dev/null; then
        local gcc_path=$(which arm-none-eabi-gcc)
        echo -e "${GREEN}✓ Found in PATH: $gcc_path${NC}"
        dirname "$gcc_path"
        return 0
    fi

    # Search common locations
    local search_paths=(
        ~/tools/gcc-arm-none-eabi-*/bin
        /usr/local/gcc-arm-none-eabi-*/bin
        /opt/gcc-arm-none-eabi-*/bin
        /usr/bin
        /usr/local/bin
    )

    for path_pattern in "${search_paths[@]}"; do
        for path in $path_pattern; do
            if [ -d "$path" ] && [ -f "$path/arm-none-eabi-gcc" ]; then
                echo -e "${GREEN}✓ Found: $path${NC}"
                echo "$path"
                return 0
            fi
        done
    done

    echo -e "${RED}✗ Not found${NC}"
    return 1
}

# -----------------------------------------------------------------------------
# Function to find nrfutil
# -----------------------------------------------------------------------------
find_nrfutil() {
    echo -e "${YELLOW}Searching for nrfutil...${NC}"

    # Check if already in PATH
    if command -v nrfutil &> /dev/null; then
        local nrfutil_path=$(which nrfutil)
        echo -e "${GREEN}✓ Found in PATH: $nrfutil_path${NC}"
        dirname "$nrfutil_path"
        return 0
    fi

    # Search common locations
    local search_paths=(
        ~/.nrfutil/bin
        ~/.local/bin
        /usr/local/bin
        /usr/bin
    )

    for path in "${search_paths[@]}"; do
        if [ -f "$path/nrfutil" ]; then
            echo -e "${GREEN}✓ Found: $path${NC}"
            echo "$path"
            return 0
        fi
    done

    echo -e "${RED}✗ Not found${NC}"
    return 1
}

# -----------------------------------------------------------------------------
# Detect tools
# -----------------------------------------------------------------------------
echo -e "${BLUE}Step 1: Detecting build tools${NC}"
echo ""

ARM_TOOLCHAIN_PATH=$(find_arm_gcc)
ARM_FOUND=$?

echo ""
NRFUTIL_PATH=$(find_nrfutil)
NRFUTIL_FOUND=$?

echo ""

# -----------------------------------------------------------------------------
# Handle missing tools
# -----------------------------------------------------------------------------
if [ $ARM_FOUND -ne 0 ]; then
    echo -e "${RED}ARM GCC toolchain not found!${NC}"
    echo ""
    echo "Please install the ARM GCC toolchain:"
    echo "  - Download from: https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads"
    echo "  - Or install via package manager:"
    echo "    sudo apt-get install gcc-arm-none-eabi (may be outdated)"
    echo ""
    read -p "Enter the path to ARM GCC bin directory (or press Enter to skip): " manual_arm_path
    if [ -n "$manual_arm_path" ]; then
        ARM_TOOLCHAIN_PATH="$manual_arm_path"
    fi
fi

if [ $NRFUTIL_FOUND -ne 0 ]; then
    echo -e "${RED}nrfutil not found!${NC}"
    echo ""
    echo "Please install nrfutil:"
    echo "  - Download from: https://www.nordicsemi.com/Products/Development-tools/nRF-Util"
    echo "  - Or install via pip:"
    echo "    pip install nrfutil"
    echo ""
    read -p "Enter the path to nrfutil bin directory (or press Enter to skip): " manual_nrfutil_path
    if [ -n "$manual_nrfutil_path" ]; then
        NRFUTIL_PATH="$manual_nrfutil_path"
    fi
fi

# -----------------------------------------------------------------------------
# Generate build_config.sh
# -----------------------------------------------------------------------------
echo ""
echo -e "${BLUE}Step 2: Generating build_config.sh${NC}"
echo ""

cat > "$PROJECT_ROOT/build_config.sh" <<EOF
#!/bin/bash
# =============================================================================
# Build Configuration File
# =============================================================================
# Generated by setup_environment.sh
# You can manually edit this file if paths change.
# =============================================================================

# ARM GCC Toolchain path
ARM_TOOLCHAIN_PATH="$ARM_TOOLCHAIN_PATH"

# nrfutil path
NRFUTIL_PATH="$NRFUTIL_PATH"

# -----------------------------------------------------------------------------
# Build PATH construction
# -----------------------------------------------------------------------------
BUILD_PATH=""
if [ -n "\$ARM_TOOLCHAIN_PATH" ]; then
    BUILD_PATH="\$ARM_TOOLCHAIN_PATH"
fi
if [ -n "\$NRFUTIL_PATH" ]; then
    if [ -n "\$BUILD_PATH" ]; then
        BUILD_PATH="\$BUILD_PATH:\$NRFUTIL_PATH"
    else
        BUILD_PATH="\$NRFUTIL_PATH"
    fi
fi

if [ -n "\$BUILD_PATH" ]; then
    export PATH="\$BUILD_PATH:\$PATH"
fi

export ARM_TOOLCHAIN_PATH
export NRFUTIL_PATH
EOF

chmod +x "$PROJECT_ROOT/build_config.sh"
echo -e "${GREEN}✓ Created: build_config.sh${NC}"

# -----------------------------------------------------------------------------
# Update tasks.json
# -----------------------------------------------------------------------------
echo ""
echo -e "${BLUE}Step 3: Updating .vscode/tasks.json${NC}"
echo ""

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

if [ -f "$PROJECT_ROOT/.vscode/tasks.json" ]; then
    # Backup existing tasks.json
    cp "$PROJECT_ROOT/.vscode/tasks.json" "$PROJECT_ROOT/.vscode/tasks.json.backup"

    # Update PATH in tasks.json
    sed -i "s|\"PATH\": \"[^\"]*\"|\"PATH\": \"$TASKS_PATH:\${env:PATH}\"|" "$PROJECT_ROOT/.vscode/tasks.json"

    echo -e "${GREEN}✓ Updated: .vscode/tasks.json${NC}"
    echo -e "  Backup saved: .vscode/tasks.json.backup"
else
    echo -e "${YELLOW}⚠ .vscode/tasks.json not found, skipping${NC}"
fi

# -----------------------------------------------------------------------------
# Update Makefile.posix
# -----------------------------------------------------------------------------
echo ""
echo -e "${BLUE}Step 4: Updating SDK Makefile.posix${NC}"
echo ""

MAKEFILE_POSIX="$PROJECT_ROOT/ports/nrf52840/drivers/nRF5_SDK_17.0.2/components/toolchain/gcc/Makefile.posix"
if [ -f "$MAKEFILE_POSIX" ] && [ -n "$ARM_TOOLCHAIN_PATH" ]; then
    # Backup
    cp "$MAKEFILE_POSIX" "$MAKEFILE_POSIX.backup"

    # Get GCC version
    if [ -f "$ARM_TOOLCHAIN_PATH/arm-none-eabi-gcc" ]; then
        GCC_VERSION=$("$ARM_TOOLCHAIN_PATH/arm-none-eabi-gcc" -dumpversion)

        cat > "$MAKEFILE_POSIX" <<EOF
GNU_INSTALL_ROOT ?= $ARM_TOOLCHAIN_PATH/
GNU_VERSION ?= $GCC_VERSION
GNU_PREFIX ?= arm-none-eabi
EOF

        echo -e "${GREEN}✓ Updated: Makefile.posix${NC}"
        echo -e "  GCC Version: $GCC_VERSION"
    fi
else
    echo -e "${YELLOW}⚠ Makefile.posix not found or ARM toolchain path not set, skipping${NC}"
fi

# -----------------------------------------------------------------------------
# Verify configuration
# -----------------------------------------------------------------------------
echo ""
echo -e "${BLUE}Step 5: Verifying configuration${NC}"
echo ""

source "$PROJECT_ROOT/build_config.sh"

VERIFY_OK=true

if command -v arm-none-eabi-gcc &> /dev/null; then
    echo -e "${GREEN}✓ arm-none-eabi-gcc: $(which arm-none-eabi-gcc)${NC}"
else
    echo -e "${RED}✗ arm-none-eabi-gcc not found in PATH${NC}"
    VERIFY_OK=false
fi

if command -v nrfutil &> /dev/null; then
    echo -e "${GREEN}✓ nrfutil: $(which nrfutil)${NC}"
else
    echo -e "${RED}✗ nrfutil not found in PATH${NC}"
    VERIFY_OK=false
fi

if command -v mergehex &> /dev/null; then
    echo -e "${GREEN}✓ mergehex: $(which mergehex)${NC}"
else
    echo -e "${YELLOW}⚠ mergehex not found (optional, install: pip install intelhex)${NC}"
fi

if command -v make &> /dev/null; then
    echo -e "${GREEN}✓ make: $(which make)${NC}"
else
    echo -e "${RED}✗ make not found${NC}"
    VERIFY_OK=false
fi

# -----------------------------------------------------------------------------
# Summary
# -----------------------------------------------------------------------------
echo ""
echo -e "${BLUE}=========================================================================${NC}"
if [ "$VERIFY_OK" = true ]; then
    echo -e "${GREEN}✓ Setup complete! You can now build the project.${NC}"
    echo ""
    echo "To build:"
    echo "  ./scripts/build_core.sh"
    echo ""
    echo "Or use VSCode tasks:"
    echo "  - Bootloader: build"
    echo "  - Core: build"
else
    echo -e "${RED}⚠ Setup incomplete. Please fix the errors above.${NC}"
    echo ""
    echo "You can manually edit build_config.sh to set the correct paths."
fi
echo -e "${BLUE}=========================================================================${NC}"
