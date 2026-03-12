#!/bin/bash

# =============================================================================
# RSPB Core Build Script
# =============================================================================
# This script builds the RSPB Core firmware with SMD satellite module support.
#
# IMPORTANT: Update build_config.sh with your toolchain paths
# =============================================================================

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Load build configuration
if [ -f "$PROJECT_ROOT/build_config.sh" ]; then
    source "$PROJECT_ROOT/build_config.sh"
else
    echo "ERROR: build_config.sh not found!"
    echo "Please create build_config.sh in the project root."
    echo "You can copy build_config.sh.example as a template."
    exit 1
fi

# Verify required tools are available
echo "Checking build tools..."

if ! command -v arm-none-eabi-gcc &> /dev/null; then
    echo "ERROR: arm-none-eabi-gcc not found in PATH!"
    echo ""
    echo "Please update build_config.sh with the correct path:"
    echo "  ARM_TOOLCHAIN_PATH=\"/path/to/your/gcc-arm-none-eabi/bin\""
    echo ""
    echo "Current configuration:"
    echo "  ARM_TOOLCHAIN_PATH=\"$ARM_TOOLCHAIN_PATH\""
    echo ""
    exit 1
fi

if ! command -v nrfutil &> /dev/null; then
    echo "ERROR: nrfutil not found in PATH!"
    echo ""
    echo "Please update build_config.sh with the correct path:"
    echo "  NRFUTIL_PATH=\"/path/to/your/nrfutil/bin\""
    echo ""
    echo "Current configuration:"
    echo "  NRFUTIL_PATH=\"$NRFUTIL_PATH\""
    echo ""
    exit 1
fi

if ! command -v mergehex &> /dev/null; then
    echo "WARNING: mergehex not found in PATH!"
    echo "The build may fail during hex file merging."
    echo "Install mergehex: pip install intelhex"
    echo ""
fi

if ! command -v make &> /dev/null; then
    echo "ERROR: make not found in PATH!"
    echo "Please install build-essential: sudo apt-get install build-essential"
    exit 1
fi

echo "✓ arm-none-eabi-gcc found: $(which arm-none-eabi-gcc)"
echo "✓ nrfutil found: $(which nrfutil)"
if command -v mergehex &> /dev/null; then
    echo "✓ mergehex found: $(which mergehex)"
fi
echo ""

mkdir -p ports/nrf52840/build/RSPB
cd ports/nrf52840/build/RSPB
git show-ref --tags -d | grep ^`git rev-parse HEAD` | sed -e "s,.* refs/tags/,," -e "s/\\^{}//" > TAG_NAME
if [ -z "$(cat TAG_NAME)" ]; then
    git describe --dirty > TAG_NAME
fi

# RSPB build configuration
# - ARGOS_SMD=ON: Use SMD satellite module (default for RSPB)
# - ENABLE_PRESSURE_SENSOR=ON: Enable LPS28DFW pressure sensor
# - ENABLE_AXL_SENSOR=ON: Enable BMA400 accelerometer
# - ENABLE_THERMISTOR_SENSOR is automatically enabled for RSPB in CMakeLists.txt
ARGOS_SMD=${ARGOS_SMD:-ON}
ENABLE_PRESSURE_SENSOR=${ENABLE_PRESSURE_SENSOR:-ON}
ENABLE_AXL_SENSOR=${ENABLE_AXL_SENSOR:-ON}

echo "Building RSPB with configuration:"
echo "  ARGOS_SMD=${ARGOS_SMD}"
echo "  ENABLE_PRESSURE_SENSOR=${ENABLE_PRESSURE_SENSOR}"
echo "  ENABLE_AXL_SENSOR=${ENABLE_AXL_SENSOR}"
echo ""

cmake -DCMAKE_TOOLCHAIN_FILE=../../toolchain_arm_gcc_nrf52.cmake \
      -DDEBUG_LEVEL=4 \
      -DBOARD=RSPB \
      -DCMAKE_BUILD_TYPE=Release \
      -DARGOS_SMD=${ARGOS_SMD} \
      -DENABLE_PRESSURE_SENSOR=${ENABLE_PRESSURE_SENSOR} \
      -DENABLE_AXL_SENSOR=${ENABLE_AXL_SENSOR} \
      ../..

make -j 20

# Check if build succeeded (check elf, not hex - hex might not exist if build was up-to-date)
if [ ! -f "LinkIt_RSPB_board.elf" ]; then
    echo "ERROR: Build failed - LinkIt_RSPB_board.elf not found"
    exit 1
fi

# Generate hex file if it doesn't exist (POST_BUILD doesn't run if target is up-to-date)
if [ ! -f "LinkIt_RSPB_board.hex" ]; then
    echo "Generating hex file..."
    arm-none-eabi-objcopy -O ihex LinkIt_RSPB_board.elf LinkIt_RSPB_board.hex
fi

echo ""
echo "✓ Build succeeded!"
echo ""

# Bootloader path
BOOTLOADER_HEX="../../bootloader/secure_bootloader/rspbtracker_v1.0/armgcc/_build/rspb_bootloader_v1_linkit_merged.hex"
SOFTDEVICE_HEX="../../drivers/nRF5_SDK_17.0.2/components/softdevice/s140/hex/s140_nrf52_7.2.0_softdevice.hex"
KEY_FILE="../../nrfutil_pkg_key.pem"

# Check if bootloader and required files exist for merged hex generation
CAN_MERGE=true

if [ ! -f "$BOOTLOADER_HEX" ]; then
    echo "WARNING: Bootloader not found at: $BOOTLOADER_HEX"
    echo "         Skipping merged hex generation."
    echo "         To build bootloader, run: scripts/build_rspb_bootloader.sh"
    CAN_MERGE=false
fi

if [ ! -f "$KEY_FILE" ]; then
    echo "WARNING: nrfutil key file not found at: $KEY_FILE"
    echo "         Skipping merged hex generation."
    CAN_MERGE=false
fi

if [ "$CAN_MERGE" = true ]; then
    echo "Generating settings and merged hex files..."

    # Generate settings
    nrfutil settings generate --family NRF52840 \
        --application LinkIt_RSPB_board.hex \
        --application-version 0 \
        --bootloader-version 1 \
        --bl-settings-version 2 \
        --app-boot-validation VALIDATE_ECDSA_P256_SHA256 \
        --sd-boot-validation VALIDATE_ECDSA_P256_SHA256 \
        --softdevice "$SOFTDEVICE_HEX" \
        --key-file "$KEY_FILE" \
        settings.hex

    if [ $? -eq 0 ]; then
        mergehex -m "$BOOTLOADER_HEX" LinkIt_RSPB_board.hex -o m1.hex
        mergehex -m m1.hex settings.hex -o LinkIt_RSPB_board_merged.hex
        rm -f m1.hex settings.hex
        echo "✓ Merged hex generated"
    else
        echo "WARNING: nrfutil settings generation failed"
        CAN_MERGE=false
    fi
fi

# Rename output files with version tag
rm -f LinkIt_RSPB_board-* LinkIt_RSPB_board_dfu-* LinkIt_RSPB_board_merged-*

# Application files (always available after successful build)
cp LinkIt_RSPB_board.elf LinkIt_RSPB_board-`cat TAG_NAME`.elf
mv LinkIt_RSPB_board.hex LinkIt_RSPB_board-`cat TAG_NAME`.hex

# Optional files (may not exist)
if [ -f "LinkIt_RSPB_board_dfu.zip" ]; then
    mv LinkIt_RSPB_board_dfu.zip LinkIt_RSPB_board_dfu-`cat TAG_NAME`.zip
fi
if [ -f "LinkIt_RSPB_board.img" ]; then
    mv LinkIt_RSPB_board.img LinkIt_RSPB_board-`cat TAG_NAME`.img
fi
if [ -f "LinkIt_RSPB_board_merged.hex" ]; then
    mv LinkIt_RSPB_board_merged.hex LinkIt_RSPB_board_merged-`cat TAG_NAME`.hex
fi

echo ""
echo "Build complete!"
echo "Output files in: ports/nrf52840/build/RSPB/"
echo ""
echo "Files generated:"
ls -la LinkIt_RSPB_board-* 2>/dev/null || true
ls -la LinkIt_RSPB_board_dfu-* 2>/dev/null || true
ls -la LinkIt_RSPB_board_merged-* 2>/dev/null || true

if [ "$CAN_MERGE" = false ]; then
    echo ""
    echo "NOTE: Merged hex was not generated (see warnings above)"
fi
