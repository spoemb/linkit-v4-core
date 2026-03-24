#!/bin/bash

# =============================================================================
# LinkIt Core Build Script
# =============================================================================
# This script builds the LinkIt Core firmware.
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

mkdir -p ports/nrf52840/build/LINKIT
cd ports/nrf52840/build/LINKIT
git show-ref --tags -d | grep ^`git rev-parse HEAD` | sed -e "s,.* refs/tags/,," -e "s/\\^{}//" > TAG_NAME
if [ -z "$(cat TAG_NAME)" ]; then
    git describe --dirty > TAG_NAME
fi
# Default to both disabled (0)
CAM_ENABLE=${CAM_ENABLE:-1}
BUZZER_ENABLE=${BUZZER_ENABLE:-0}
cmake -DCMAKE_TOOLCHAIN_FILE=../../toolchain_arm_gcc_nrf52.cmake -DDEBUG_LEVEL=3 -DBOARD=LINKIT -DCMAKE_BUILD_TYPE=Release -DCAM_ENABLE=${CAM_ENABLE} -DBUZZER_ENABLE=${BUZZER_ENABLE} ../..
make -j 20
nrfutil settings generate --family NRF52840 --application LinkIt_board.hex --application-version 0 --bootloader-version 1 --bl-settings-version 2 --app-boot-validation VALIDATE_ECDSA_P256_SHA256 --sd-boot-validation VALIDATE_ECDSA_P256_SHA256 --softdevice ../../drivers/nRF5_SDK_17.0.2/components/softdevice/s140/hex/s140_nrf52_7.2.0_softdevice.hex --key-file ../../nrfutil_pkg_key.pem settings.hex
mergehex -m ../../bootloader/secure_bootloader/linkitv4_v1.0/armgcc/_build/cls_bootloader_v1_linkit_merged.hex LinkIt_board.hex -o m1.hex
mergehex -m m1.hex settings.hex -o LinkIt_board_merged.hex
mergehex -m LinkIt_board.hex settings.hex -o LinkIt_board_app_settings.hex
rm -f m1.hex settings.hex
rm -f LinkIt_board-* LinkIt_board_dfu-* LinkIt_board_merged-*
mv LinkIt_board_dfu.zip LinkIt_board_dfu-`cat TAG_NAME`.zip
cp LinkIt_board.elf LinkIt_board-`cat TAG_NAME`.elf
mv LinkIt_board.hex LinkIt_board-`cat TAG_NAME`.hex
mv LinkIt_board.img LinkIt_board-`cat TAG_NAME`.img
mv LinkIt_board_merged.hex LinkIt_board_merged-`cat TAG_NAME`.hex
if [ -f "LinkIt_board_app_settings.hex" ]; then
    mv LinkIt_board_app_settings.hex LinkIt_board_app_settings-`cat TAG_NAME`.hex
fi

# Show flash command
TAG=$(cat TAG_NAME)
BUILD_DIR="ports/nrf52840/build/LINKIT"
echo ""
echo "Flash commands:"
if [ -f "LinkIt_board_merged-${TAG}.hex" ]; then
    echo "  Full (app + bootloader + softdevice):"
    echo "    nrfjprog --program ${BUILD_DIR}/LinkIt_board_merged-${TAG}.hex --chiperase --verify --reset"
fi
if [ -f "LinkIt_board_app_settings-${TAG}.hex" ]; then
    echo "  App only (bootloader + softdevice must already be flashed):"
    echo "    nrfjprog --program ${BUILD_DIR}/LinkIt_board_app_settings-${TAG}.hex --sectorerase --verify --reset"
fi