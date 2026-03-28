#!/bin/bash

# =============================================================================
# Build with Bootloader (application + bootloader + SoftDevice → merged hex)
# =============================================================================
# Usage:
#   ./scripts/build_with_bootloader.sh <target> [options]
#
# Targets:
#   linkit-kim    LinkIt V4 KIM (UART, CLS KIM2)
#   linkit-smd    LinkIt V4 SMD (SPI, Arribada SMD)
#   linkit-lora   LinkIt V4 LoRa (UART, RAK3172-SiP)
#   rspb          RSPB bird tracker (SPI, SMD + mortality)
#
# Options:
#   --clean       Clean build (remove build dir first)
#   --debug       Build in Debug mode (default: Release with DEBUG_LEVEL=3)
#   --no-merge    Skip bootloader build and hex merge
#
# Examples:
#   ./scripts/build_release.sh rspb
#   ./scripts/build_release.sh linkit-smd --clean
#   ./scripts/build_release.sh linkit-kim --debug --no-merge
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Load build configuration
if [ -f "$PROJECT_ROOT/build_config.sh" ]; then
    source "$PROJECT_ROOT/build_config.sh"
else
    echo "ERROR: build_config.sh not found!"
    exit 1
fi

# Parse arguments
TARGET=""
CLEAN=false
DEBUG_MODE=false
NO_MERGE=false
RECOVER=false

for arg in "$@"; do
    case "$arg" in
        --clean)   CLEAN=true ;;
        --debug)   DEBUG_MODE=true ;;
        --no-merge) NO_MERGE=true ;;
        --recover) RECOVER=true ;;
        -*)        echo "Unknown option: $arg"; exit 1 ;;
        *)         TARGET="$arg" ;;
    esac
done

if [ "$RECOVER" = true ]; then
    echo "NOTE: --recover flag set — recover command will be shown in flash commands below"
    echo ""
fi

if [ -z "$TARGET" ]; then
    echo "Usage: $0 <target> [--clean] [--debug] [--no-merge] [--recover]"
    echo ""
    echo "Targets:"
    echo "  linkit-kim    LinkIt V4 KIM (UART)"
    echo "  linkit-smd    LinkIt V4 SMD (SPI)"
    echo "  linkit-lora   LinkIt V4 LoRa (RAK3172)"
    echo "  rspb          RSPB bird tracker"
    exit 1
fi

# Verify tools
for tool in arm-none-eabi-gcc nrfutil; do
    if ! command -v $tool &> /dev/null; then
        echo "ERROR: $tool not found in PATH"
        exit 1
    fi
done

echo "=== Release Build: $TARGET ==="
echo ""

# Configure per-target
case "$TARGET" in
    linkit-kim)
        BUILD_DIR="LINKIT"
        BOOTLOADER_DIR="linkitv4_v1.0"
        TARGET_NAME="LinkIt_board"
        CMAKE_ARGS="-DBOARD=LINKIT"
        BUILD_SCRIPT="build_linkitv4_kim.sh"
        ;;
    linkit-smd)
        BUILD_DIR="LINKIT_SMD"
        BOOTLOADER_DIR="linkitv4_v1.0"
        TARGET_NAME="LinkIt_board"
        CMAKE_ARGS="-DBOARD=LINKIT -DARGOS_SMD=ON -DENABLE_AXL_SENSOR=${ENABLE_AXL_SENSOR:-OFF} -DENABLE_SWS_LOG=${ENABLE_SWS_LOG:-ON}"
        BUILD_SCRIPT="build_linkitv4_smd.sh"
        ;;
    linkit-lora)
        BUILD_DIR="LINKIT_LORA"
        BOOTLOADER_DIR="linkitv4_v1.0"
        TARGET_NAME="LinkIt_board"
        CMAKE_ARGS="-DBOARD=LINKIT -DLORA_RAK3172=ON -DENABLE_AXL_SENSOR=${ENABLE_AXL_SENSOR:-ON}"
        BUILD_SCRIPT="build_linkitv4_lora.sh"
        ;;
    rspb)
        BUILD_DIR="RSPB"
        BOOTLOADER_DIR="rspbtracker_v1.0"
        TARGET_NAME="LinkIt_RSPB_board"
        CMAKE_ARGS="-DBOARD=RSPB -DARGOS_SMD=ON -DENABLE_PRESSURE_SENSOR=1 -DENABLE_AXL_SENSOR=1 -DENABLE_THERMISTOR_SENSOR=1 -DENABLE_MORTALITY_SENSOR=1"
        BUILD_SCRIPT="build_rspb.sh"
        ;;
    *)
        echo "ERROR: Unknown target '$TARGET'"
        exit 1
        ;;
esac

# Build type
if [ "$DEBUG_MODE" = true ]; then
    BUILD_TYPE="Debug"
    DEBUG_LEVEL=4
else
    BUILD_TYPE="Release"
    DEBUG_LEVEL=3
fi

BOOTLOADER_ARMGCC="$PROJECT_ROOT/ports/nrf52840/bootloader/secure_bootloader/$BOOTLOADER_DIR/armgcc"
BOOTLOADER_HEX_NAME=$(ls "$BOOTLOADER_ARMGCC/_build/"*_merged.hex 2>/dev/null | head -1)
SOFTDEVICE_HEX="$PROJECT_ROOT/ports/nrf52840/drivers/nRF5_SDK_17.0.2/components/softdevice/s140/hex/s140_nrf52_7.2.0_softdevice.hex"
KEY_FILE="$PROJECT_ROOT/ports/nrf52840/nrfutil_pkg_key.pem"
APP_BUILD_DIR="$PROJECT_ROOT/ports/nrf52840/build/$BUILD_DIR"

# =========================================================================
# Step 1: Build application
# =========================================================================
echo "--- Step 1: Build application ($BUILD_TYPE) ---"

if [ "$CLEAN" = true ]; then
    rm -rf "$APP_BUILD_DIR"
fi

mkdir -p "$APP_BUILD_DIR"
cd "$APP_BUILD_DIR"

# Version tag
git -C "$PROJECT_ROOT" show-ref --tags -d | grep ^$(git -C "$PROJECT_ROOT" rev-parse HEAD) | sed -e "s,.* refs/tags/,," -e "s/\\^{}//" > TAG_NAME
if [ -z "$(cat TAG_NAME)" ]; then
    git -C "$PROJECT_ROOT" describe --dirty > TAG_NAME
fi
VERSION=$(cat TAG_NAME)
echo "Version: $VERSION"

cmake -DCMAKE_TOOLCHAIN_FILE=../../toolchain_arm_gcc_nrf52.cmake \
      -DDEBUG_LEVEL=$DEBUG_LEVEL \
      -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
      $CMAKE_ARGS \
      ../..

make -j$(nproc)

if [ ! -f "${TARGET_NAME}.elf" ]; then
    echo "ERROR: Build failed — ${TARGET_NAME}.elf not found"
    exit 1
fi

# Generate hex if needed
if [ ! -f "${TARGET_NAME}.hex" ]; then
    arm-none-eabi-objcopy -O ihex ${TARGET_NAME}.elf ${TARGET_NAME}.hex
fi

echo ""
arm-none-eabi-size ${TARGET_NAME}.elf
echo ""
echo "✓ Application build OK"

# =========================================================================
# Step 2: Build bootloader (if needed)
# =========================================================================
if [ "$NO_MERGE" = true ]; then
    echo ""
    echo "--- Skipping bootloader and merge (--no-merge) ---"
else
    echo ""
    echo "--- Step 2: Build bootloader ($BOOTLOADER_DIR) ---"

    # Clean bootloader if --clean
    if [ "$CLEAN" = true ] && [ -d "$BOOTLOADER_ARMGCC/_build" ]; then
        echo "Cleaning bootloader build..."
        rm -rf "$BOOTLOADER_ARMGCC/_build"
    fi

    # Check if bootloader already exists
    BOOTLOADER_HEX_NAME=$(ls "$BOOTLOADER_ARMGCC/_build/"*_merged.hex 2>/dev/null | head -1)
    if [ -z "$BOOTLOADER_HEX_NAME" ]; then
        echo "Bootloader not found, building..."
        cd "$BOOTLOADER_ARMGCC"
        make mergehex
        cd "$APP_BUILD_DIR"
        BOOTLOADER_HEX_NAME=$(ls "$BOOTLOADER_ARMGCC/_build/"*_merged.hex 2>/dev/null | head -1)
    fi

    if [ -z "$BOOTLOADER_HEX_NAME" ]; then
        echo "ERROR: Bootloader build failed"
        exit 1
    fi
    echo "✓ Bootloader: $(basename $BOOTLOADER_HEX_NAME)"

    # =========================================================================
    # Step 3: Generate settings + merge
    # =========================================================================
    echo ""
    echo "--- Step 3: Generate merged hex ---"

    if [ ! -f "$KEY_FILE" ]; then
        echo "ERROR: nrfutil key file not found: $KEY_FILE"
        exit 1
    fi

    # Generate nrfutil settings
    nrfutil settings generate --family NRF52840 \
        --application ${TARGET_NAME}.hex \
        --application-version 0 \
        --bootloader-version 1 \
        --bl-settings-version 2 \
        --app-boot-validation VALIDATE_ECDSA_P256_SHA256 \
        --sd-boot-validation VALIDATE_ECDSA_P256_SHA256 \
        --softdevice "$SOFTDEVICE_HEX" \
        --key-file "$KEY_FILE" \
        settings.hex

    # Merge: bootloader + app → m1.hex, then m1 + settings → merged
    mergehex -m "$BOOTLOADER_HEX_NAME" ${TARGET_NAME}.hex -o m1.hex
    mergehex -m m1.hex settings.hex -o ${TARGET_NAME}_merged.hex
    # App + settings (for app-only flash without re-flashing bootloader)
    mergehex -m ${TARGET_NAME}.hex settings.hex -o ${TARGET_NAME}_app_settings.hex
    rm -f m1.hex settings.hex

    echo "✓ Merged hex generated"
fi

# =========================================================================
# Step 4: Rename with version tag
# =========================================================================
echo ""
echo "--- Step 4: Tag output files ($VERSION) ---"

rm -f ${TARGET_NAME}-* ${TARGET_NAME}_dfu-* ${TARGET_NAME}_merged-*

cp ${TARGET_NAME}.elf ${TARGET_NAME}-${VERSION}.elf
cp ${TARGET_NAME}.hex ${TARGET_NAME}-${VERSION}.hex

if [ -f "${TARGET_NAME}_dfu.zip" ]; then
    cp ${TARGET_NAME}_dfu.zip ${TARGET_NAME}_dfu-${VERSION}.zip
fi
if [ -f "${TARGET_NAME}.img" ]; then
    cp ${TARGET_NAME}.img ${TARGET_NAME}-${VERSION}.img
fi
if [ -f "${TARGET_NAME}_merged.hex" ]; then
    cp ${TARGET_NAME}_merged.hex ${TARGET_NAME}_merged-${VERSION}.hex
fi
if [ -f "${TARGET_NAME}_app_settings.hex" ]; then
    cp ${TARGET_NAME}_app_settings.hex ${TARGET_NAME}_app_settings-${VERSION}.hex
fi

# =========================================================================
# Summary
# =========================================================================
echo ""
echo "========================================="
echo "  BUILD COMPLETE: $TARGET ($BUILD_TYPE)"
echo "  Version: $VERSION"
echo "========================================="
echo ""
echo "Output: $APP_BUILD_DIR/"
echo ""
ls -lh ${TARGET_NAME}-${VERSION}.* 2>/dev/null || true
ls -lh ${TARGET_NAME}_dfu-${VERSION}.* 2>/dev/null || true
ls -lh ${TARGET_NAME}_merged-${VERSION}.* 2>/dev/null || true
ls -lh ${TARGET_NAME}_app_settings-${VERSION}.* 2>/dev/null || true
echo ""

FLASH_DIR="ports/nrf52840/build/${BUILD_DIR}"
echo "Flash commands:"
echo "  NOTE: If flashing fails with 'Access protection enabled', the device has"
echo "  readback protection (APPROTECT) active. Run 'nrfjprog --recover' first."
echo "  This erases ALL flash (firmware + config) and disables protection."
echo ""
if [ "$RECOVER" = true ]; then
    echo "  Recover (erase ALL flash + disable APPROTECT — run BEFORE flashing):"
    echo "    nrfjprog --recover -f nrf52"
    echo ""
fi
if [ -f "${TARGET_NAME}_merged-${VERSION}.hex" ]; then
    echo "  Full (app + bootloader + softdevice):"
    echo "    nrfjprog --program ${FLASH_DIR}/${TARGET_NAME}_merged-${VERSION}.hex --chiperase --verify --reset"
fi
if [ -f "${TARGET_NAME}_app_settings-${VERSION}.hex" ]; then
    echo "  App only (bootloader + softdevice must already be flashed):"
    echo "    nrfjprog --program ${FLASH_DIR}/${TARGET_NAME}_app_settings-${VERSION}.hex --sectorerase --verify --reset"
fi
echo ""
