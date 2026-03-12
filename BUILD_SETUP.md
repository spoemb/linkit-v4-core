# Build Environment Setup

## Required Tools

The following tools must be installed and accessible in your PATH:

1. **ARM GCC Toolchain** (`arm-none-eabi-gcc`)
2. **nrfutil** (Nordic utility tool)
3. **mergehex** (Intel hex file merger - install with: `pip install intelhex`)
4. **make** (GNU Make)

## Configuration Files

If you need to update the toolchain paths, modify these files:

### 1. Build Script
**File:** `scripts/build_core.sh` (line 16)
```bash
export PATH="/home/schade/tools/gcc-arm-none-eabi-10-2020-q4-major/bin:/home/schade/.nrfutil/bin:$PATH"
```

### 2. VSCode Tasks
**File:** `.vscode/tasks.json` (line 14)
```json
"PATH": "/home/schade/tools/gcc-arm-none-eabi-10-2020-q4-major/bin:/home/schade/.nrfutil/bin:${env:PATH}"
```

### 3. Shell Configuration (Optional)
For terminal usage, update one of these:
- `~/.bashrc` (for bash)
- `~/.zshrc` (for zsh)

### 4. SDK Makefile
**File:** `ports/nrf52840/drivers/nRF5_SDK_17.0.2/components/toolchain/gcc/Makefile.posix`
```makefile
GNU_INSTALL_ROOT ?= /home/schade/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/
GNU_VERSION ?= 10.2.1
```

## Build Verification

The build script automatically verifies all required tools are available before building. If any tool is missing, you'll see an error message with instructions on which files to update.

### Example Success Output:
```
Checking build tools...
✓ arm-none-eabi-gcc found: /home/schade/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/arm-none-eabi-gcc
✓ nrfutil found: /home/schade/.nrfutil/bin/nrfutil
✓ mergehex found: /usr/local/bin/mergehex
```

### Example Error Output:
```
ERROR: arm-none-eabi-gcc not found in PATH!

Please update the PATH in one of these files:
  - scripts/build_core.sh (line 4)
  - .vscode/tasks.json (line 5)
```

## Build Options

The build script supports the following options:

- **CAM_ENABLE** (default: 1) - Enable camera support
- **BUZZER_ENABLE** (default: 0) - Enable buzzer support

To modify these, edit `scripts/build_core.sh` lines 13-14:
```bash
CAM_ENABLE=${CAM_ENABLE:-1}
BUZZER_ENABLE=${BUZZER_ENABLE:-0}
```

## Building

### From Command Line:
```bash
# Build bootloader first
cd ports/nrf52840/bootloader/secure_bootloader/linkitv4_v1.0/armgcc
make mergehex

# Then build core
cd ../../../../../..
./scripts/build_core.sh
```

### From VSCode:
1. **Bootloader: build** - Build the bootloader
2. **Core: build** - Build the core application
3. **Core: clean build** - Clean and rebuild

## Troubleshooting

If builds fail:

1. Check that all paths in the configuration files point to valid directories
2. Verify tools are executable: `arm-none-eabi-gcc --version`
3. Ensure the bootloader is built before building the core
4. Check the error messages - they will guide you to the correct files to update
