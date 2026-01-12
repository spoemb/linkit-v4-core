# LinkIt Build System - Quick Start Guide

Welcome! This guide will help you set up your build environment quickly and easily.

## 🚀 Quick Setup (Recommended for New Users)

Run the automated setup script:

```bash
./scripts/setup_environment.sh
```

This script will:
- ✓ Auto-detect your installed toolchains
- ✓ Generate `build_config.sh` with your paths
- ✓ Update VSCode tasks configuration
- ✓ Update SDK Makefile
- ✓ Verify everything is working

## 🔧 Manual Setup

If you prefer to configure manually:

### Step 1: Install Required Tools

You need these tools installed on your system:

1. **ARM GCC Toolchain** (`arm-none-eabi-gcc`)
   - Download from: https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads
   - Recommended version: 10.2.1 or newer

2. **nrfutil** (Nordic utility)
   - Download from: https://www.nordicsemi.com/Products/Development-tools/nRF-Util
   - Or install via pip: `pip install nrfutil`

3. **mergehex** (Intel hex merger)
   - Install via pip: `pip install intelhex`

4. **make** (GNU Make)
   - Linux: `sudo apt-get install build-essential`
   - macOS: Included with Xcode Command Line Tools

### Step 2: Configure Build Paths

Edit `build_config.sh` and set your toolchain paths:

```bash
# Example for Linux
ARM_TOOLCHAIN_PATH="/home/username/tools/gcc-arm-none-eabi-10-2020-q4-major/bin"
NRFUTIL_PATH="/home/username/.nrfutil/bin"

# Example for macOS
ARM_TOOLCHAIN_PATH="/usr/local/gcc-arm-none-eabi/bin"
NRFUTIL_PATH="/usr/local/bin"

# Example for Windows (WSL)
ARM_TOOLCHAIN_PATH="/mnt/c/Program Files/GNU Arm Embedded Toolchain/bin"
NRFUTIL_PATH="/home/username/.nrfutil/bin"
```

**Tip:** The setup script can auto-detect tools in common locations!

### Step 3: Update VSCode Tasks (Optional)

If using VSCode, run:

```bash
./scripts/setup_vscode_tasks.sh
```

Copy the generated PATH to `.vscode/tasks.json`.

### Step 4: Verify Configuration

Test your configuration:

```bash
./scripts/build_core.sh
```

You should see:
```
Checking build tools...
✓ arm-none-eabi-gcc found: /path/to/arm-none-eabi-gcc
✓ nrfutil found: /path/to/nrfutil
✓ mergehex found: /path/to/mergehex
```

## 📁 File Structure

```
linkit-v4-core/
├── build_config.sh          # Main configuration file (edit this!)
├── scripts/
│   ├── setup_environment.sh # Automated setup script
│   ├── build_core.sh        # Core build script
│   └── setup_vscode_tasks.sh # VSCode helper
├── .vscode/
│   └── tasks.json           # VSCode build tasks
└── ports/nrf52840/drivers/nRF5_SDK_17.0.2/components/toolchain/gcc/
    └── Makefile.posix       # SDK toolchain config
```

## 🏗️ Building

### Command Line

```bash
# Build bootloader (required first time)
cd ports/nrf52840/bootloader/gentracker_secure_bootloader/gentracker_v1.0/armgcc
make mergehex
cd ../../../../../..

# Build core
./scripts/build_core.sh
```

### VSCode Tasks

1. Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on macOS)
2. Type "Tasks: Run Task"
3. Select:
   - **Bootloader: build** (first time only)
   - **Core: build** (main build)

## ⚙️ Build Options

Edit `scripts/build_core.sh` to customize:

```bash
CAM_ENABLE=${CAM_ENABLE:-1}      # Camera support (1=enabled, 0=disabled)
BUZZER_ENABLE=${BUZZER_ENABLE:-0} # Buzzer support (1=enabled, 0=disabled)
```

Or set environment variables before building:

```bash
CAM_ENABLE=0 ./scripts/build_core.sh
```

## 🐛 Troubleshooting

### Problem: "arm-none-eabi-gcc not found"

**Solution:** Update `build_config.sh` with the correct path to your ARM toolchain:
```bash
ARM_TOOLCHAIN_PATH="/path/to/your/gcc-arm-none-eabi/bin"
```

### Problem: "nrfutil not found"

**Solution:** Install nrfutil or update the path in `build_config.sh`:
```bash
# Install
pip install nrfutil

# Or set path
NRFUTIL_PATH="/path/to/your/nrfutil/bin"
```

### Problem: Build fails in VSCode but works in terminal

**Solution:** VSCode tasks might not have the correct PATH. Run:
```bash
./scripts/setup_vscode_tasks.sh
```

Then update `.vscode/tasks.json` with the generated PATH.

### Problem: "mergehex: command not found"

**Solution:** Install intelhex:
```bash
pip install intelhex
```

### Problem: Wrong GCC version in SDK

**Solution:** The setup script automatically updates this, but you can manually edit:
```
ports/nrf52840/drivers/nRF5_SDK_17.0.2/components/toolchain/gcc/Makefile.posix
```

## 📚 Additional Resources

- **Full Documentation:** See `BUILD_SETUP.md` for detailed information
- **ARM Toolchain:** https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads
- **nRF5 SDK:** https://www.nordicsemi.com/Products/Development-software/nrf5-sdk
- **nrfutil:** https://www.nordicsemi.com/Products/Development-tools/nRF-Util

## 🆘 Need Help?

If you encounter issues:

1. Run `./scripts/setup_environment.sh` to reconfigure
2. Check that all paths in `build_config.sh` are correct
3. Verify tools are installed: `arm-none-eabi-gcc --version`, `nrfutil --version`
4. Check the error messages - they will guide you to the correct file to update

---

**Happy Building! 🎉**
