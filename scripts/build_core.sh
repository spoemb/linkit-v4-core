#!/bin/bash

mkdir -p ports/nrf52840/build/CORE
cd ports/nrf52840/build/CORE
git show-ref --tags -d | grep ^`git rev-parse HEAD` | sed -e "s,.* refs/tags/,," -e "s/\\^{}//" > TAG_NAME
if [ -z "$(cat TAG_NAME)"]; then
    git describe --dirty > TAG_NAME
fi
# Default to CAM if not set
EXT_GPIO5_DEVICE=${EXT_GPIO5_DEVICE:-CAM}
cmake -DCMAKE_TOOLCHAIN_FILE=../../toolchain_arm_gcc_nrf52.cmake -DDEBUG_LEVEL=4 -DBOARD=LINKIT -DCMAKE_BUILD_TYPE=Release -DMODEL=CORE -DHWVERS=4 -DEXT_GPIO5_DEVICE=${EXT_GPIO5_DEVICE} ../..
make -j 4
nrfutil settings generate --family NRF52840 --application LinkIt_CORE_board.hex --application-version 0 --bootloader-version 1 --bl-settings-version 2 --app-boot-validation VALIDATE_ECDSA_P256_SHA256 --sd-boot-validation VALIDATE_ECDSA_P256_SHA256 --softdevice ../../drivers/nRF5_SDK_17.0.2/components/softdevice/s140/hex/s140_nrf52_7.2.0_softdevice.hex --key-file ../../nrfutil_pkg_key.pem settings.hex
mergehex -m ../../bootloader/gentracker_secure_bootloader/gentracker_v1.0/armgcc/_build/cls_bootloader_v1_linkit_merged.hex LinkIt_CORE_board.hex -o m1.hex
mergehex -m m1.hex settings.hex -o LinkIt_CORE_board_merged.hex
rm -f m1.hex settings.hex
rm -f LinkIt_CORE_board-* LinkIt_CORE_board_dfu-* LinkIt_CORE_board_merged-*
mv LinkIt_CORE_board_dfu.zip LinkIt_CORE_board_dfu-`cat TAG_NAME`.zip
cp LinkIt_CORE_board.elf LinkIt_CORE_board-`cat TAG_NAME`.elf
mv LinkIt_CORE_board.hex LinkIt_CORE_board-`cat TAG_NAME`.hex
mv LinkIt_CORE_board.img LinkIt_CORE_board-`cat TAG_NAME`.img
mv LinkIt_CORE_board_merged.hex LinkIt_CORE_board_merged-`cat TAG_NAME`.hex