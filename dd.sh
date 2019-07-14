#!/bin/bash

TARGET_BOARD=lpc11u35
BIN_FILE=firmware.bin

BIN_PATH=./.pioenvs/${TARGET_BOARD}/${BIN_FILE}

# compile
#echo "step 1: compile"
#mbed compile -t gcc_arm -m ${TARGET_BOARD}

# add checksum
#echo "step 2: add checksum"
#lpc_checksum ${BIN_PATH}

# dd
echo "dd if=${BIN_PATH} of=/media/iwait/CRP\ DISABLD/firmware.bin"
dd if=${BIN_PATH} of=/media/iwait/CRP\ DISABLD/firmware.bin conv=notrunc