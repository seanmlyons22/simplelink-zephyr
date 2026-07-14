# Copyright (c) 2024 Texas Instruments Incorporated
# Copyright (c) 2024 BayLibre, SAS
#
# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=CC2340R5")
board_runner_args(openocd --cmd-pre-init "source [find board/ti_lp_em_cc2340r5.cfg]")

# Use the TI OpenOCD fork (built from ~/git/ti-openocd, installed under /usr/local). It provides
# the cc23xx NOR flash driver + SACI reset handling that upstream/SDK OpenOCD lacks. Pin the
# binary and its script path explicitly so this build always uses it, regardless of PATH order
# or any openocd bundled with the Zephyr SDK.
set(OPENOCD /usr/local/bin/openocd)
set(OPENOCD_DEFAULT_PATH /usr/local/share/openocd/scripts)

# OpenOCD can now flash cc23x0 -> make it the default flasher/debugger (J-Link stays available
# for anyone who has one: `west flash -r jlink`).
board_set_flasher_ifnset(openocd)
board_set_debugger_ifnset(openocd)

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
