# Copyright (c) 2025 Texas Instruments Incorporated
# Copyright (c) 2024 BayLibre, SAS
#
# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=CC2745R10-Q1")
board_runner_args(openocd --cmd-pre-init "source [find board/ti_lp_em_cc2745r10.cfg]")

# Use the TI OpenOCD fork (installed under /usr/local). It ships the cc27xx NOR
# flash driver + SACI reset and the ti_lp_em_cc2745r10.cfg board file. Pin the
# binary and script path explicitly so this build always uses it, regardless of
# PATH order or any openocd bundled with the Zephyr SDK.
set(OPENOCD /usr/local/bin/openocd)
set(OPENOCD_DEFAULT_PATH /usr/local/share/openocd/scripts)

# Make openocd the default flasher/debugger over XDS110 (J-Link stays available
# for anyone who has one: `west flash -r jlink`).
board_set_flasher_ifnset(openocd)
board_set_debugger_ifnset(openocd)

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
