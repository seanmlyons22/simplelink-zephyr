# Copyright (c) 2026 Texas Instruments Incorporated
#
# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=CC2340R53")
board_runner_args(openocd --cmd-pre-init "source [find board/ti_lp_em_cc2340r53.cfg]")
# The TI OpenOCD fork is required (stock OpenOCD has no cc23xx/cc27xx flash
# driver). Point TI_OPENOCD_INSTALL_DIR at an extracted TI OpenOCD release
# (it contains openocd/bin/{bin,share}); otherwise fall back to `openocd`
# found in PATH instead of baking a nonexistent absolute path into runners.yaml.
if(DEFINED ENV{TI_OPENOCD_INSTALL_DIR})
  set(OPENOCD_BASE $ENV{TI_OPENOCD_INSTALL_DIR}/openocd/bin)
  set(OPENOCD ${OPENOCD_BASE}/bin/openocd)
  set(OPENOCD_DEFAULT_PATH ${OPENOCD_BASE}/share/openocd/scripts)
endif()
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
