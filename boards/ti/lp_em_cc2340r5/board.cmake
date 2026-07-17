# Copyright (c) 2024 Texas Instruments Incorporated
# Copyright (c) 2024 BayLibre, SAS
#
# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=CC2340R5")
board_runner_args(openocd --cmd-pre-init "source [find board/ti_lp_em_cc2340r5.cfg]")

# cc23xx needs the TI OpenOCD fork (cc23xx NOR flash driver + SACI reset) that
# upstream and Zephyr SDK OpenOCD lack. Resolve it portably:
# - TI_OPENOCD_INSTALL_DIR, when set, names the fork's install prefix (binary
#   at <prefix>/bin/openocd). The previous code appended openocd/bin/bin to it
#   and could never resolve.
# - Otherwise search the user's PATH only. FindHostTools has already cached
#   OPENOCD by now (preferring a Zephyr SDK host-tools openocd, which lacks
#   cc23xx support), so use a separate cache variable restricted to PATH.
#   Symlinked binaries are not resolved; the scripts path is derived from the
#   binary's install prefix.
# OpenOCD stays the default flasher over XDS110 via
# boards/common/openocd.board.cmake; J-Link remains available with
# `west flash -r jlink`.
if(DEFINED ENV{TI_OPENOCD_INSTALL_DIR})
  set(OPENOCD $ENV{TI_OPENOCD_INSTALL_DIR}/bin/openocd)
  set(OPENOCD_DEFAULT_PATH $ENV{TI_OPENOCD_INSTALL_DIR}/share/openocd/scripts)
else()
  find_program(TI_OPENOCD openocd NO_CMAKE_PATH NO_CMAKE_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH)
  if(TI_OPENOCD)
    set(OPENOCD ${TI_OPENOCD})
    get_filename_component(_openocd_prefix "${TI_OPENOCD}" DIRECTORY)
    get_filename_component(_openocd_prefix "${_openocd_prefix}" DIRECTORY)
    set(OPENOCD_DEFAULT_PATH ${_openocd_prefix}/share/openocd/scripts)
  endif()
endif()

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
