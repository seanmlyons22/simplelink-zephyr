# Copyright (c) 2026 Texas Instruments
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(entropy_lpf3_rng_init)

target_sources(app PRIVATE src/main.c)
