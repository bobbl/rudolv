# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
# Override standard Microsemi UART adressing with the on from Future Electronics
set(DTC_OVERLAY_FILE ${CMAKE_CURRENT_LIST_DIR}/src/ripe.dts)

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
#project(hello_world C ASM)
project(hello_world)

#target_sources(app PRIVATE src/ripe_attack_generator.c src/setjmp.S )
target_sources(app PRIVATE src/ripe_attack_generator.c )
