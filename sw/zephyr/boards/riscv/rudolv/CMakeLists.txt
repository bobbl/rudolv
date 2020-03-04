# RudolV board implementation
#
# Copyright (c) 2020 Jörg Mische <bobbl@gmx.de>
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(timer_rudolv.c)
zephyr_library_sources(uart_rudolv.c)

zephyr_library_include_directories(${ZEPHYR_BASE}/drivers)

