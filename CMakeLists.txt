# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(i2c_demo)

list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/drivers/shtc3)
target_sources(app PRIVATE src/main.c)
# zephyr_include_directories(include)
add_subdirectory(drivers)
include_directories(../zephyr/include)
zephyr_include_directories($ENV{ZEPHYR_BASE}/drivers/sensor/shtcx)