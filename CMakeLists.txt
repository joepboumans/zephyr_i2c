#
# Copyright (c) 2018 Bosch Sensortec GmbH
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(I2C_app)

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/bno/blabla.c)
target_sources(app PRIVATE src/bno/blabla.h)
target_sources(app PRIVATE src/bno/bno055.c)
target_sources(app PRIVATE src/bno/bno055.h)
