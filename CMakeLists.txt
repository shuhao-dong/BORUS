# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(BORUS)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE 
    src/driver/bmp390.c
    src/driver/bmi270.c
    src/driver/battery.c
    src/feature/dsp_filters.c
    src/feature/gait_analysis.c
)