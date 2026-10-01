cmake_minimum_required(VERSION 3.20.0)

# Automatically discover custom boards defined in this repository's boards/ folder
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sufst_bms LANGUAGES C)

target_include_directories(app PRIVATE include)

target_sources(app PRIVATE
    src/main.c
    src/bms_sm.c
    src/bms_safety.c
    src/drv3946.c
    src/bms_hv_sense.c
)
