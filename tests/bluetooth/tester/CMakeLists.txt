# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

LIST(APPEND QEMU_EXTRA_FLAGS -serial unix:/tmp/bt-stack-tester)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tester)

zephyr_library_include_directories(${ZEPHYR_BASE}/samples/bluetooth)
zephyr_library_include_directories(${ZEPHYR_BASE}/subsys/bluetooth/mesh)
zephyr_library_include_directories(${ZEPHYR_NRF_MODULE_DIR}/subsys/bluetooth/mesh)
target_sources(app PRIVATE
    src/main.c
    src/bttester.c
    src/gap.c
    )

if(CONFIG_BT_MESH)
target_sources(app PRIVATE src/mesh.c)
target_sources(app PRIVATE src/mmdl.c)
target_sources(app PRIVATE src/model_handler.c)
endif()
