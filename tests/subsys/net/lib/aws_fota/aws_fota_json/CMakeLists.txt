#
# Copyright (c) 2019 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(json)

test_runner_generate(src/main.c)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

target_sources(app
  PRIVATE
  ${ZEPHYR_NRF_MODULE_DIR}/subsys/net/lib/aws_fota/src/aws_fota_json.c
  ${ZEPHYR_BASE}/subsys/net/lib/http/http_parser_url.c
  )

target_include_directories(app
  PRIVATE
  ${ZEPHYR_NRF_MODULE_DIR}/subsys/net/lib/aws_fota/include/
  ${ZEPHYR_BASE}/include/zephyr/net/http/
  )

# Do this in a non-standard way as the Kconfig options of "aws_jobs/Kconfig"
# is not executed. Hence these can not be set through prj.conf.
target_compile_options(app
  PRIVATE
  -DCONFIG_DOWNLOADER_MAX_HOSTNAME_SIZE=1024
  -DCONFIG_DOWNLOADER_MAX_FILENAME_SIZE=1024
  )
