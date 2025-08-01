#
# Copyright (c) 2019 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project("Application Event Manager sample")

# Include application application event headers
zephyr_library_include_directories(src/events)

# Application sources
# NORDIC SDK APP START
target_sources(app PRIVATE src/main.c)

target_sources(app PRIVATE
		src/events/ack_event.c
		src/events/config_event.c
		src/events/control_event.c
		src/events/measurement_event.c
)

target_sources(app PRIVATE
		src/modules/controller.c
		src/modules/sensor_simulated.c
		src/modules/stats.c
)
# NORDIC SDK APP END
