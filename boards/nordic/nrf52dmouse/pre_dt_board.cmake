# Copyright (c) 2022 Nordic Semiconductor
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause

# Suppress "unique_unit_address_if_enabled" to handle the following overlaps:
# - power@40000000 & clock@40000000 & bprot@40000000
# - acl@4001e000 & flash-controller@4001e000
list(APPEND EXTRA_DTC_FLAGS "-Wno-unique_unit_address_if_enabled")
