#
# Copyright (c) 2026 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

target_include_directories(app PRIVATE .)
target_sources(app PRIVATE impact.c post_impact.c)
target_sources_ifdef(CONFIG_APP_BMI270_LOG app PRIVATE imu_log.c)
