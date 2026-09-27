/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _CLOUD_PEDOMETER_H_
#define _CLOUD_PEDOMETER_H_

#include <stdbool.h>
#include <stdint.h>

#include "pedometer.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Send a pedometer report to nRF Cloud as a custom "PEDOMETER" appId message. */
int cloud_pedometer_send(const struct pedometer_msg *ped, int64_t timestamp_ms, bool confirmable);

#ifdef __cplusplus
}
#endif

#endif /* _CLOUD_PEDOMETER_H_ */
