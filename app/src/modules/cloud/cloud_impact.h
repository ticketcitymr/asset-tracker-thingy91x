/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _CLOUD_IMPACT_H_
#define _CLOUD_IMPACT_H_

#include <stdbool.h>
#include <stdint.h>

#include "impact.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Send an impact sample to nRF Cloud.
 *
 * @param impact Impact message to send.
 * @param timestamp_ms Timestamp in unix milliseconds (or NRF_CLOUD_NO_TIMESTAMP).
 * @param confirmable Whether to send as a confirmable CoAP message.
 *
 * @retval 0 on success, negative errno on failure.
 */
int cloud_impact_send(const struct impact_msg *impact, int64_t timestamp_ms, bool confirmable);

#ifdef __cplusplus
}
#endif

#endif /* _CLOUD_IMPACT_H_ */
