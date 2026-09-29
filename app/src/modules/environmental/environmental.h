/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _ENVIRONMENTAL_H_
#define _ENVIRONMENTAL_H_

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>

#ifdef __cplusplus
extern "C" {
#endif


/* Channels provided by this module */
ZBUS_CHAN_DECLARE(
	environmental_chan
);

enum environmental_msg_type {
	/* Output message types */

	/* Response message to a request for current environmental sensor values.
	 * The sampled values are found in the respective fields of the message structure.
	 */
	ENVIRONMENTAL_SENSOR_SAMPLE_RESPONSE = 0x1,

	/* Input message types */

	/* Request to sample the current environmental sensor values.
	 * The response is sent as a ENVIRONMENTAL_SENSOR_SAMPLE_RESPONSE message.
	 */
	ENVIRONMENTAL_SENSOR_SAMPLE_REQUEST,
};

struct environmental_msg {
	enum environmental_msg_type type;

	/** Contains the current temperature in celsius. */
	double temperature;

	/** Contains the current humidity in percentage. */
	double humidity;

	/** Contains the current pressure in Pa. */
	double pressure;

	/** Indoor air quality index from the BSEC library, 0 (clean) to 500 (heavily polluted).
	 *  Only meaningful when iaq_accuracy is above 0. Zero when CONFIG_BME68X_IAQ is disabled.
	 */
	double iaq;

	/** Estimated CO2 equivalent in ppm from the BSEC library. */
	double co2;

	/** Estimated breath VOC equivalent in ppm from the BSEC library. */
	double voc;

	/** BSEC accuracy of the air quality values: 0 unreliable, 1 low, 2 medium, 3 high.
	 *  The values improve as the sensor calibrates over hours to days.
	 */
	int iaq_accuracy;

	/** Timestamp when the sample was taken in milliseconds.
	 *  This is either:
	 * - Unix time in milliseconds if the system clock was synchronized at sampling time, or
	 * - Uptime in milliseconds if the system clock was not synchronized at sampling time.
	 * Only valid for ENVIRONMENTAL_SENSOR_SAMPLE_RESPONSE events.
	 */
	int64_t timestamp;
};


#ifdef __cplusplus
}
#endif

#endif /* _ENVIRONMENTAL_H_ */
