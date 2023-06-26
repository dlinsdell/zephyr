/*
 * Copyright (c) 2020 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>
#include "icm42605.h"
#include "icm42605_setup.h"
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

LOG_MODULE_DECLARE(ICM42605, CONFIG_SENSOR_LOG_LEVEL);

int inv_spi_single_write(const struct device *dev, uint8_t reg, uint8_t *data)
{
	int result;
	const struct icm42605_config* cfg = dev->config;

	const struct spi_buf buf[2] = {
		{
			.buf = &reg,
			.len = 1,
		},
		{
			.buf = data,
			.len = 1,
		}
	};
	const struct spi_buf_set tx = {
		.buffers = buf,
		.count = 2,
	};

	result = spi_write_dt(&cfg->spi, &tx);

	if (result) {
		return result;
	}

	return 0;
}

int inv_spi_read(const struct device *dev, uint8_t reg, uint8_t *data, size_t len)
{
	int result;
	const struct icm42605_config *cfg = dev->config;
	unsigned char tx_buffer[2] = { 0, };

	tx_buffer[0] = 0x80 | reg;

	const struct spi_buf tx_buf = {
		.buf = tx_buffer,
		.len = 1,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};

	struct spi_buf rx_buf[2] = {
		{
			.buf = tx_buffer,
			.len = 1,
		},
		{
			.buf = data,
			.len = len,
		}
	};

	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = 2,
	};

	result = spi_transceive_dt(&cfg->spi, &tx, &rx);

	if (result) {
		return result;
	}

	return 0;
}

static const struct icm42605_transfer_function icm42605_spi_transfer_fn = {
	.inv_read = inv_spi_read,
	.inv_single_write = inv_spi_single_write,
};

int icm42605_spi_init(const struct device *dev)
{
	struct icm42605_data *drv_data = dev->data;
	const struct icm42605_config *cfg = dev->config;

	drv_data->hw_tf = &icm42605_spi_transfer_fn;
	drv_data->bus_type = 0;

	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	return 0;

}

#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */
