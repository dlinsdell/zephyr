#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>
#include "icm42605.h"
#include "icm42605_setup.h"

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

LOG_MODULE_DECLARE(ICM42605, CONFIG_SENSOR_LOG_LEVEL);

int inv_i2c_single_write(const struct device* dev, uint8_t reg, uint8_t* data)
{
	int result;
	const struct icm42605_config* cfg = dev->config;
	const uint8_t buf[2] = { reg, *data };
	result = i2c_write_dt(&cfg->i2c, buf, 2);

	if (result) {
		return result;
	}

	return 0;
}

int inv_i2c_read(const struct device *dev, uint8_t reg, uint8_t *data, size_t len)
{
	int result;
	const struct icm42605_config *cfg = dev->config;
	result = i2c_burst_read_dt(&cfg->i2c, reg, data, len);

	if (result) {
		return result;
	}
	return 0;
} 

static const struct icm42605_transfer_function icm42605_i2c_transfer_fn = {
	.inv_read = inv_i2c_read,
	.inv_single_write = inv_i2c_single_write,
};

int icm42605_i2c_init(const struct device *dev)
{
	struct icm42605_data *drv_data = dev->data;
	const struct icm42605_config *cfg = dev->config;

	drv_data->hw_tf = &icm42605_i2c_transfer_fn;
	drv_data->bus_type = 1;

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	return 0;

}

#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c) */
