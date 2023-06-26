/*
 * Copyright (c) 2020 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "icm42605.h"
#include "icm42605_reg.h"
#include "icm42605_setup.h"

LOG_MODULE_REGISTER(ICM42605, CONFIG_SENSOR_LOG_LEVEL);

static const uint16_t icm42605_gyro_sensitivity_x10[] = {
	1310, 655, 328, 164
};

/* see "Accelerometer Measurements" section from register map description */
static void icm42605_convert_accel(struct sensor_value *val,
				   int16_t raw_val,
				   uint16_t sensitivity_shift)
{
	int64_t conv_val;

	conv_val = ((int64_t)raw_val * SENSOR_G) >> sensitivity_shift;
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

/* see "Gyroscope Measurements" section from register map description */
static void icm42605_convert_gyro(struct sensor_value *val,
				  int16_t raw_val,
				  uint16_t sensitivity_x10)
{
	int64_t conv_val;

	conv_val = ((int64_t)raw_val * SENSOR_PI * 10) /
		   (sensitivity_x10 * 180U);
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

/* see "Temperature Measurement" section from register map description */
static inline void icm42605_convert_temp(struct sensor_value *val,
					 int16_t raw_val)
{
	val->val1 = (((int64_t)raw_val * 100) / 207) + 25;
	val->val2 = ((((int64_t)raw_val * 100) % 207) * 1000000) / 207;

	if (val->val2 < 0) {
		val->val1--;
		val->val2 += 1000000;
	} else if (val->val2 >= 1000000) {
		val->val1++;
		val->val2 -= 1000000;
	}
}

static int icm42605_channel_get(const struct device *dev,
				enum sensor_channel chan,
				struct sensor_value *val)
{
	const struct icm42605_data *drv_data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		icm42605_convert_accel(val, drv_data->accel_x,
				       drv_data->accel_sensitivity_shift);
		icm42605_convert_accel(val + 1, drv_data->accel_y,
				       drv_data->accel_sensitivity_shift);
		icm42605_convert_accel(val + 2, drv_data->accel_z,
				       drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_ACCEL_X:
		icm42605_convert_accel(val, drv_data->accel_x,
				       drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		icm42605_convert_accel(val, drv_data->accel_y,
				       drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		icm42605_convert_accel(val, drv_data->accel_z,
				       drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		icm42605_convert_gyro(val, drv_data->gyro_x,
				      drv_data->gyro_sensitivity_x10);
		icm42605_convert_gyro(val + 1, drv_data->gyro_y,
				      drv_data->gyro_sensitivity_x10);
		icm42605_convert_gyro(val + 2, drv_data->gyro_z,
				      drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_GYRO_X:
		icm42605_convert_gyro(val, drv_data->gyro_x,
				      drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_GYRO_Y:
		icm42605_convert_gyro(val, drv_data->gyro_y,
				      drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_GYRO_Z:
		icm42605_convert_gyro(val, drv_data->gyro_z,
				      drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_DIE_TEMP:
		icm42605_convert_temp(val, drv_data->temp);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

int icm42605_tap_fetch(const struct device *dev)
{
	int result = 0;
	struct icm42605_data *drv_data = dev->data;

	if (drv_data->tap_en &&
	    (drv_data->tap_handler || drv_data->double_tap_handler)) {
		result = drv_data->hw_tf->inv_read(dev, REG_INT_STATUS3, drv_data->fifo_data, 1);
		if (drv_data->fifo_data[0] & BIT_INT_STATUS_TAP_DET) {
			result = drv_data->hw_tf->inv_read(dev, REG_APEX_DATA4,
					      drv_data->fifo_data, 1);
			if (drv_data->fifo_data[0] & APEX_TAP) {
				if (drv_data->tap_trigger->type ==
				    SENSOR_TRIG_TAP) {
					if (drv_data->tap_handler) {
						LOG_DBG("Single Tap detected");
						drv_data->tap_handler(dev
						      , drv_data->tap_trigger);
					}
				} else {
					LOG_ERR("Trigger type is mismatched");
				}
			} else if (drv_data->fifo_data[0] & APEX_DOUBLE_TAP) {
				if (drv_data->double_tap_trigger->type ==
				    SENSOR_TRIG_DOUBLE_TAP) {
					if (drv_data->double_tap_handler) {
						LOG_DBG("Double Tap detected");
						drv_data->double_tap_handler(dev
						     , drv_data->tap_trigger);
					}
				} else {
					LOG_ERR("Trigger type is mismatched");
				}
			} else {
				LOG_DBG("Not supported tap event");
			}
		}
	}

	return 0;
}

static int icm42605_sample_fetch(const struct device *dev,
				 enum sensor_channel chan)
{
	int result = 0;
	uint16_t fifo_count = 0;
	struct icm42605_data *drv_data = dev->data;

	/* Read INT_STATUS (0x45) and FIFO_COUNTH(0x46), FIFO_COUNTL(0x47) */
	result = drv_data->hw_tf->inv_read(dev, REG_INT_STATUS, drv_data->fifo_data, 3);

	if (drv_data->fifo_data[0] & BIT_INT_STATUS_DRDY) {
		fifo_count = (drv_data->fifo_data[1] << 8)
			+ (drv_data->fifo_data[2]);
		result = drv_data->hw_tf->inv_read(dev, REG_FIFO_DATA, drv_data->fifo_data,
				      fifo_count);

		/* FIFO Data structure
		 * Packet 1 : FIFO Header(1), AccelX(2), AccelY(2),
		 *            AccelZ(2), Temperature(1)
		 * Packet 2 : FIFO Header(1), GyroX(2), GyroY(2),
		 *            GyroZ(2), Temperature(1)
		 * Packet 3 : FIFO Header(1), AccelX(2), AccelY(2), AccelZ(2),
		 *            GyroX(2), GyroY(2), GyroZ(2), Temperature(1)
		 */
		if (drv_data->fifo_data[0] & BIT_FIFO_HEAD_ACCEL) {
			/* Check empty values */
			if (!(drv_data->fifo_data[1] == FIFO_ACCEL0_RESET_VALUE
			      && drv_data->fifo_data[2] ==
			      FIFO_ACCEL1_RESET_VALUE)) {
				drv_data->accel_x =
					(drv_data->fifo_data[1] << 8)
					+ (drv_data->fifo_data[2]);
				drv_data->accel_y =
					(drv_data->fifo_data[3] << 8)
					+ (drv_data->fifo_data[4]);
				drv_data->accel_z =
					(drv_data->fifo_data[5] << 8)
					+ (drv_data->fifo_data[6]);
			}
			if (!(drv_data->fifo_data[0] & BIT_FIFO_HEAD_GYRO)) {
				drv_data->temp =
					(int16_t)(drv_data->fifo_data[7]);
			} else {
				if (!(drv_data->fifo_data[7] ==
				      FIFO_GYRO0_RESET_VALUE &&
				      drv_data->fifo_data[8] ==
				      FIFO_GYRO1_RESET_VALUE)) {
					drv_data->gyro_x =
						(drv_data->fifo_data[7] << 8)
						+ (drv_data->fifo_data[8]);
					drv_data->gyro_y =
						(drv_data->fifo_data[9] << 8)
						+ (drv_data->fifo_data[10]);
					drv_data->gyro_z =
						(drv_data->fifo_data[11] << 8)
						+ (drv_data->fifo_data[12]);
				}
				drv_data->temp =
					(int16_t)(drv_data->fifo_data[13]);
			}
		} else {
			if (drv_data->fifo_data[0] & BIT_FIFO_HEAD_GYRO) {
				if (!(drv_data->fifo_data[1] ==
				      FIFO_GYRO0_RESET_VALUE &&
				      drv_data->fifo_data[2] ==
				      FIFO_GYRO1_RESET_VALUE)) {
					drv_data->gyro_x =
						(drv_data->fifo_data[1] << 8)
						+ (drv_data->fifo_data[2]);
					drv_data->gyro_y =
						(drv_data->fifo_data[3] << 8)
						+ (drv_data->fifo_data[4]);
					drv_data->gyro_z =
						(drv_data->fifo_data[5] << 8)
						+ (drv_data->fifo_data[6]);
				}
				drv_data->temp =
					(int16_t)(drv_data->fifo_data[7]);
			}
		}
	}

	return 0;
}

static int icm42605_attr_set(const struct device *dev,
			     enum sensor_channel chan,
			     enum sensor_attribute attr,
			     const struct sensor_value *val)
{
	struct icm42605_data *drv_data = dev->data;

	__ASSERT_NO_MSG(val != NULL);

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
			if (val->val1 > 8000 || val->val1 < 1) {
				LOG_ERR("Incorrect sampling value");
				return -EINVAL;
			} else {
				drv_data->accel_hz = val->val1;
			}
		} else if (attr == SENSOR_ATTR_FULL_SCALE) {
			if (val->val1 < ACCEL_FS_16G ||
			    val->val1 > ACCEL_FS_2G) {
				LOG_ERR("Incorrect fullscale value");
				return -EINVAL;
			} else {
				drv_data->accel_sf = val->val1;
			}
		} else {
			LOG_ERR("Not supported ATTR");
			return -ENOTSUP;
		}

		break;
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_GYRO_XYZ:
		if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
			if (val->val1 > 8000 || val->val1 < 12) {
				LOG_ERR("Incorrect sampling value");
				return -EINVAL;
			} else {
				drv_data->gyro_hz = val->val1;
			}
		} else if (attr == SENSOR_ATTR_FULL_SCALE) {
			if (val->val1 < GYRO_FS_2000DPS ||
			    val->val1 > GYRO_FS_15DPS) {
				LOG_ERR("Incorrect fullscale value");
				return -EINVAL;
			} else {
				drv_data->gyro_sf = val->val1;
			}
		} else {
			LOG_ERR("Not supported ATTR");
			return -EINVAL;
		}
		break;
	default:
		LOG_ERR("Not support");
		return -EINVAL;
	}

	return 0;
}

static int icm42605_attr_get(const struct device *dev,
			     enum sensor_channel chan,
			     enum sensor_attribute attr,
			     struct sensor_value *val)
{
	const struct icm42605_data *drv_data = dev->data;

	__ASSERT_NO_MSG(val != NULL);

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
			val->val1 = drv_data->accel_hz;
		} else if (attr == SENSOR_ATTR_FULL_SCALE) {
			val->val1 = drv_data->accel_sf;
		} else {
			LOG_ERR("Not supported ATTR");
			return -EINVAL;
		}

		break;
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_GYRO_XYZ:
		if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
			val->val1 = drv_data->gyro_hz;
		} else if (attr == SENSOR_ATTR_FULL_SCALE) {
			val->val1 = drv_data->gyro_sf;
		} else {
			LOG_ERR("Not supported ATTR");
			return -EINVAL;
		}

		break;

	default:
		LOG_ERR("Not support");
		return -EINVAL;
	}

	return 0;
}

static int icm42605_data_init(struct icm42605_data *data,
			      const struct icm42605_config *cfg)
{
	data->accel_x = 0;
	data->accel_y = 0;
	data->accel_z = 0;
	data->temp = 0;
	data->gyro_x = 0;
	data->gyro_y = 0;
	data->gyro_z = 0;
	data->accel_hz = cfg->accel_hz;
	data->gyro_hz = cfg->gyro_hz;

	data->accel_sf = cfg->accel_fs;
	data->gyro_sf = cfg->gyro_fs;

	data->tap_en = false;
	data->sensor_started = false;

	return 0;
}


static int icm42605_init(const struct device *dev)
{
	int ret;
	const struct icm42605_config *cfg = dev->config;
	struct icm42605_data *drv_data = dev->data;

	ret = cfg->bus_init(dev);
	if (ret < 0) {
		LOG_ERR("Failed to initialize sensor bus");
		return ret;
	}

	icm42605_data_init(drv_data, cfg);
	icm42605_sensor_init(dev);

	drv_data->accel_sensitivity_shift = 14 - 3;
	drv_data->gyro_sensitivity_x10 = icm42605_gyro_sensitivity_x10[3];

#ifdef CONFIG_ICM42605_TRIGGER
	if (icm42605_init_interrupt(dev) < 0) {
		LOG_ERR("Failed to initialize interrupts.");
		return -EIO;
	}
#endif

	LOG_DBG("Initialize interrupt done");

	return 0;
}

static const struct sensor_driver_api icm42605_driver_api = {
#ifdef CONFIG_ICM42605_TRIGGER
	.trigger_set = icm42605_trigger_set,
#endif
	.sample_fetch = icm42605_sample_fetch,
	.channel_get = icm42605_channel_get,
	.attr_set = icm42605_attr_set,
	.attr_get = icm42605_attr_get,
};

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "ICM42605 driver enabled without any devices"
#endif

#define ICM42605_DEVICE_INIT(index)						\
	SENSOR_DEVICE_DT_INST_DEFINE(index,					\
				icm42605_init,							\
				NULL,									\
				&icm42605_data_##index,					\
				&icm42605_config_##index,				\
				POST_KERNEL,							\
				CONFIG_SENSOR_INIT_PRIORITY,			\
				&icm42605_driver_api);

#define ICM42605_CONFIG(index)									\
		.gpio_int = GPIO_DT_SPEC_INST_GET(index, int_gpios),	\
		.accel_hz = DT_INST_PROP(index, accel_hz),				\
		.gyro_hz = DT_INST_PROP(index, gyro_hz),				\
		.accel_fs = DT_INST_ENUM_IDX(index, accel_fs),			\
		.gyro_fs = DT_INST_ENUM_IDX(index, gyro_fs),			\

#define ICM42605_CONFIG_SPI(index)					\
	{												\
		.bus_init = icm42605_spi_init,				\
		.spi = SPI_DT_SPEC_INST_GET(index,			\
						SPI_OP_MODE_MASTER |		\
						SPI_MODE_CPOL |				\
						SPI_MODE_CPHA |				\
						SPI_WORD_SET(8) |			\
						SPI_TRANSFER_MSB,			\
						0U),						\
		ICM42605_CONFIG(index)						\
		}

#define ICM42605_DEFINE_SPI(index)					\
	static struct icm42605_data icm42605_data_##index;		\
	static const struct icm42605_config icm42605_config_##index =	\
		ICM42605_CONFIG_SPI(index);			\
	ICM42605_DEVICE_INIT(index)

#define ICM42605_CONFIG_I2C(index)					\
	{												\
		.bus_init = icm42605_i2c_init,				\
		.i2c = I2C_DT_SPEC_INST_GET(index),			\
		ICM42605_CONFIG(index)						\
	}

#define ICM42605_DEFINE_I2C(index)					\
	static struct icm42605_data icm42605_data_##index;		\
	static const struct icm42605_config icm42605_config_##index = 	\
		ICM42605_CONFIG_I2C(index);			\
	ICM42605_DEVICE_INIT(index)

#define ICM42605_DEFINE(index)						\
	COND_CODE_1(DT_INST_ON_BUS(index, spi),			\
			(ICM42605_DEFINE_SPI(index)),			\
			(ICM42605_DEFINE_I2C(index)))

DT_INST_FOREACH_STATUS_OKAY(ICM42605_DEFINE)
