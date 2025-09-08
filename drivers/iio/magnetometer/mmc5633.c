// SPDX-License-Identifier: GPL-2.0-only
/*
 * MMC5633 - MEMSIC 3-axis Magnetic Sensor (based on MMC35240 driver.)
 *
 * Copyright (c) 2015, Intel Corporation.
 * Copyright (c) 2022, IMD tec.
 *
 * IIO driver for MMC5633 (7-bit I2C slave address 0x30).
 *
 * TODO: offset, ACPI, continuous measurement mode, PM
 *
 * MMC5633 NOTES:
 * ==============
 *
 * TODO: Use better values for the BW settings.
 * TODO: Use HP mode with higher bandwidth.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/acpi.h>
#include <linux/pm.h>

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

#define MMC5633_DRV_NAME "mmc5633"
#define MMC5633_REGMAP_NAME "mmc5633_regmap"

#define MMC5633_REG_XOUT0	0x00
#define MMC5633_REG_XOUT1	0x01
#define MMC5633_REG_YOUT0	0x02
#define MMC5633_REG_YOUT1	0x03
#define MMC5633_REG_ZOUT0	0x04
#define MMC5633_REG_ZOUT1	0x05

#define MMC5633_REG_XOUT2	0x06
#define MMC5633_REG_YOUT2	0x07
#define MMC5633_REG_ZOUT2	0x08


#define MMC5633_REG_STATUS1	0x18
#define MMC5633_REG_STATUS0	0x19

#define MMC5633_REG_ODR		0x1A
#define MMC5633_REG_CTRL0	0x1B
#define MMC5633_REG_CTRL1	0x1C
#define MMC5633_REG_CTRL2	0x1D

#define MMC5633_REG_PRODUCT_ID	0x39

#define MM5633_REG_MAX (MMC5633_REG_PRODUCT_ID)

#define MMC5633_STATUS1_MEAS_DONE_BIT		BIT(6)
#define MMC5633_STATUS1_MEAS_DONE_INT_BIT	BIT(0)

#define MMC5633_CTRL0_TM_BIT		BIT(0)
#define MMC5633_CTRL0_SET_BIT 		BIT(3)
#define MMC5633_CTRL0_RESET_BIT		BIT(4)
#define MMC5633_CTRL0_AUTO_SR		BIT(5)
#define MMC5633_CRTL0_CMM_FREQ_EN	BIT(7)

/* output resolution bits */

#define MMC5633_CTRL1_BW0_BIT		BIT(0)
#define MMC5633_CTRL1_BW1_BIT		BIT(1)


#define MMC5633_CTRL1_BW_MASK \
	 (MMC5633_CTRL1_BW0_BIT |  MMC5633_CTRL1_BW1_BIT)

#define MMC5633_CTRL1_BW_SHIFT		0

#define MMC5633_CRTL2_CMM_EN 		BIT(4)
#define MMC5633_CTRL2_HPOWER 		BIT(7)

#define MMC5633_WAIT_SET_RESET		1000	/* us */

/*
 * The MMC5633 has a register layout that is three 16 bit values
 * and then another three bytes that contains extra bits.
 * Read the whole lot as bytes.
 */

#define MMC5633_BLK_READ_SIZE_BYTES	(3*3)
#define MMC5633_REG_BLK_READ_START	(MMC5633_REG_XOUT0)

enum mmc5633_bw {
	MMC5633_BW_00_66 = 0, /* 6.6ms */
	MMC5633_BW_01_35 = 1, /* 3.5ms */
	MMC5633_BW_10_20 = 2, /* 2.0ms */
	MMC5633_BW_11_12 = 3  /* 1.2ms */
};

/*
 * How long to wait to take a measurment. (us)
 */
static const int bw_tm_wait_us[] = {
	6600,
	3500,
	2000,
	1200
};

/*
 * ODR setting for different BWs. (hz)
 */
static const int odr_settings[] = {
	75,
	150,
	255,
	255 /* This will give 1000hz in high power mode. */
};

/*
 * How long extra we are prepared to take a measurment. (us)
 */

#define MMC5633_TM_WAIT_RANGE_US 100

enum mmc5633_axis {
	AXIS_X = 0,
	AXIS_Y,
	AXIS_Z,
};


struct mmc5633_data {
	struct i2c_client *client;
	struct mutex mutex;
	struct regmap *regmap;
	enum mmc5633_bw bw;
	struct iio_trigger *trig;
	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 * ?? Is this true for mmc5633?
	 */
	union {
		struct {
			/* Just supply enabled values */
			__le32 sample[3];
			s64 timestamp;
		} scan;
		__le32 d32;
		u8 d8[2];
	} data ____cacheline_aligned;
};


/*
 * These samp frequiecies do not seam to relate to anything.
 * However, having these allows the changing of the chip's BW
 */

static const struct {
	int val;
	int val2;
} mmc5633_samp_freq[] = { {75, 0},
			   {150, 0},
			   {255, 0},
			   {1000, 0} };


static IIO_CONST_ATTR_SAMP_FREQ_AVAIL("75 150 255 1000");

#define MMC5633_CHANNEL(_axis,_ind) { \
	.type = IIO_MAGN, \
	.modified = 1, \
	.channel2 = IIO_MOD_ ## _axis, \
	.address = AXIS_ ## _axis, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ) | \
			BIT(IIO_CHAN_INFO_SCALE), \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.scan_index = _ind, \
	.scan_type = { \
			.sign = 's', \
			.realbits = 32, \
			.storagebits = 32, \
			.shift = 0, \
			.endianness = IIO_CPU, \
		}, \
}

static const struct iio_chan_spec mmc5633_channels[] = {
	MMC5633_CHANNEL(X,0),
	MMC5633_CHANNEL(Y,1),
	MMC5633_CHANNEL(Z,2),
	IIO_CHAN_SOFT_TIMESTAMP(3)
};

static struct attribute *mmc5633_attributes[] = {
	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
	NULL
};

static const struct attribute_group mmc5633_attribute_group = {
	.attrs = mmc5633_attributes,
};

static int mmc5633_get_samp_freq_index(struct mmc5633_data *data,
					int val, int val2)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mmc5633_samp_freq); i++)
		if (mmc5633_samp_freq[i].val == val &&
		    mmc5633_samp_freq[i].val2 == val2)
			return i;
	return -EINVAL;
}

static int mmc5633_hw_set(struct mmc5633_data *data, bool set)
{
	u8 coil_bit;

	if (set)
		coil_bit = MMC5633_CTRL0_SET_BIT;
	else
		coil_bit = MMC5633_CTRL0_RESET_BIT;

	return regmap_update_bits(
		data->regmap,
		MMC5633_REG_CTRL0,
		coil_bit,
		coil_bit);
}


static int mmc5633_init(struct mmc5633_data *data)
{
	int ret;
	unsigned int reg_id;


	ret = regmap_read(data->regmap, MMC5633_REG_PRODUCT_ID, &reg_id);
	if (ret < 0) {
		dev_err(&data->client->dev, "Error reading product id\n");
		return ret;
	}

	dev_dbg(&data->client->dev, "MMC5633 chip id %x\n", reg_id);

	/*
	 * make sure we restore sensor characteristics, by doing
	 * a SET/RESET sequence, the axis polarity being naturally
	 * aligned after RESET
	 *
	 * (Inhetited from MMC35240 driver. Asumued harmless.
	 * Unsure if required.)
	 */

	ret = mmc5633_hw_set(data, true);
	if (ret < 0)
		return ret;
	usleep_range(MMC5633_WAIT_SET_RESET, MMC5633_WAIT_SET_RESET + 1);

	ret = mmc5633_hw_set(data, false);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(data->regmap, MMC5633_REG_CTRL1,
		MMC5633_CTRL1_BW_MASK,
		data->bw << MMC5633_CTRL1_BW_SHIFT);

	if (ret < 0)
		return ret;

	return 0;
}

static int mmc5633_take_measurement(struct mmc5633_data *data)
{
	int ret, tries = 100;
	unsigned int reg_status;

	ret = regmap_write(
		data->regmap,
		MMC5633_REG_CTRL0,
		MMC5633_CTRL0_TM_BIT | MMC5633_CTRL0_AUTO_SR);

	if (ret < 0)
		return ret;

	while (tries-- > 0) {
		ret = regmap_read(data->regmap, MMC5633_REG_STATUS1,
				  &reg_status);
		if (ret < 0)
			return ret;
		if (reg_status & MMC5633_STATUS1_MEAS_DONE_BIT)
			break;

		usleep_range(
			bw_tm_wait_us[data->bw],
			bw_tm_wait_us[data->bw] + MMC5633_TM_WAIT_RANGE_US);
	}

	if (tries < 0) {
		dev_err(&data->client->dev, "data not ready\n");
		return -EIO;
	}

	return 0;
}

static int mmc5633_read_bulk(
	struct mmc5633_data *data,
	u8 buf[MMC5633_BLK_READ_SIZE_BYTES])
{
	return regmap_bulk_read(
		data->regmap,
		MMC5633_REG_BLK_READ_START,
		buf,
		MMC5633_BLK_READ_SIZE_BYTES);
}

static int mmc5633_read_measurement(
	struct mmc5633_data *data,
	u8 buf[MMC5633_BLK_READ_SIZE_BYTES])
{
	int ret;

	ret = mmc5633_take_measurement(data);
	if (ret < 0)
		return ret;

	return mmc5633_read_bulk(data, buf);
}


/*
 * At the time of writing mmc5633_raw_to_mgauss() was not BW
 * dependent. However data param was not removed.
 */
static int mmc5633_raw_to_mgauss(struct mmc5633_data *data, int index,
				  u8 buf[], int *val)
{
	switch (index) {
	case AXIS_X:
		*val = (
				(buf[MMC5633_REG_XOUT0] << 12) +
				(buf[MMC5633_REG_XOUT1] << 4) +
				(buf[MMC5633_REG_XOUT2] >> 4)
			) - 0x80000;
		break;
	case AXIS_Y:
		*val = (
				(buf[MMC5633_REG_YOUT0] << 12) +
				(buf[MMC5633_REG_YOUT1] << 4) +
				(buf[MMC5633_REG_YOUT2] >> 4)
			) - 0x80000;
		break;
	case AXIS_Z:
		*val = (
				(buf[MMC5633_REG_ZOUT0] << 12) +
				(buf[MMC5633_REG_ZOUT1] << 4) +
				(buf[MMC5633_REG_ZOUT2] >> 4)
			) - 0x80000;
		break;
	default:
		return -EINVAL;
	}

	return 0;

}

static int mmc5633_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, int *val,
			     int *val2, long mask)
{
	struct mmc5633_data *data = iio_priv(indio_dev);
	int ret, i;
	u8 buf[MMC5633_BLK_READ_SIZE_BYTES];

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret) {
			dev_err(&data->client->dev,
				"Failed to claim direct mode.");
			return ret;
		}

		mutex_lock(&data->mutex);
		ret = mmc5633_read_measurement(data, buf);
		mutex_unlock(&data->mutex);
		if (ret < 0) {
			dev_err(&data->client->dev,
				"Failed to read measurment.");
			goto err_chan_info_raw;
		}
		ret = mmc5633_raw_to_mgauss(data, chan->address, buf, val);
		if (ret < 0) {
			dev_err(&data->client->dev,
				"Failed convert raw to mgauss.");
			goto err_chan_info_raw;
		}
err_chan_info_raw:
		iio_device_release_direct_mode(indio_dev);

		if (ret < 0)
			return ret;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = 1000; /* Milli G */
		*val2 = 16384; /* 16384 counts per G */
		return IIO_VAL_FRACTIONAL;
	case IIO_CHAN_INFO_SAMP_FREQ:
		i = data->bw;
		*val = mmc5633_samp_freq[i].val;
		*val2 = mmc5633_samp_freq[i].val2;
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static int mmc5633_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan, int val,
			      int val2, long mask)
{
	struct mmc5633_data *data = iio_priv(indio_dev);
	int i, ret;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		i = mmc5633_get_samp_freq_index(data, val, val2);
		if (i < 0)
			return -EINVAL;
		mutex_lock(&data->mutex);

		ret = regmap_update_bits(data->regmap, MMC5633_REG_CTRL1,
			MMC5633_CTRL1_BW_MASK,
			i << MMC5633_CTRL1_BW_SHIFT);

		data->bw = i;

		mutex_unlock(&data->mutex);
		return ret;
	default:
		return -EINVAL;
	}
}

static const struct iio_info mmc5633_info = {
	.read_raw	= mmc5633_read_raw,
	.write_raw	= mmc5633_write_raw,
	.attrs		= &mmc5633_attribute_group,
};

static bool mmc5633_is_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MMC5633_REG_CTRL0:
	case MMC5633_REG_CTRL1:
	case MMC5633_REG_CTRL2:
	case MMC5633_REG_ODR:
		return true;
	default:
		return false;
	}
}

static bool mmc5633_is_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MMC5633_REG_XOUT0:
	case MMC5633_REG_XOUT1:
	case MMC5633_REG_XOUT2:
	case MMC5633_REG_YOUT0:
	case MMC5633_REG_YOUT1:
	case MMC5633_REG_YOUT2:
	case MMC5633_REG_ZOUT0:
	case MMC5633_REG_ZOUT1:
	case MMC5633_REG_ZOUT2:
	case MMC5633_REG_STATUS1:
	case MMC5633_REG_PRODUCT_ID:
		return true;
	default:
		return false;
	}
}

static bool mmc5633_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MMC5633_REG_CTRL0:
	case MMC5633_REG_CTRL1:
	case MMC5633_REG_CTRL2:
		return false;
	default:
		return true;
	}
}

static const struct reg_default mmc5633_reg_defaults[] = {
	{ MMC5633_REG_CTRL0,  0x00 },
	{ MMC5633_REG_CTRL1,  0x00 },
	{ MMC5633_REG_CTRL2,  0x00 },
	{ MMC5633_REG_ODR,  0x00 },
};

static const struct regmap_config mmc5633_regmap_config = {
	.name = MMC5633_REGMAP_NAME,

	.reg_bits = 8,
	.val_bits = 8,

	.max_register = MM5633_REG_MAX,
	.cache_type = REGCACHE_FLAT,

	.writeable_reg = mmc5633_is_writeable_reg,
	.readable_reg = mmc5633_is_readable_reg,
	.volatile_reg = mmc5633_is_volatile_reg,

	.reg_defaults = mmc5633_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(mmc5633_reg_defaults),
};


static irqreturn_t mmc5633_trigger_handler(int irq, void *p)
{
	int ret = 0;
	int sample_count = 0; /* Count of samples enabled. */
	int sample;
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct mmc5633_data *data = iio_priv(indio_dev);
	unsigned long mask = *indio_dev->active_scan_mask;

	u8 buf[MMC5633_BLK_READ_SIZE_BYTES];

	mutex_lock(&data->mutex);

	ret = mmc5633_read_bulk(data, buf);
	if(ret < 0) {
		dev_err(&data->client->dev,
			"Failed to read triggered measurment.");
	}

	for(sample = 0; sample < 3; sample++) {
		if(mask & BIT(sample)) {
			ret = mmc5633_raw_to_mgauss(
				data,
				sample,
				buf,
				&data->data.scan.sample[sample_count]);

			if(ret < 0) {
				dev_err(&data->client->dev,
					"Failed to convert triggered x measurment.");
			}
			sample_count++;
		}
	}

	iio_push_to_buffers_with_timestamp(indio_dev, &data->data.scan,
					   iio_get_time_ns(indio_dev));

	iio_trigger_notify_done(indio_dev->trig);

	mutex_unlock(&data->mutex);

	return IRQ_HANDLED;
}

static int mmc5633_buffer_postenable(struct iio_dev *indio_dev)
{
	int ret = 0;
	struct mmc5633_data *data = iio_priv(indio_dev);
	int odr, hpower, crtl0_mask, crtl0_value, crtl2_mask, crtl2_value;
	mutex_lock(&data->mutex);

	/* Setup ODR */

	odr = odr_settings[data->bw];

	ret = regmap_write(
		data->regmap,
		MMC5633_REG_ODR,
		odr);

	if (ret < 0) {
		dev_err(&data->client->dev,
			"Failed to write ODR");
		goto err;
	}

	hpower = (data->bw == MMC5633_BW_11_12) ? MMC5633_CTRL2_HPOWER : 0;

	crtl0_mask = MMC5633_CRTL0_CMM_FREQ_EN;
	crtl0_value = MMC5633_CRTL0_CMM_FREQ_EN;

	/* Enable continuous mode */

	ret = regmap_update_bits(data->regmap, MMC5633_REG_CTRL0,
		crtl0_mask,
		crtl0_value);

	if (ret < 0) {
		dev_err(&data->client->dev,
			"Failed set crtl0 in post enable.");
		goto err;
	}

	crtl2_mask = MMC5633_CTRL2_HPOWER| MMC5633_CRTL2_CMM_EN;
	crtl2_value = hpower | MMC5633_CRTL2_CMM_EN;

	ret = regmap_update_bits(data->regmap, MMC5633_REG_CTRL2,
		crtl2_mask,
		crtl2_value);

	if (ret < 0) {
		dev_err(&data->client->dev,
			"Failed set crtl2 in post enable.");
		goto err;
	}

err:
	mutex_unlock(&data->mutex);
	return ret;
}

static int mmc5633_buffer_predisable(struct iio_dev *indio_dev)
{
	int ret = 0;
	struct mmc5633_data *data = iio_priv(indio_dev);
	int hpower, crtl0_mask, crtl0_value, crtl2_mask, crtl2_value;

	mutex_lock(&data->mutex);

	hpower = 0;

	crtl0_mask = MMC5633_CRTL0_CMM_FREQ_EN;
	crtl0_value = 0;

	ret = regmap_update_bits(
		data->regmap,
		MMC5633_REG_CTRL0,
		crtl0_mask,
		crtl0_value);

	if (ret < 0) {
		dev_err(&data->client->dev,
			"Failed set crtl0 in pre disable.");
		goto err;
	}

	crtl2_mask = MMC5633_CTRL2_HPOWER| MMC5633_CRTL2_CMM_EN;
	crtl2_value = 0;

	ret = regmap_update_bits(
		data->regmap,
		MMC5633_REG_CTRL2,
		crtl2_mask,
		crtl2_value);

	if (ret < 0) {
		dev_err(&data->client->dev,
			"Failed set crtl2 in pre disable.");
		goto err;
	}

err:
	mutex_unlock(&data->mutex);

	return ret;
}

static const struct iio_buffer_setup_ops mmc5633_buffer_ops = {
	.postenable = &mmc5633_buffer_postenable,
	.predisable = &mmc5633_buffer_predisable,
};

static const struct iio_trigger_ops mmc5633_trigger_ops = {
	.validate_device = iio_trigger_validate_own_device,
};

static int mmc5633_probe(struct i2c_client *client)
{
	struct mmc5633_data *data;
	struct iio_dev *indio_dev;
	struct regmap *regmap;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	regmap = devm_regmap_init_i2c(client, &mmc5633_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "regmap initialization failed\n");
		return PTR_ERR(regmap);
	}

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;
	data->regmap = regmap;
	data->bw = MMC5633_BW_00_66; /* least noise */

	mutex_init(&data->mutex);

	indio_dev->info = &mmc5633_info;
	indio_dev->name = MMC5633_DRV_NAME;
	indio_dev->channels = mmc5633_channels;
	indio_dev->num_channels = ARRAY_SIZE(mmc5633_channels);
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_TRIGGERED;


	data->trig = devm_iio_trigger_alloc(&client->dev, "%s-dev%d",
					  indio_dev->name, iio_device_id(indio_dev));
	if (!data->trig)
		return -ENOMEM;

	data->trig->ops = &mmc5633_trigger_ops;
	data->trig->dev.parent = &client->dev;
	iio_trigger_set_drvdata(data->trig, indio_dev);
	ret = devm_iio_trigger_register(&client->dev, data->trig);
	if (ret)
		return ret;

	indio_dev->trig = iio_trigger_get(data->trig);


	ret = devm_iio_triggered_buffer_setup(&client->dev, indio_dev,
					      &iio_pollfunc_store_time,
					      &mmc5633_trigger_handler,
					      &mmc5633_buffer_ops);

	if (ret) {
		dev_err(&client->dev, "Failed to setup triggered buffers");
		return ret;
	}

	ret = mmc5633_init(data);

	if (ret < 0) {
		dev_err(&client->dev, "mmc5633 chip init failed\n");
		return ret;
	}

	ret = devm_iio_device_register(&client->dev, indio_dev);
	if (ret < 0) {
		dev_err(&client->dev, "mmc5633 register failed\n");
		return ret;
	}
	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int mmc5633_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct mmc5633_data *data = iio_priv(indio_dev);

	regcache_cache_only(data->regmap, true);

	return 0;
}

static int mmc5633_resume(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct mmc5633_data *data = iio_priv(indio_dev);
	int ret;

	regcache_mark_dirty(data->regmap);
	ret = regcache_sync_region(data->regmap, MMC5633_REG_CTRL0,
				   MMC5633_REG_CTRL2);
	if (ret < 0)
		dev_err(dev, "Failed to restore control registers\n");

	regcache_cache_only(data->regmap, false);

	return 0;
}
#endif

static const struct dev_pm_ops mmc5633_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mmc5633_suspend, mmc5633_resume)
};

static const struct of_device_id mmc5633_of_match[] = {
	{ .compatible = "memsic,mmc5633", },
	{ }
};
MODULE_DEVICE_TABLE(of, mmc5633_of_match);

static const struct acpi_device_id mmc5633_acpi_match[] = {
	{"MMC5633", 0},
	{ },
};
MODULE_DEVICE_TABLE(acpi, mmc5633_acpi_match);

static const struct i2c_device_id mmc5633_id[] = {
	{"mmc5633", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, mmc5633_id);

static struct i2c_driver mmc5633_driver = {
	.driver = {
		.name = MMC5633_DRV_NAME,
		.of_match_table = mmc5633_of_match,
		.pm = &mmc5633_pm_ops,
		.acpi_match_table = ACPI_PTR(mmc5633_acpi_match),
	},
	.probe		= mmc5633_probe,
	.id_table	= mmc5633_id,
};

module_i2c_driver(mmc5633_driver);

MODULE_AUTHOR("Daniel Baluta <daniel.baluta@intel.com>");
MODULE_AUTHOR("William Bagshaw <william.bagshaw@imd-tec.com>");
MODULE_DESCRIPTION("MEMSIC MMC5633 magnetic sensor driver");
MODULE_LICENSE("GPL v2");
