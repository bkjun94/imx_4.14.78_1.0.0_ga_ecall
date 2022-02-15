/*
 * STMicroelectronics st_ais2120 sensor driver
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <asm/unaligned.h>

#include <linux/platform_data/st_sensors_pdata.h>

#include "st_ais2120.h"

#define AIS2120_DEV_NAME		"ais2120"

#define REG_ID_SENSOR_TYPE      0x0C
#define REG_WHOAMI_ADDR			REG_ID_SENSOR_TYPE
#define REG_WHOAMI_VAL			0x2A

#define REG_CONFIG_ADDR         0x02

#define REG_ACC_CHX_LOW         0x07
#define REG_ACC_CHY_LOW         0x09

#define FIR_BANDWITH_NUM        2

static const int ais2120_fir[FIR_BANDWITH_NUM] = {400, 800};

#define ST_AIS2120_DATA_CHANNEL(addr, modx, scan_idx)		\
{								\
	.type = IIO_ACCEL,					\
	.address = addr,					\
	.modified = 1,						\
	.channel2 = modx,					\
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.scan_index = scan_idx,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 14,					\
		.storagebits = 14,				\
		.endianness = IIO_LE,				\
	},							\
}

static const struct iio_chan_spec st_ais2120_channels[] = {
	ST_AIS2120_DATA_CHANNEL(REG_ACC_CHX_LOW, IIO_MOD_X, 0),
	ST_AIS2120_DATA_CHANNEL(REG_ACC_CHY_LOW, IIO_MOD_Y, 1),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};


int st_ais2120_spi_read(struct device *dev, u8 addr, int len, u8 *data)
{
	struct spi_device *spi = to_spi_device(dev);
    //struct st_ais2120_hw *hw = spi_get_drvdata(spi);
	int ret;
	u8 tx_buf[4], rx_buf[4];

	ais2120_reg_make_cmd(tx_buf, 1, addr, 0);
	ret = spi_write(spi, tx_buf, ST_AIS2120_DATA_SIZE);
	if (ret < 0) {
        return ret;
    }
	ret = spi_read(spi, rx_buf, ST_AIS2120_DATA_SIZE);
	if (ret < 0) {
        return ret;
    }
	ret = ais2120_reg_decode_data(rx_buf, data);
    if (ret != 0) {
        return -(ret);
    }
	return ret;
}

int st_ais2120_spi_read_sensor_data(struct device *dev, u8 channel, u16 *data)
{
	struct spi_device *spi = to_spi_device(dev);
    //struct st_ais2120_hw *hw = spi_get_drvdata(spi);
	int ret;
	u8 tx_buf[4], rx_buf[4];

	ais2120_sensor_make_cmd(tx_buf, channel);
	ret = spi_write(spi, tx_buf, ST_AIS2120_DATA_SIZE);
	if (ret < 0) {
        return ret;
    }
	ret = spi_read(spi, rx_buf, ST_AIS2120_DATA_SIZE);
	if (ret < 0) {
        return ret;
    }
	ret = ais2120_sensor_decode_data(rx_buf, data);
    if (ret != 0) {
        printk("[ais2120] spi sensor read error = %02X\n", ret);
        return -(ret);
    }
	return ret;
}

static int st_ais2120_spi_write(struct device *dev, u8 addr, int len, u8 *data)
{
	struct spi_device *spi = to_spi_device(dev);
    //struct st_ais2120_hw *hw = spi_get_drvdata(spi);
	int ret;
	u8 tx_buf[4];

	ais2120_reg_make_cmd(tx_buf, 0, addr, (u8)(*data));
	ret = spi_write(spi, tx_buf, ST_AIS2120_DATA_SIZE);
    return ret;
}

int st_ais2120_write_with_mask(struct device *dev, u8 addr, u8 mask, u8 val)
{
	u8 data;
	int ret;

    struct st_ais2120_hw *hw = iio_priv(dev_get_drvdata(dev));

	mutex_lock(&hw->lock);

	ret = st_ais2120_spi_read(hw->dev, addr, 1, &data);
	if (ret < 0) {
		dev_err(hw->dev, "failed to spi read %02x register\n", addr);
		mutex_unlock(&hw->lock);
		return ret;
	}
    printk("shsong read data=%02X\n", data);

	data = (data & ~mask) | ((val << __ffs(mask)) & mask);

    printk("shsong write data=%02X\n", data);

	ret = st_ais2120_spi_write(hw->dev, addr, 1, (u8 *)&data);
	if (ret < 0) {
		dev_err(hw->dev, "failed to spi write %02x register\n", addr);
		mutex_unlock(&hw->lock);
		return ret;
	}

	mutex_unlock(&hw->lock);
	return 0;
}

#if 0
int st_ais2120_set_enable(struct st_ais2120_hw *hw, bool enable)
{
	return st_ais2120_write_with_mask(hw, REG_CTRL1_ADDR,
					  REG_CTRL1_EN_MASK, enable);
}
#endif

static int st_ais2120_read_raw(struct iio_dev *iio_dev,
			       struct iio_chan_spec const *ch,
			       int *val, int *val2, long mask)
{
	struct st_ais2120_hw *hw = iio_priv(iio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW: {
		//u8 data[2]; // 14bit
        u16 data; // 14bit(0~0x3FFF, 16383)

		mutex_lock(&iio_dev->mlock);

		//ret = st_ais2120_spi_read_sensor_data(hw->dev, ch->address, 1, data);
		ret = st_ais2120_spi_read_sensor_data(hw->dev, ch->scan_index, (u16 *)&data);
        //printk("[ais2120] channel=%d, raw=0x%04X\n", ch->scan_index, (u16)data);
		if (ret < 0) {
            dev_err(hw->dev, "failed to read %d channel sensor data\n", ch->scan_index);
			mutex_unlock(&iio_dev->mlock);
			return ret;
		}
		//*val = (u16)data;
        //*val = (s16)get_unaligned_le16(&data);		
		*val = (s16)((data ^ 1 << 13) - (1 << 13));
        mutex_unlock(&iio_dev->mlock);

		ret = IIO_VAL_INT;
		break;
	}
    
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = 0;
		ret = IIO_VAL_INT_PLUS_MICRO;
		break;

	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = 0;
		ret = IIO_VAL_INT;
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static __maybe_unused int st_ais2120_reg_access(struct iio_dev *iio_dev,
                 unsigned int reg, unsigned int writeval,
                 unsigned int *readval)
{
    struct st_ais2120_hw *hw = iio_priv(iio_dev);
    int ret;

    ret = iio_device_claim_direct_mode(iio_dev);
    if (ret)
        return ret;    

    if (readval == NULL) {
        ret = st_ais2120_spi_write(hw->dev, reg, 1, (u8 *)&writeval);
    } else {
        ret = st_ais2120_spi_read(hw->dev, reg, 1, (u8 *)readval);
    }
    iio_device_release_direct_mode(iio_dev);
    
    return (ret < 0) ? ret : 0;
}


static ssize_t st_ais2120_get_sampling_frequency(struct device *dev,
                    struct device_attribute *attr,
                    char *buf)
{
    struct st_ais2120_hw *hw = iio_priv(dev_get_drvdata(dev));

    return sprintf(buf, "%d\n", hw->fir);
}

ssize_t st_ais2120_set_sampling_frequency(struct device * dev,
                    struct device_attribute * attr,
                    const char *buf, size_t count)
{
    int i, ret;
    unsigned int fir;
    u8 value;
    struct st_ais2120_hw *hw = iio_priv(dev_get_drvdata(dev));

    ret = kstrtoint(buf, 10, &fir);
    if (ret < 0)
        return ret;
    
    if (fir == hw->fir) {
        return count;
    }

    for(i=0; i<FIR_BANDWITH_NUM; i++) {
        if (ais2120_fir[i] == fir) {
            hw->fir = fir;
            break;
        }
    }

    if (i == FIR_BANDWITH_NUM)
        return -EINVAL;

    if (i == 0)
        value = 0x0;
    else
        value = 0x5;

    ret = st_ais2120_write_with_mask(dev, REG_CONFIG_ADDR, 0xF0, value);
    
    return (ret < 0) ? ret : count;
}


static ssize_t st_ais2120_get_sampling_frequency_avail(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d %d\n", ais2120_fir[0], ais2120_fir[1]);
}

static IIO_DEV_ATTR_SAMP_FREQ(S_IWUSR | S_IRUGO,
                    st_ais2120_get_sampling_frequency,
                    st_ais2120_set_sampling_frequency);
static IIO_DEV_ATTR_SAMP_FREQ_AVAIL(st_ais2120_get_sampling_frequency_avail);

#if 0
static ssize_t st_ais2120_get_scale_avail(struct device *device,
					  struct device_attribute *attr,
					  char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "0.%06d\n", (int)0);
}

static IIO_DEVICE_ATTR(in_accel_scale_available, 0444,
		       st_ais2120_get_scale_avail, NULL, 0);
#endif

static struct attribute *st_ais2120_attributes[] = {
    &iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	/*&iio_dev_attr_in_accel_scale_available.dev_attr.attr,*/
	NULL,
};

static const struct attribute_group st_ais2120_attribute_group = {
	.attrs = st_ais2120_attributes,
};

static const struct iio_info st_ais2120_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_ais2120_attribute_group,
	.read_raw = st_ais2120_read_raw,
    .debugfs_reg_access = &st_ais2120_reg_access,
};

static int st_ais2120_check_whoami(struct st_ais2120_hw *hw)
{
	u8 data;
	int ret;

	ret = st_ais2120_spi_read(hw->dev, REG_WHOAMI_ADDR, sizeof(data), &data);
	if (ret < 0) {
		dev_err(hw->dev, "failed to read whoami register\n");
		return ret;
	}

	if (data != REG_WHOAMI_VAL) {
		dev_err(hw->dev, "wrong whoami {%02x-%02x}\n",
			data, REG_WHOAMI_VAL);
		/*return -ENODEV;*/
	}else {
        printk("[ais2120] DEVID = %02X detected\n", data);
    }
	return 0;
}


#if 0
static int st_ais2120_set_normal(struct st_ais2120_hw *hw)
{
    u8 data;
    data = 0x01;
    ret = st_ais2120_spi_write(hw->dev, REG_CTRL_0, 1, (u8 *)&data);
	if (ret < 0) {
		dev_err(hw->dev, "failed to spi write %02x register\n", addr);
		mutex_unlock(&hw->lock);
		return ret;
	}
    return 0;
}
#endif


static int st_ais2120_init_device(struct st_ais2120_hw *hw)
{
#if 0
	int err;

	err = st_ais2120_write_with_mask(hw, REG_CTRL1_ADDR,
					 REG_CTRL1_SW_RESET_MASK, 1);
	if (err < 0)
		return err;

	msleep(200);

	err = st_ais2120_write_with_mask(hw, REG_CTRL1_ADDR,
					 REG_CTRL1_BDU_MASK, 1);
	if (err < 0)
		return err;
#endif
	return 0;
}

static int st_ais2120_spi_probe(struct spi_device *spi)
{
	struct st_ais2120_hw *hw;
	struct iio_dev *iio_dev;
	int err;

	iio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*hw));
	if (!iio_dev)
		return -ENOMEM;

	spi_set_drvdata(spi, iio_dev);

	iio_dev->channels = st_ais2120_channels;
	iio_dev->num_channels = ARRAY_SIZE(st_ais2120_channels);
	iio_dev->modes = INDIO_DIRECT_MODE;
	iio_dev->info = &st_ais2120_info;
	iio_dev->dev.parent = &spi->dev;
	iio_dev->name = spi->modalias;

	hw = iio_priv(iio_dev);

	mutex_init(&hw->lock);

	hw->dev = &spi->dev;
	hw->name = spi->modalias;
	hw->irq = spi->irq;
    hw->self_test_status = 0; // 0 PASS, 1: x positive, 2: x negative, 5: y pos, 6: y neg 
    hw->fir = ais2120_fir[0]; // FIR bandwidth

	err = st_ais2120_check_whoami(hw);
	if (err < 0)
		return err;

	err = st_ais2120_init_device(hw);
	if (err < 0)
		return err;

	return devm_iio_device_register(hw->dev, iio_dev);
}

static const struct of_device_id st_ais2120_spi_of_match[] = {
	{
		.compatible = "st,ais2120",
		.data = AIS2120_DEV_NAME,
	},
	{},
};
MODULE_DEVICE_TABLE(of, st_ais2120_spi_of_match);

static const struct spi_device_id st_ais2120_spi_id_table[] = {
	{ AIS2120_DEV_NAME },
	{},
};
MODULE_DEVICE_TABLE(spi, st_ais2120_spi_id_table);

static struct spi_driver st_ais2120_driver = {
	.driver = {
		.name = "st_ais2120",
		.of_match_table = of_match_ptr(st_ais2120_spi_of_match),
	},
	.probe = st_ais2120_spi_probe,
	.id_table = st_ais2120_spi_id_table,
};
module_spi_driver(st_ais2120_driver);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("STMicroelectronics st_ais2120 sensor driver");
MODULE_LICENSE("GPL v2");
