 /* experimental IIO driver for LPS331AP*/ 
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/bitops.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/triggered_buffer.h>


struct lps331ap_data {
  struct i2c_client *client;
  struct mutex lock;
  int pressure_scale;
  int temperature_scale;
};

static const struct iio_chan_spec lps331ap_channels[] = {
  {
    .type = IIO_PRESSURE,
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW), 
    .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), 
    .scan_index = 0,
    .scan_type = {
      .sign = 's',
      .realbits = 24,
      .storagebits = 32,
      .shift = 8,
      .endianness = IIO_LE,
    },
  }, 
  {
    .type = IIO_TEMP,
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW), 
    .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), 
    .scan_index = 1,
    .scan_type = {
      .sign = 's',
      .realbits = 16,
      .storagebits = 16,
      .shift = 0,
      .endianness = IIO_LE,
    },
  },  
  IIO_CHAN_SOFT_TIMESTAMP(2),
};

static int lps331ap_read_measurements(struct i2c_client *client, 
   u8 reg_address, int *pressure, int *temperature)
{
  int ret;
  u8 buf[7] = {0};

  buf[0] = 0x80 | reg_address;	// automatic register address increasment
  ret = i2c_master_send(client, buf, 1);
  if (ret < 0)
    return ret;

  ret = i2c_master_recv(client, buf, 5);
  if (ret < 0)
  { 
		return ret;
  }
  *pressure = ((buf[2] << 16) | (buf[1] << 8) | buf[0] ) * 24;
  *temperature = ((int)((short)((buf[4] << 8) | buf[3])) + 20400 ) * 2; // 42.5 + RAW_TEMP / 480 = (RAW_TEMP + 20400 ) /480
  return ret;
}

static int lps331ap_read_raw(struct iio_dev *iio_dev,
      struct iio_chan_spec const *channel, 
      int *val, int *val2, long mask)
{
	struct lps331ap_data *data = iio_priv(iio_dev);
	int err = 0;
	int pressure = 1;
	int temperature = 2;
	int scale = 0;

	
	switch (mask) 
	{
		case IIO_CHAN_INFO_RAW:
			
			mutex_lock(&data->lock);
			err = lps331ap_read_measurements(data->client, 0x28 , &pressure, &temperature);
			switch (channel->type) 
			{
				scale = data->pressure_scale;
				case IIO_PRESSURE:
					*val = pressure;
					break;
				case IIO_TEMP:
					*val = temperature;
					break;
				default:
					return -EINVAL;
			}
			mutex_unlock(&data->lock);
			if (err < 0)
				goto read_error;
			return IIO_VAL_INT;
		case IIO_CHAN_INFO_SCALE:
			*val = 0;
			switch (channel->type) 
			{
				case IIO_PRESSURE:
					*val2 = data->pressure_scale;
					break;
				case IIO_TEMP:
					*val2 = data->temperature_scale;
					break;
				default:
					return -EINVAL;
			}
			return IIO_VAL_INT_PLUS_MICRO;
		default:
			return -EINVAL;
	}

	read_error:
	return err;
}

static const struct iio_info lps331ap_info = {
  .read_raw = lps331ap_read_raw,
  .driver_module = THIS_MODULE,
};

static int lps331ap_init(struct i2c_client *client)
{
	int ret;
	struct iio_dev *indio_dev;
	struct lps331ap_data *data;

	ret = i2c_smbus_write_byte_data(client, 0x10, 0x7A);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to write resolution register.\n");
		return ret;
	}
	ret = i2c_smbus_write_byte_data(client, 0x20, 0xE0);		// power up and configure to 12.5Hz output rate
	if (ret < 0) {
		dev_err(&client->dev, "Failed to write ctrl register 1.\n");
		return ret;
	}

	indio_dev = i2c_get_clientdata(client);
	data = iio_priv(indio_dev);

	data->pressure_scale = 1000; // milipascal per lsb
	data->temperature_scale = 1000; // mili C
	return 0;
}

static int lps331ap_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	
  struct iio_dev *indio_dev;
  struct lps331ap_data *data;
  struct iio_buffer *buffer;
  int ret;

  
  if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
    ret = -ENODEV;
    goto error_ret;
  }

  ret = i2c_smbus_read_byte_data(client, 0x0F);
  if (ret < 0) {
    ret = -EINVAL;
    goto error_ret;
  }
  if (ret == 0xBB) {
    dev_info(&client->dev, "LPS331AP found.\n");
  }  else {
    dev_err(&client->dev, "No LPS331AP sensor found.\n");
    ret = -ENODEV;
    goto error_ret;
  }

  indio_dev = iio_device_alloc(sizeof(*data));
  if (!indio_dev) {
    ret = -ENOMEM;
    goto error_ret;
  }
  
  data = iio_priv(indio_dev);
  mutex_init(&data->lock);
  i2c_set_clientdata(client, indio_dev);
  data->client = client;

  indio_dev->dev.parent = &client->dev;
  indio_dev->name = dev_name(&client->dev);
  indio_dev->modes = INDIO_DIRECT_MODE;


	ret = lps331ap_init(client);
	indio_dev->info = &lps331ap_info;
	indio_dev->channels = lps331ap_channels;
	indio_dev->num_channels = ARRAY_SIZE(lps331ap_channels);

  if (ret < 0)
    goto error_free_device;

	
  ret = iio_device_register(indio_dev);
  if (ret < 0)
    goto error_free_device;

  return 0;

error_free_device:
  iio_device_free(indio_dev);
error_ret:
  return ret;
}


static int lps331ap_remove(struct i2c_client *client)
{
  struct iio_dev *indio_dev = i2c_get_clientdata(client);
  iio_device_unregister(indio_dev);
  iio_device_free(indio_dev);
  dev_info(&client->dev, "LPS331AP sensor driver removed.");
  return 0;
}

static const struct i2c_device_id lps331ap_id[] = {
  { "lps331ap", 0 },
  { }
};
MODULE_DEVICE_TABLE(i2c, lps331ap_id);

static struct i2c_driver lps331ap_driver = {
  .driver = {
    .name = "lps331ap",
    .owner = THIS_MODULE,
  },
  .probe = lps331ap_probe,
  .remove = lps331ap_remove,
  .id_table = lps331ap_id,
};
module_i2c_driver(lps331ap_driver);

MODULE_AUTHOR("Yuncheng Sng <yuncheng.song@gmail.com>");
MODULE_DESCRIPTION("LPS331AP presseure and temperature sensor");
MODULE_LICENSE("GPL");