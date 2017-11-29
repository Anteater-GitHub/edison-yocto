/* experimental IIO driver for BME280*/ 
#include <linux/kernel.h>
#include <asm-generic/div64.h>
#include <asm/div64.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/bitops.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/triggered_buffer.h>

struct bme280_caldata{
	int	dig_T1	;
	int	dig_T2	;
	int	dig_T3	;
	int	dig_P1	;
	int	dig_P2	;
	int	dig_P3	;
	int	dig_P4	;
	int	dig_P5	;
	int	dig_P6	;
	int	dig_P7	;
	int	dig_P8	;
	int	dig_P9	;
	int	dig_H1	;
	int	dig_H2	;
	int	dig_H3	;
	int	dig_H4	;
	int	dig_H5	;
	int	dig_H6	;
};
struct bme280_data {
  struct i2c_client *client;
  struct bme280_caldata par;
  struct mutex lock;
  int pressure_scale;
  int temperature_scale;
  int humidity_scale;
};

static const struct iio_chan_spec bme280_channels[] = {
  {
    .type = IIO_PRESSURE,
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW), 
    .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), 
    .scan_index = 0,
    .scan_type = {
      .sign = 's',
      .realbits = 20,
      .storagebits = 32,
      .shift = 12,
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
      .realbits = 20,
      .storagebits = 32,
      .shift = 12,
      .endianness = IIO_LE,
    },
  },  
  {
    .type = IIO_HUMIDITYRELATIVE,
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW), 
    .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), 
    .scan_index = 2,
    .scan_type = {
      .sign = 's',
      .realbits = 16,
      .storagebits = 16,
      .shift = 0,
      .endianness = IIO_LE,
    },
  },  
  IIO_CHAN_SOFT_TIMESTAMP(3),
};

uint32_t __attribute__((weak)) div64_32(uint64_t *n, uint32_t base)
{
        uint64_t rem = *n;
        uint64_t b = base;
        uint64_t res, d = 1;
        uint32_t high = rem >> 32;

        /* Reduce the thing a bit first */
        res = 0;
        if (high >= base) {
                high /= base;
                res = (uint64_t) high << 32;
                rem -= (uint64_t) (high*base) << 32;
        }

        while ((int64_t)b > 0 && b < rem) {
                b = b+b;
                d = d+d;
        }

        do {
                if (rem >= b) {
                        rem -= b;
                        res += d;
                }
                b >>= 1;
                d >>= 1;
        } while (d);

        *n = res;
        return rem;
}


static int bme280_read_measurements(struct i2c_client *client, 
   u8 reg_address, int *pressure, int *temperature, int *humidity)
{
  int ret;
  u8 buf[8] = {0};

  buf[0] = 0x00 | reg_address;	// automatic register address increasment
  ret = i2c_master_send(client, buf, 1);
  if (ret < 0)
    return ret;

  ret = i2c_master_recv(client, buf, 8);
  if (ret < 0)
    return ret;
  *humidity = ((buf[6] << 8) | buf[7]) & 0xFFFF;
  *pressure = ( (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4)) &0xFFFFF;
  *temperature = ( (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4)) &0xFFFFF;
  return ret;
}

static int bme280_read_raw(struct iio_dev *iio_dev,
      struct iio_chan_spec const *channel, 
      int *val, int *val2, long mask)
{
	struct bme280_data *data = iio_priv(iio_dev);
	int err = 0;
	int pressure = 1;
	int temperature = 2;
	int humidity = 3;
	int scale = 0;
	int var1, var2, t_fine; // local variable for readout compensation
	unsigned int p;
	switch (mask) 
	{
		case IIO_CHAN_INFO_RAW:
		
			err = bme280_read_measurements(data->client, 0xF7, &pressure, &temperature, &humidity);
			
			var1=( ( (temperature >> 3 ) - (data->par.dig_T1 << 1) ) *  data->par.dig_T2 )>>11;
			var2 = (((((temperature>>4) - data->par.dig_T1) * ((temperature>>4) - data->par.dig_T1)) >> 12) * data->par.dig_T3) >> 14;
			t_fine = var1 + var2;
			temperature = ((t_fine * 5 + 128) >> 8 )*10; // in 0.001 deg C
	
			var1 = (t_fine>>1) - 64000;
			var2 = (((var1>>2) * (var1>>2)) >> 11 ) * data->par.dig_P6;
			var2 = var2 + ((var1*data->par.dig_P5)<<1);
			var2 = (var2>>2)+(data->par.dig_P4<<16);
			var1 = (((data->par.dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((data->par.dig_P2 * var1)>>1))>>18;
			var1 = (((32768+var1))*data->par.dig_P1)>>15;

			if (var1 == 0)
			{
				p = 0 ; // avoid exception caused by division by zero
			}
			else
			{
				p = ((unsigned int)(1048576 - pressure) - (unsigned int)(var2>>12))*3125;
				if (p < 0x80000000)
				{
					p = (p << 1) / ((unsigned int)var1);
				}
				else
				{
					p = ( p / (unsigned int)var1) * 2;
				}
				var1 = (data->par.dig_P9 * ((int)(((p>>3) * (p>>3))>>13)))>>12;
				var2 = (((int)(p>>2)) * data->par.dig_P8)>>13;
				p = (unsigned int)((int)p + ((var1 + var2 + data->par.dig_P7) >> 4));
			}
			pressure = (int)p * 1000;

						
			//printk(  "pressure raw: %lld\n", pressure);
			//printk(  "t_fine: %d\n", t_fine);
			//printk(  "humidity raw: %d\n", humidity);

			var1 = (t_fine - 76800);
			var1 = ((((humidity << 14) - (data->par.dig_H4 << 20) - data->par.dig_H5 * var1) + 16384) >> 15) * 	(((((((var1 * data->par.dig_H6) >> 10) * (((var1 * data->par.dig_H3) >> 11) + 32768)) >> 10) + 2097152) * data->par.dig_H2 + 8192) >> 14);
			var1 = var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * data->par.dig_H1) >> 4 );
			var1 = var1 < 0 ? 0 : var1;
			var1 = var1 > 419430400 ? 419430400 : var1;
			humidity = (int)((var1>>12))*1000/1024; // in 0.001% RH
		
				
			mutex_lock(&data->lock);
			switch (channel->type) 
			{
				
			//	var1=( ( (temperature >> 3 ) - (data->par.dig_T1 << 1) ) *  data->par.dig_T2 )>>11;
			//	var1= ( ( ( ((temperature >> 4 )- data->par.dig_T1 ) * ((temperature >> 4 )- data->par.dig_T1 ) ) >> 12 ) * data->par.dig_T3) >> 14;
			//	t_fine = var1 + var2;
			//	temperature = ((t_fine * 5 + 128 ) >> 8 ) *10; // unit: mili C
				/*				
				

				

				*/
				case IIO_PRESSURE:
					*val = pressure;
					break;
				case IIO_TEMP:
					*val =  temperature;
					break;
				case IIO_HUMIDITYRELATIVE:
					*val = humidity;
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
				case IIO_HUMIDITYRELATIVE:
					*val2 = data->humidity_scale;
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

static const struct iio_info bme280_info = {
  .read_raw = bme280_read_raw,
  .driver_module = THIS_MODULE,
};

static int bme280_init(struct i2c_client *client)
{
	int ret;
	struct iio_dev *indio_dev;
	struct bme280_data *data;

	ret = i2c_smbus_write_byte_data(client, 0xF2, 0x01);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to write humidity control register.\n");
		return ret;
	}
	ret = i2c_smbus_write_byte_data(client, 0xF4, 0xAB);		// power up and configure to 12.5Hz output rate
	if (ret < 0) {
		dev_err(&client->dev, "Failed to write pressure and temperature ctrl register 1.\n");
		return ret;
	}
	ret = i2c_smbus_write_byte_data(client, 0xF5, 0x10);		
	if (ret < 0) {
		dev_err(&client->dev, "Failed to write ctrl register 1.\n");
		return ret;
	}

	indio_dev = i2c_get_clientdata(client);
	data = iio_priv(indio_dev);

	// read calibration data
	char addr;
	char buf[33] = {0};
	addr = 0x00 | 0x88;	// calib00 starting address
	ret = i2c_master_send(client, &addr, 1);
	if (ret < 0)
		return ret;
	ret = i2c_master_recv(client, &buf[0], 26);
	if (ret < 0)
		return ret;
	addr = 0x00 | 0xE1;	// calib26 starting address
	ret = i2c_master_send(client, &addr, 1);
	if (ret < 0)
		return ret;
	ret = i2c_master_recv(client, &buf[26], 7);
	if (ret < 0)
		return ret;
	data->par.dig_T1 = ((((int)buf[1])& 0xFF) << 8 ) | (((int)buf[0])& 0xFF);	// unsigned short
	data->par.dig_T2 = (((int)buf[3]) << 8 ) | (((int)buf[2])& 0xFF);			// signed short
	data->par.dig_T3 = (((int)buf[5]) << 8 ) | (((int)buf[4])& 0xFF);			// signed short
	data->par.dig_P1 = ((((int)buf[7])& 0xFF) << 8 ) | (((int)buf[6])& 0xFF);	// unsigned short
	data->par.dig_P2 = (((int)buf[9]) << 8 ) | (((int)buf[8])& 0xFF);			// signed short
	data->par.dig_P3 = (((int)buf[11]) << 8 ) | (((int)buf[10])& 0xFF);			// signed short
	data->par.dig_P4 = (((int)buf[13]) << 8 ) | (((int)buf[12])& 0xFF);			// signed short
	data->par.dig_P5 = (((int)buf[15]) << 8 ) | (((int)buf[14])& 0xFF);			// signed short
	data->par.dig_P6 = (((int)buf[17]) << 8 ) | (((int)buf[16])& 0xFF);			// signed short
	data->par.dig_P7 = (((int)buf[19]) << 8 ) | (((int)buf[18])& 0xFF);			// signed short
	data->par.dig_P8 = (((int)buf[21]) << 8 ) | (((int)buf[20])& 0xFF);			// signed short
	data->par.dig_P9 = (((int)buf[23]) << 8 ) | (((int)buf[22])& 0xFF);			// signed short
	data->par.dig_H1 = ((int)buf[25])& 0xFF;									// unsigned char
	data->par.dig_H2 = (((int)buf[27]) << 8 ) | (((int)buf[26])& 0xFF);			// signed short
	data->par.dig_H3 = ((int)buf[28])& 0xFF;									// unsigned char
	data->par.dig_H4 = (((int)buf[29])<< 4 ) | (((int)buf[30])& 0xF);			// signed short
	data->par.dig_H5 = (((int)buf[31]) << 4 ) | ((((int)buf[30])& 0xF0) >> 4);	// signed short
	data->par.dig_H6 = (int)buf[32];											// signed char
	/*
	dev_info(&client->dev, "data->par.dig_T1: %d\n", data->par.dig_T1);
	dev_info(&client->dev, "data->par.dig_T2: %d\n", data->par.dig_T2);
	dev_info(&client->dev, "data->par.dig_T3: %d\n", data->par.dig_T3);
	dev_info(&client->dev, "data->par.dig_P1: %d\n", data->par.dig_P1);	
	dev_info(&client->dev, "data->par.dig_P2: %d\n", data->par.dig_P2);
	dev_info(&client->dev, "data->par.dig_P3: %d\n", data->par.dig_P3);
	dev_info(&client->dev, "data->par.dig_P4: %d\n", data->par.dig_P4);
	dev_info(&client->dev, "data->par.dig_P5: %d\n", data->par.dig_P5);
	dev_info(&client->dev, "data->par.dig_P6: %d\n", data->par.dig_P6);
	dev_info(&client->dev, "data->par.dig_P7: %d\n", data->par.dig_P7);
	dev_info(&client->dev, "data->par.dig_P8: %d\n", data->par.dig_P8);
	dev_info(&client->dev, "data->par.dig_P9: %d\n", data->par.dig_P9);
	dev_info(&client->dev, "data->par.dig_H1: %d\n", data->par.dig_H1);	
	dev_info(&client->dev, "data->par.dig_H2: %d\n", data->par.dig_H2);
	dev_info(&client->dev, "data->par.dig_H3: %d\n", data->par.dig_H3);
	dev_info(&client->dev, "data->par.dig_H4: %d\n", data->par.dig_H4);
	dev_info(&client->dev, "data->par.dig_H5: %d\n", data->par.dig_H5);
	dev_info(&client->dev, "data->par.dig_H6: %d\n", data->par.dig_H6);
*/
	data->pressure_scale = 1000; // 0.001 pa
	data->temperature_scale = 1000; // 0.001 C
	data->humidity_scale = 1000; // 0.001 % RH
	return 0;
}

static int bme280_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	
  struct iio_dev *indio_dev;
  struct bme280_data *data;
  struct iio_buffer *buffer;
  int ret;

  
  if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
    ret = -ENODEV;
    goto error_ret;
  }

  ret = i2c_smbus_read_byte_data(client, 0xD0);
  if (ret < 0) {
    ret = -EINVAL;
    goto error_ret;
  }
  if (ret == 0x60) {
    dev_info(&client->dev, "BME280 found.\n");
  }  else {
    dev_err(&client->dev, "No BME280 sensor found.\n");
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


	ret = bme280_init(client);
	indio_dev->info = &bme280_info;
	indio_dev->channels = bme280_channels;
	indio_dev->num_channels = ARRAY_SIZE(bme280_channels);

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


static int bme280_remove(struct i2c_client *client)
{
  struct iio_dev *indio_dev = i2c_get_clientdata(client);
  iio_device_unregister(indio_dev);
  iio_device_free(indio_dev);
  dev_info(&client->dev, "BME280 sensor driver removed.");
  return 0;
}

static const struct i2c_device_id bme280_id[] = {
  { "bme280", 0 },
  { }
};
MODULE_DEVICE_TABLE(i2c, bme280_id);

static struct i2c_driver bme280_driver = {
  .driver = {
    .name = "bme280",
    .owner = THIS_MODULE,
  },
  .probe = bme280_probe,
  .remove = bme280_remove,
  .id_table = bme280_id,
};
module_i2c_driver(bme280_driver);

MODULE_AUTHOR("Yuncheng Song <yuncheng.song@gmail.com>");
MODULE_DESCRIPTION("BME280 presseure and temperature sensor");
MODULE_LICENSE("GPL");