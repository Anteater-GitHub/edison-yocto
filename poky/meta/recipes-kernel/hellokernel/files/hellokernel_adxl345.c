/* experimental IIO driver for ADXL345*/ 
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

#define ADXL345_OUT_X_L_A_REG	0x32
#define ADSL345_DATA_FORMAT 	0x31
#define ADXL345_FS_MASK           (0x03 << 0)
#define ADXL345_FS_2G_VAL         (0x00 << 0)
#define ADXL345_FS_4G_VAL         (0x01 << 0)
#define ADXL345_FS_8G_VAL         (0x02 << 0)
#define ADXL345_FS_16G_VAL        (0x03 << 0)
#define ADXL345_FS_2G_GAIN        3900     /* ug/LSB  */
#define ADXL345_FS_4G_GAIN        3900    /* ug/LSB  */
#define ADXL345_FS_8G_GAIN        3900    /* ug/LSB  */
#define ADXL345_FS_16G_GAIN       3900    /* ug/LSB  */

enum { SCAN_INDEX_ACCEL_X, SCAN_INDEX_ACCEL_Y, SCAN_INDEX_ACCEL_Z};
enum { ACCEL };

struct adxl345_data {
  struct i2c_client *client;
  struct mutex lock;
  int sensor_type;
  int accel_scale;
};

struct sensor_fs_avl {
  unsigned int num;
  u8 value;
  unsigned int gain;
};

static const struct sensor_fs_avl adxl345_fs_avl[4] = {
  {2,  ADXL345_FS_2G_VAL,  ADXL345_FS_2G_GAIN},
  {4,  ADXL345_FS_4G_VAL,  ADXL345_FS_4G_GAIN},
  {8,  ADXL345_FS_8G_VAL,  ADXL345_FS_8G_GAIN},
  {16, ADXL345_FS_16G_VAL, ADXL345_FS_16G_GAIN},
};

static ssize_t adxl345_show_scale_avail(struct device *dev,
        struct device_attribute *attr, char *buf)
{

  size_t len = 0;
  int n;
  const struct sensor_fs_avl (*avl)[];

  if (strcmp(attr->attr.name, "in_accel_scale_available") == 0) {
    avl = &adxl345_fs_avl;
    n = ARRAY_SIZE(adxl345_fs_avl);
  } else {
    return -EINVAL;
  }

  while (n-- > 0)
    len += scnprintf(buf + len, PAGE_SIZE - len,
        "0.%06u ", (*avl)[n].gain);
  buf[len - 1] = '\n';
  
  return len;
}

static IIO_DEVICE_ATTR(in_accel_scale_available, S_IRUGO, adxl345_show_scale_avail, NULL, 0);

static struct attribute *adxl345_attributes[] = {
  &iio_dev_attr_in_accel_scale_available.dev_attr.attr,
  NULL
};

static const struct attribute_group adxl345_group = {
  .attrs = adxl345_attributes,
};

static const struct iio_chan_spec adxl345_channels[] = {
  {
    .type = IIO_ACCEL,
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW), 
    .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), 
    .modified = 1,
    .channel2 = IIO_MOD_X,
    .scan_index = SCAN_INDEX_ACCEL_X,
    .scan_type = {
      .sign = 's',
      .realbits = 16,
      .storagebits = 16,
      .shift = 0,
      .endianness = IIO_LE,
    },
  }, {
    .type = IIO_ACCEL,
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW), 
    .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), 
    .modified = 1,
    .channel2 = IIO_MOD_Y,
    .scan_index = SCAN_INDEX_ACCEL_Y,
    .scan_type = {
      .sign = 's',
      .realbits = 16,
      .storagebits = 16,
      .shift = 0,
      .endianness = IIO_LE,
    },
  }, {
    .type = IIO_ACCEL,
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW), 
    .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), 
    .modified = 1,
    .channel2 = IIO_MOD_Z,
    .scan_index = SCAN_INDEX_ACCEL_Z,
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

static int adxl345_read_measurements(struct i2c_client *client, 
   u8 reg_address, s16 *x, s16 *y, s16 *z)
{
  int ret;
  u8 buf[6] = {0};

  buf[0] = 0x00 | reg_address;
  ret = i2c_master_send(client, buf, 1);
  if (ret < 0)
    return ret;

  ret = i2c_master_recv(client, buf, 6);
  if (ret < 0)
    return ret;

  *x = (buf[1] << 8) | buf[0];
  *y = (buf[3] << 8) | buf[2];
  *z = (buf[5] << 8) | buf[4];
  return ret;
}

static int adxl345_read_raw(struct iio_dev *iio_dev,
      struct iio_chan_spec const *channel, 
      int *val, int *val2, long mask)
{
  struct adxl345_data *data = iio_priv(iio_dev);
  int err = 0;
  s16 x = 0, y = 0, z = 0;
  int scale = 0;

  switch (mask) {
  case IIO_CHAN_INFO_RAW:
    mutex_lock(&data->lock);
    switch (channel->type) {
    case IIO_ACCEL:
      err = adxl345_read_measurements(data->client, 
          ADXL345_OUT_X_L_A_REG, &x, &y, &z);
      scale = data->accel_scale;
      break;
    default:
      return -EINVAL;
    }
    mutex_unlock(&data->lock);
    if (err < 0)
      goto read_error;

    switch (channel->channel2) {
    case IIO_MOD_X:
      *val = x;
      break;
    case IIO_MOD_Y:
      *val = y;
      break;
    case IIO_MOD_Z:
      *val = z;
      break;
    }
    return IIO_VAL_INT;
  case IIO_CHAN_INFO_SCALE:
    *val = 0;
    switch (channel->type) {
    case IIO_ACCEL:
      *val2 = data->accel_scale;
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


static int adxl345_write_config(struct i2c_client *client,
    u8 reg_address, u8 mask, u8 value)
{
  u8 reg;
  s32 ret;
  ret = i2c_smbus_read_byte_data(client, reg_address);
  if (ret < 0) 
    return -EINVAL;

  reg = (u8)ret;
  reg &= ~mask;
  reg |= value;

  ret = i2c_smbus_write_byte_data(client, reg_address, reg); 

  return ret;  
} 


static int adxl345_write_raw(struct iio_dev *indio_dev,
      struct iio_chan_spec const *channel,
      int val, int val2, long mask)
{
  struct adxl345_data *data = iio_priv(indio_dev);
  struct i2c_client *client = data->client;
  const struct sensor_fs_avl (*avl)[];
  int n, i, ret;
  u8 reg_address, reg_mask, new_value;
  int *scale_in_data;

  mutex_lock(&data->lock);
  switch (mask) {
  case IIO_CHAN_INFO_SCALE:
    dev_info(&client->dev, "Vals %d %d\n", val, val2);
    switch (channel->type) {
    case IIO_ACCEL:
      avl = &adxl345_fs_avl;
      n = ARRAY_SIZE(adxl345_fs_avl);
      reg_address = ADSL345_DATA_FORMAT;
      reg_mask = ADXL345_FS_MASK;
      scale_in_data = &(data->accel_scale);
      break;
    default:
      ret = -EINVAL;
      goto done;
    }
    ret = -EINVAL;
    for (i = 0; i < n; i++) {
      if ((*avl)[i].gain == val2) {
        ret = 0;
        new_value = (*avl)[i].value;
        break;
      }
    }
    if (ret < 0)
      goto done;

    ret = adxl345_write_config(client, reg_address, reg_mask, new_value);
    if (ret < 0)
      goto done;

    *scale_in_data = (*avl)[i].gain;
    break;
  default:
    ret = -EINVAL;
  }

done:
  mutex_unlock(&data->lock);
  return ret;
}
/*
static irqreturn_t adxl345_trigger_h(int irq, void *p)
{
	
  struct iio_poll_func *pf = p;
  struct iio_dev *indio_dev = pf->indio_dev;
  struct adxl345_data *data = iio_priv(indio_dev);
  u32 *buf_data;
  int i, j;
  s16 x1, y1, z1, x2, y2, z2;
  int err;

  buf_data = kmalloc(indio_dev->scan_bytes, GFP_KERNEL);
  if (!buf_data)
    goto done;

  mutex_lock(&data->lock);
  if (!bitmap_empty(indio_dev->active_scan_mask, indio_dev->masklength)) {

   if (data->sensor_type == ACCEL) {
      err = adxl345_read_measurements(data->client, 
          ADXL345_OUT_X_L_A_REG, &x1, &y1, &z1);
      if (err < 0)
        goto free_buf;
    } else 
      goto free_buf;
    
    for (i = 0, j = 0;
         i < bitmap_weight(indio_dev->active_scan_mask, indio_dev->masklength);
         i++, j++) {
      j = find_next_bit(indio_dev->active_scan_mask, indio_dev->masklength, j);

       switch (j) {
          case SCAN_INDEX_ACCEL_X:
            buf_data[i] = x1;
            break;
          case SCAN_INDEX_ACCEL_Y:
            buf_data[i] = y1;
            break;
          case SCAN_INDEX_ACCEL_Z:
            buf_data[i] = z1;
            break;
          default:
            break;
        }
      }
    }

  iio_push_to_buffers(indio_dev, buf_data);

free_buf:
  kfree(buf_data);
  mutex_unlock(&data->lock);

done:
  iio_trigger_notify_done(indio_dev->trig);

  return IRQ_HANDLED;
  
}
*/

static const struct iio_info adxl345_info = {
  .attrs = &adxl345_group,
  .read_raw = adxl345_read_raw,
  .write_raw = adxl345_write_raw,
  .driver_module = THIS_MODULE,
};

static int adxl345_init(struct i2c_client *client)
{
  int ret;
  struct iio_dev *indio_dev;
  struct adxl345_data *data;

  ret = i2c_smbus_write_byte_data(client, 0x2c, 0x0A);
  if (ret < 0) {
    dev_err(&client->dev, "Failed to write sample rate register.\n");
    return ret;
  }
  ret = i2c_smbus_write_byte_data(client, 0x2D, 0x08);
  if (ret < 0) {
    dev_err(&client->dev, "Failed to write power register 4.\n");
    return ret;
  }
  ret = i2c_smbus_write_byte_data(client, 0x31, 0x08);
  if (ret < 0) {
    dev_err(&client->dev, "Failed to write power register 4.\n");
    return ret;
  }
  indio_dev = i2c_get_clientdata(client);
  data = iio_priv(indio_dev);

  data->accel_scale = ADXL345_FS_2G_GAIN;

  return 0;
}

static const struct iio_buffer_setup_ops adxl345_buffer_setup_ops = {
  .postenable = &iio_triggered_buffer_postenable,
  .predisable = &iio_triggered_buffer_predisable,
};


static int adxl345_probe(struct i2c_client *client,
    const struct i2c_device_id *id)
{
	
  struct iio_dev *indio_dev;
  struct adxl345_data *data;
  struct iio_buffer *buffer;
  int sensor_type;
  int ret;

  
  if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
    ret = -ENODEV;
    goto error_ret;
  }

  ret = i2c_smbus_read_byte_data(client, 0x00);
  if (ret < 0) {
    ret = -EINVAL;
    goto error_ret;
  }
  if (ret == 0xE5) {
    dev_info(&client->dev, "ADXL345 found.\n");
    sensor_type = ACCEL;
  }  else {
    dev_err(&client->dev, "No ADXL345 sensor found.\n");
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
  data->sensor_type = sensor_type;

  indio_dev->dev.parent = &client->dev;
  indio_dev->name = dev_name(&client->dev);
  indio_dev->modes = INDIO_DIRECT_MODE;


	ret = adxl345_init(client);
	indio_dev->info = &adxl345_info;
	indio_dev->channels = adxl345_channels;
	indio_dev->num_channels = ARRAY_SIZE(adxl345_channels);

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


static int adxl345_remove(struct i2c_client *client)
{
  struct iio_dev *indio_dev = i2c_get_clientdata(client);
  iio_device_unregister(indio_dev);
  iio_device_free(indio_dev);
  dev_info(&client->dev, "Driver removed.");
  return 0;
}

static const struct i2c_device_id adxl345_id[] = {
  { "adxl345", 0 },
  { }
};
MODULE_DEVICE_TABLE(i2c, adxl345_id);

static struct i2c_driver adxl345_driver = {
  .driver = {
    .name = "adxl345",
    .owner = THIS_MODULE,
  },
  .probe = adxl345_probe,
  .remove = adxl345_remove,
  .id_table = adxl345_id,
};
module_i2c_driver(adxl345_driver);

MODULE_AUTHOR("Matija Podravec <matija_podravec@fastmail.fm>");
MODULE_DESCRIPTION("LSM9DS0 gyroscope, accelerometer, and magnetometer sensor");
MODULE_LICENSE("GPL");