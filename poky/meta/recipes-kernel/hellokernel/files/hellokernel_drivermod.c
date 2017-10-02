#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/bitops.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/kfifo_buf.h>
struct adxl345_data {
  struct i2c_client *client;
  struct mutex lock;
  int sensor_type;
  int accel_scale;
};
static int adxl345_probe(struct i2c_client *client,
    const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	  struct adxl345_data *data;
	int ret;
	indio_dev = iio_device_alloc(sizeof(*data));
		ret = iio_device_register(indio_dev);
	  return 0;
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

MODULE_AUTHOR("Yuncheng Song");
MODULE_DESCRIPTION("ADXL345 accelerometer sensor");
MODULE_LICENSE("GPL");