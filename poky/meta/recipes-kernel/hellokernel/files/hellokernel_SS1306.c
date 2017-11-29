/*
 * Driver for the Solomon SSD1306 OLED controller
 *
 * Copyright 2012 Free Electrons
 *
 * Licensed under the GPLv2 or later.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/fb.h>
#include <linux/uaccess.h>
//#include <linux/of_device.h>
//#include <linux/of_gpio.h>
#include <linux/pwm.h>
#include <linux/delay.h>
#include <linux/lnw_gpio.h>
	
#define SSD1306FB_WIDTH					96
#define SSD1306FB_HEIGHT				16

#define SSD1306FB_DATA					0x40
#define SSD1306FB_COMMAND				0x80

#define SSD1306FB_CONTRAST				0x81
#define SSD1306FB_SEG_REMAP_ON				0xa1
#define SSD1306FB_DISPLAY_OFF				0xae
#define SSD1306FB_DISPLAY_ON				0xaf
#define SSD1306FB_START_PAGE_ADDRESS		0xb0

struct ssd1306fb_par {
	struct i2c_client *client;
	struct fb_info *info;
//	struct pwm_device *pwm;
//	u32 pwm_period;
//	int reset;
};

static struct fb_fix_screeninfo ssd1306fb_fix = {
	.id		= "Solomon SSD1306",
	.type		= FB_TYPE_PACKED_PIXELS,
	.visual		= FB_VISUAL_MONO10,
	.xpanstep	= 0,
	.ypanstep	= 0,
	.ywrapstep	= 0,
	.line_length	= SSD1306FB_WIDTH / 8,
	.accel		= FB_ACCEL_NONE,
};

static struct fb_var_screeninfo ssd1306fb_var = {
	.xres		= SSD1306FB_WIDTH,
	.yres		= SSD1306FB_HEIGHT,
	.xres_virtual	= SSD1306FB_WIDTH,
	.yres_virtual	= SSD1306FB_HEIGHT,
	.bits_per_pixel	= 1,
};

static int ssd1306fb_write_array(struct i2c_client *client, u8 type, u8 *cmd, u32 len)
{
	u8 *buf;
	int ret = 0;

	buf = kzalloc(len + 1, GFP_KERNEL);
	if (!buf) {
		dev_err(&client->dev, "Couldn't allocate sending buffer.\n");
		return -ENOMEM;
	}

	buf[0] = type;
	memcpy(buf + 1, cmd, len);

	ret = i2c_master_send(client, buf, len + 1);
	if (ret != len + 1) {
		dev_err(&client->dev, "Couldn't send I2C command.\n");
		goto error;
	}

error:
	kfree(buf);
	return ret;
}

static inline int ssd1306fb_write_cmd_array(struct i2c_client *client, u8 *cmd, u32 len)
{
	return ssd1306fb_write_array(client, SSD1306FB_COMMAND, cmd, len);
}

static inline int ssd1306fb_write_cmd(struct i2c_client *client, u8 cmd)
{
	return ssd1306fb_write_cmd_array(client, &cmd, 1);
}

static inline int ssd1306fb_write_data_array(struct i2c_client *client, u8 *cmd, u32 len)
{
	return ssd1306fb_write_array(client, SSD1306FB_DATA, cmd, len);
}

static inline int ssd1306fb_write_data(struct i2c_client *client, u8 data)
{
	return ssd1306fb_write_data_array(client, &data, 1);
}

static void ssd1306fb_update_display(struct ssd1306fb_par *par)
{
	u8 *vmem = par->info->screen_base;
	int i, j, k;

	/*
	 * The screen is divided in pages, each having a height of 8
	 * pixels, and the width of the screen. When sending a byte of
	 * data to the controller, it gives the 8 bits for the current
	 * column. I.e, the first byte are the 8 bits of the first
	 * column, then the 8 bits for the second column, etc.
	 *
	 *
	 * Representation of the screen, assuming it is 5 bits
	 * wide. Each letter-number combination is a bit that controls
	 * one pixel.
	 *
	 * A0 A1 A2 A3 A4
	 * B0 B1 B2 B3 B4
	 * C0 C1 C2 C3 C4
	 * D0 D1 D2 D3 D4
	 * E0 E1 E2 E3 E4
	 * F0 F1 F2 F3 F4
	 * G0 G1 G2 G3 G4
	 * H0 H1 H2 H3 H4
	 *
	 * If you want to update this screen, you need to send 5 bytes:
	 *  (1) A0 B0 C0 D0 E0 F0 G0 H0
	 *  (2) A1 B1 C1 D1 E1 F1 G1 H1
	 *  (3) A2 B2 C2 D2 E2 F2 G2 H2
	 *  (4) A3 B3 C3 D3 E3 F3 G3 H3
	 *  (5) A4 B4 C4 D4 E4 F4 G4 H4
	 */

	for (i = 0; i < (SSD1306FB_HEIGHT / 8); i++) {
		ssd1306fb_write_cmd(par->client, SSD1306FB_START_PAGE_ADDRESS + (i + 0));
		ssd1306fb_write_cmd(par->client, 0x00);
		ssd1306fb_write_cmd(par->client, 0x10);

		for (j = 0; j < SSD1306FB_WIDTH; j++) {
			u8 buf = 0;
			for (k = 0; k < 8; k++) {
				u32 page_length = SSD1306FB_WIDTH * i;
				u32 index = page_length + (SSD1306FB_WIDTH * k + j) / 8;
				u8 byte = *(vmem + index);
				u8 bit = byte & (1 << (j % 8));
				bit = bit >> (j % 8);
				buf |= bit << k;
			}
			ssd1306fb_write_data(par->client, buf);
		}
	}
}


static ssize_t ssd1306fb_write(struct fb_info *info, const char __user *buf, size_t count, loff_t *ppos)
{
	struct ssd1306fb_par *par = info->par;
	unsigned long total_size;
	unsigned long p = *ppos;
	u8 __iomem *dst;

	total_size = info->fix.smem_len;

	if (p > total_size)
		return -EINVAL;

	if (count + p > total_size)
		count = total_size - p;

	if (!count)
		return -EINVAL;

	dst = (void __force *) (info->screen_base + p);

	if (copy_from_user(dst, buf, count))
		return -EFAULT;

	ssd1306fb_update_display(par);

	*ppos += count;

	return count;
}

static void ssd1306fb_fillrect(struct fb_info *info, const struct fb_fillrect *rect)
{
	struct ssd1306fb_par *par = info->par;
	sys_fillrect(info, rect);
	ssd1306fb_update_display(par);
}

static void ssd1306fb_copyarea(struct fb_info *info, const struct fb_copyarea *area)
{
	struct ssd1306fb_par *par = info->par;
	sys_copyarea(info, area);
	ssd1306fb_update_display(par);
}

static void ssd1306fb_imageblit(struct fb_info *info, const struct fb_image *image)
{
	struct ssd1306fb_par *par = info->par;
	sys_imageblit(info, image);
	ssd1306fb_update_display(par);
}

static struct fb_ops ssd1306fb_ops = {
	.owner		= THIS_MODULE,
	.fb_read	= fb_sys_read,
	.fb_write	= ssd1306fb_write,
	.fb_fillrect	= ssd1306fb_fillrect,
	.fb_copyarea	= ssd1306fb_copyarea,
	.fb_imageblit	= ssd1306fb_imageblit,
};

static void ssd1306fb_deferred_io(struct fb_info *info,
				struct list_head *pagelist)
{
	ssd1306fb_update_display(info->par);
}

static struct fb_deferred_io ssd1306fb_defio = {
	.delay		= HZ,
	.deferred_io	= ssd1306fb_deferred_io,
};

static int ssd1306fb_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	lnw_gpio_set_alt(27, LNW_ALT_1 );
	lnw_gpio_set_alt(28, LNW_ALT_1 );

	struct fb_info *info;
	u32 vmem_size = SSD1306FB_WIDTH * SSD1306FB_HEIGHT / 8;
	struct ssd1306fb_par *par;
	u8 *vmem;
	int ret;
/*
	if (!client->dev.of_node) {
		dev_err(&client->dev, "No device tree data found!\n");
		return -EINVAL;
	}
	*/

	info = framebuffer_alloc(sizeof(struct ssd1306fb_par), &client->dev);
	if (!info) {
		dev_err(&client->dev, "Couldn't allocate framebuffer.\n");
		return -ENOMEM;
	}

	vmem = devm_kzalloc(&client->dev, vmem_size, GFP_KERNEL);
	if (!vmem) {
		dev_err(&client->dev, "Couldn't allocate graphical memory.\n");
		ret = -ENOMEM;
		goto fb_alloc_error;
	}

	info->fbops = &ssd1306fb_ops;
	info->fix = ssd1306fb_fix;
	info->fbdefio = &ssd1306fb_defio;

	info->var = ssd1306fb_var;
	info->var.red.length = 1;
	info->var.red.offset = 0;
	info->var.green.length = 1;
	info->var.green.offset = 0;
	info->var.blue.length = 1;
	info->var.blue.offset = 0;

	info->screen_base = (u8 __force __iomem *)vmem;
	info->fix.smem_start = (unsigned long)vmem;
	info->fix.smem_len = vmem_size;

	fb_deferred_io_init(info);

	par = info->par;
	par->info = info;
	par->client = client;
/*
	par->reset = of_get_named_gpio(client->dev.of_node,
					 "reset-gpios", 0);
	if (!gpio_is_valid(par->reset)) {
		ret = -EINVAL;
		goto reset_oled_error;
	}

	ret = devm_gpio_request_one(&client->dev, par->reset,
				    GPIOF_OUT_INIT_HIGH,
				    "oled-reset");
	if (ret) {
		dev_err(&client->dev,
			"failed to request gpio %d: %d\n",
			par->reset, ret);
		goto reset_oled_error;
	}

	par->pwm = pwm_get(&client->dev, NULL);
	if (IS_ERR(par->pwm)) {
		dev_err(&client->dev, "Could not get PWM from device tree!\n");
		ret = PTR_ERR(par->pwm);
		goto pwm_error;
	}

	par->pwm_period = pwm_get_period(par->pwm);

	dev_dbg(&client->dev, "Using PWM%d with a %dns period.\n", par->pwm->pwm, par->pwm_period);
*/
	ret = register_framebuffer(info);
	if (ret) {
		dev_err(&client->dev, "Couldn't register the framebuffer\n");
		goto fbreg_error;
	}

	i2c_set_clientdata(client, info);
/*
	// Reset the screen 
	gpio_set_value(par->reset, 0);
	udelay(4);
	gpio_set_value(par->reset, 1);
	udelay(4);

	// Enable the PWM 
	pwm_config(par->pwm, par->pwm_period / 2, par->pwm_period);
	pwm_enable(par->pwm);

	// Map column 127 of the OLED to segment 0
	ret = ssd1306fb_write_cmd(client, SSD1306FB_SEG_REMAP_ON);
	if (ret < 0) {
		dev_err(&client->dev, "Couldn't remap the screen.\n");
		goto remap_error;
	}
*/
	ret = ssd1306fb_write_cmd(client,0xAE);
	ret = ssd1306fb_write_cmd(client,0xD5);
	ret = ssd1306fb_write_cmd(client,0xf0);
	ret = ssd1306fb_write_cmd(client,0xa8);
	ret = ssd1306fb_write_cmd(client,0x0f);
	ret = ssd1306fb_write_cmd(client,0xd3);
	ret = ssd1306fb_write_cmd(client,0x00);
	ret = ssd1306fb_write_cmd(client,0x40);
	ret = ssd1306fb_write_cmd(client,0xa6);
	ret = ssd1306fb_write_cmd(client,0xa4);
	ret = ssd1306fb_write_cmd(client,0xa1);
	ret = ssd1306fb_write_cmd(client,0xC8);
	ret = ssd1306fb_write_cmd(client,0xda);
	ret = ssd1306fb_write_cmd(client,0x02);
	ret = ssd1306fb_write_cmd(client,0x81);
	ret = ssd1306fb_write_cmd(client,0x7F);
	ret = ssd1306fb_write_cmd(client,0xd9);
	ret = ssd1306fb_write_cmd(client,0x22);
	ret = ssd1306fb_write_cmd(client,0xdb);
	ret = ssd1306fb_write_cmd(client,0x49);
	ret = ssd1306fb_write_cmd(client,0x8d);
	ret = ssd1306fb_write_cmd(client,0x14);
	ret = ssd1306fb_write_cmd(client,0xAF);

/*
	// Turn on the display 
	ret = ssd1306fb_write_cmd(client, SSD1306FB_DISPLAY_ON);
	if (ret < 0) {
		dev_err(&client->dev, "Couldn't turn the display on.\n");
		goto remap_error;
	}
*/
	dev_info(&client->dev, "fb%d: %s framebuffer device registered, using %d bytes of video memory\n", info->node, info->fix.id, vmem_size);

	return 0;

remap_error:
	unregister_framebuffer(info);
//	pwm_disable(par->pwm);
fbreg_error:
//	pwm_put(par->pwm);
/*pwm_error:
reset_oled_error:
	fb_deferred_io_cleanup(info);*/
fb_alloc_error:
	framebuffer_release(info);
	return ret;
	
}

static int ssd1306fb_remove(struct i2c_client *client)
{
	struct fb_info *info = i2c_get_clientdata(client);
	struct ssd1306fb_par *par = info->par;

	unregister_framebuffer(info);
//	pwm_disable(par->pwm);
//	pwm_put(par->pwm);
	fb_deferred_io_cleanup(info);
	framebuffer_release(info);

	return 0;
}

static const struct i2c_device_id ssd1306fb_i2c_id[] = {
	{ "ssd1306fb", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ssd1306fb_i2c_id);


static struct i2c_driver ssd1306fb_driver = {
	.probe = ssd1306fb_probe,
	.remove = ssd1306fb_remove,
	.id_table = ssd1306fb_i2c_id,
	.driver = {
		.name = "ssd1306fb",
		.owner = THIS_MODULE,
	},
};

module_i2c_driver(ssd1306fb_driver);

MODULE_DESCRIPTION("FB driver for the Solomon SSD1306 OLED controller");
MODULE_AUTHOR("Maxime Ripard <maxime.ripard@free-electrons.com>");
MODULE_LICENSE("GPL");
