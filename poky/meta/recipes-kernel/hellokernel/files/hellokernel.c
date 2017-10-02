/*  
 *  hello-1.c - The simplest kernel module.
 */
#include <linux/module.h>
#include <linux/kernel.h>


#include <linux/moduleparam.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/lnw_gpio.h>
#include <linux/spi/intel_mid_ssp_spi.h>
#include <asm/intel-mid.h>

#include <linux/init.h>
#include <linux/interrupt.h>            // Required for the IRQ code

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yuncheng Song");
MODULE_DESCRIPTION("A hello world kernel model for edison");
MODULE_VERSION("0.1");
static int spi5cs0 = 110;

module_param(spi5cs0, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);

MODULE_PARM_DESC(spi5cs0, "An integer");

static int __init hellokernel_init(void)
{
	lnw_gpio_set_alt(27,LNW_ALT_1);
	lnw_gpio_set_alt(28,LNW_ALT_1);
	
	pr_err("%s: GPIO mux:%d \t %d\n", __func__,gpio_get_alt(27),gpio_get_alt(28));
	/*
	gpio_free(spi5cs0);
	printk(KERN_INFO "Hello world 2.\n");
	pr_err("%s IRQ for GPIO %d: %d\n", __func__, spi5cs0 , gpio_to_irq(spi5cs0 )  );
	pr_err("%s: LNW_GPIO :%d \n", __func__,LNW_GPIO);
	int saved_muxing = gpio_get_alt(spi5cs0);
	lnw_gpio_set_alt(spi5cs0, LNW_GPIO);
//
	int err = gpio_request_one(spi5cs0, GPIOF_DIR_OUT|GPIOF_INIT_HIGH, "MCP2515 CS");
	pr_err("GPIO %d saved muxing %d \n gpio %d request one return %d\n", spi5cs0, saved_muxing, spi5cs0, err);
	*/
	//gpio_request_one(48, GPIOF_DIR_OUT|GPIOF_INIT_HIGH, "MCP2515 CS"));
	/* 
	 * A non 0 return means init_module failed; module can't be loaded. 
	 */
	return 0;
}

static void __exit hellokernel_exit(void)
{
	printk(KERN_INFO "Goodbye world 2.\n");
}

 
/// This next calls are  mandatory -- they identify the initialization function
/// and the cleanup function (as above).
module_init(hellokernel_init);
module_exit(hellokernel_exit);