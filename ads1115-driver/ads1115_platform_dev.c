#include <linux/module.h>
#include <linux/kernel.h>
//For platform drivers....
#include <linux/platform_device.h>
#include <linux/i2c/ads1015.h>


#define DEVICE_NAME "ads1015"

MODULE_LICENSE("GPL");

static struct ads1015_platform_data data = {
	.channel_data = {
		[2] = { .enabled = true, .pga = 1, .data_rate = 0 },
		[4] = { .enabled = true, .pga = 4, .data_rate = 5 },
	}
};

static struct platform_device ads1115_device = {
        .name           = DEVICE_NAME,
        .id             = PLATFORM_DEVID_NONE,
        .dev.platform_data = &data
};

int ads1115_device_init(void)
{
 printk(KERN_ALERT "\n Register te hello ads1115_device (device).... \n");
 platform_device_register(&ads1115_device);
 return 0;
}

void ads1115_device_exit(void)
{
 platform_device_unregister(&ads1115_device);
 printk(KERN_ALERT "\n Ungister the hello ads1115_device (device) ... \n");
}

module_init(ads1115_device_init);
module_exit(ads1115_device_exit);