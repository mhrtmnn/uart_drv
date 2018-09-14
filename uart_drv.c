#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>


/*******************************************************************************
* PLATFORM DRV
*******************************************************************************/
static int serial_probe(struct platform_device *pdev)
{
	pr_info("Called serial_probe\n");
	return 0;
}

static int serial_remove(struct platform_device *pdev)
{
	pr_info("Called serial_remove\n");
        return 0;
}

/****************************** driver structures *****************************/
static struct platform_driver serial_driver = {
        .driver = {
                .name = "UART driver",
                .owner = THIS_MODULE,
        },
        .probe = serial_probe,
        .remove = serial_remove,
};

module_platform_driver(serial_driver);

/*******************************************************************************
* MODULE INFORMATION
*******************************************************************************/
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marco Hartmann");
MODULE_DESCRIPTION("UART driver");
MODULE_VERSION(MOD_VER);
