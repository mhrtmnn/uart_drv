#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>


static int __init hello_start(void)
{
                printk(KERN_INFO "Hello world\n");
                return 0;
}
module_init(hello_start);

static void __exit hello_end(void)
{
                printk(KERN_INFO "Goodbye Mr.\n");
}
module_exit(hello_end);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marco Hartmann");
MODULE_DESCRIPTION("UART driver");
MODULE_VERSION(MOD_VER);
