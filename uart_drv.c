#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/serial_reg.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>

/*******************************************************************************
* MACROS/DEFINES
*******************************************************************************/
#define UART_BAUD 115200
#define LO_BYTE(x) (x & 0xff)
#define HI_BYTE(x) ((x >> 8) & 0xff)

/*******************************************************************************
* DATA STRUCTURES
*******************************************************************************/
typedef struct _priv_serial_dev_t
{
	void __iomem *iomem_base; /* virtual addr of the memory mapped io region */
	struct miscdevice miscdev;
} priv_serial_dev_t;

/*******************************************************************************
* HELPER FUNCTIONS
*******************************************************************************/
/**
 * write to register specified by an addr offset from the UART base address
 *
 * See "19.5.1 UART Registers" in AM335x Technical Reference Manual for offsets
 * The UART Registers are 16bit wide
 */
static void reg_write(priv_serial_dev_t *dev, int val, int off)
{
	/**
	 * Common UART Register offsets are defined in linux/serial_reg.h
	 * For OMAP SoCs, the following conversion rule has to be applied:
	 *
	 * AM335x_offset = 4*linux_offset
	 */
	iowrite16(val, dev->iomem_base + 4 * off);
}

/* read from register specified by an addr offset from the UART base address */
static int reg_read(priv_serial_dev_t *dev, int off)
{
	/* AM335x_offset = 4*linux_offset */
	return ioread16(dev->iomem_base + 4 * off);
}

static void uart_char_write(priv_serial_dev_t *dev, char c)
{
	while (!(reg_read(dev, UART_LSR) & UART_LSR_THRE)) {
		/* Optimization barrier, reload variable on each iteration */
		cpu_relax();
	}

	/* write char to register */
	reg_write(dev, c, UART_TX);
}

static void uart_str_write(priv_serial_dev_t *dev, char *s)
{
	char c;

	while ((c = *s++) != '\0')
		uart_char_write(dev, c);
}

static void init_uart(struct platform_device *pdev)
{
	unsigned uartclk, baud_divisor;
	priv_serial_dev_t *priv;

	priv = platform_get_drvdata(pdev);

	/* Setup UART module */
	of_property_read_u32(pdev->dev.of_node, "clock-frequency", &uartclk); /* input clock to the UART */
	baud_divisor = uartclk / 16 / UART_BAUD; /* See "4.4.4.2.1 Clock Generation and Control" in AM335x Technical Reference Manual */

	reg_write(priv, 0x07, UART_OMAP_MDR1);            /* set MODESELECT to Disable */
	reg_write(priv, UART_LCR_DLAB, UART_LCR);         /* set Divisor latch enable (Allows access to DLL and DLH) */
	reg_write(priv, LO_BYTE(baud_divisor), UART_DLL); /* set CLOCK_LSB */
	reg_write(priv, HI_BYTE(baud_divisor), UART_DLM); /* set CLOCK_MSB */
	reg_write(priv, UART_LCR_WLEN8, UART_LCR);        /* set word length to 8bit */

	/* Soft reset */
	reg_write(priv, UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT, UART_FCR); /* Clears the RX/TX FIFO and resets its counter logic to 0 */
	reg_write(priv, 0x00, UART_OMAP_MDR1);                                /* MPDESELECT = UART 16x mode */


	// DBG
	uart_str_write(priv, "Hello World, config done!");
}

static void deinit_uart(struct platform_device *pdev)
{
	priv_serial_dev_t *priv;

	priv = platform_get_drvdata(pdev);

	/* set MODESELECT to Disable */
	reg_write(priv, 0x07, UART_OMAP_MDR1);
}

/*******************************************************************************
* MISC DEVICE DRV
*******************************************************************************/
ssize_t write(struct file *f, const char *s, size_t len, loff_t *l)
{
	return -EINVAL;
}
ssize_t read(struct file *f, char *s, size_t len, loff_t *l)
{
	return -EINVAL;
}

static struct file_operations fops = {
	.write = write,
	.read = read
};

/*******************************************************************************
* PLATFORM PM
*******************************************************************************/
int platform_pm_suspend(struct device *dev)
{
	struct platform_device *pdev;

	pr_alert("%s called!\n", __func__);

	pdev = to_platform_device(dev);
	deinit_uart(pdev);

	return 0;
}

int platform_pm_resume(struct device *dev)
{
	struct platform_device *pdev;

	pr_alert("%s called!\n", __func__);

	pdev = to_platform_device(dev);
	init_uart(pdev);

	return 0;
}

int platform_pm_idle(struct device *dev)
{
	pr_alert("%s called!\n", __func__);
	// TODO: idle the UART unit
	return 0;
}

/*******************************************************************************
* PLATFORM DRV
*******************************************************************************/
static int serial_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *res;
	priv_serial_dev_t *priv;

	pr_alert("%s: Module Base: 0x%x\n", __func__, (int)THIS_MODULE->core_layout.base);

	/**
	 * start addr (base addr) of the memory mapped UART Registers
	 * Total size of the mem region: 4KB
	 * pdev has 2 resources: IORESOURCE_MEM and IORESOURCE_IRQ
	 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		goto no_base_addr;

	/* private data, automatically freed on driver detach */
	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		goto alloc_err;

	/**
	 * map physical addr of UART registers into virtual addr space of kernel
	 * Returns a pointer to the remapped virtual memory
	 * or an ERR_PTR() encoded error code on failure
	 *
	 * Map is automatically unmapped on driver detach
	 */
	priv->iomem_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->iomem_base))
		goto remap_err;

	/* save priv data ptr */
	platform_set_drvdata(pdev, priv);

	/**
	 * Power management:
	 *
	 * This indirectly calls omap_device_enable()
	 *
	 * From arch/arm/mach-omap2/omap_device.c:
	 *  # Do whatever is necessary for the hwmods underlying omap_device @od
	 *  # to be accessible and ready to operate.  This generally involves
	 *  # enabling clocks, setting SYSCONFIG registers; and in the future may
	 *  # involve remuxing pins.  Device drivers should call this function
	 *  # indirectly via pm_runtime_get*().
	 */
	pr_info("Pstate: %d\n", pdev->dev.power.runtime_status);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev); /* get reference, resume and sync */
	pr_info("Pstate: %d\n", pdev->dev.power.runtime_status);

	/* misc drv */
	priv->miscdev.fops 	= &fops;
	priv->miscdev.minor	= MISC_DYNAMIC_MINOR; /*dynamically set minor number */
	priv->miscdev.name 	= devm_kasprintf(&pdev->dev, GFP_KERNEL, "serial-%x", res->start); /* allocate buffer and print to it */

	ret = misc_register(&priv->miscdev);
	if (ret)
		goto misc_reg_err;

	dev_info(&pdev->dev, "Created misc device \"%s\"\n", priv->miscdev.name);

	return 0;

/* error handling */
no_base_addr:
	dev_err(&pdev->dev, "Error: Cannot find Base Address\n");
	return -ENODEV;

alloc_err:
	dev_err(&pdev->dev, "Error: Cannot allocate memory\n");
	return -ENOMEM;

remap_err:
	dev_err(&pdev->dev, "Error: Cannot remap registers\n");
	return -ENOMEM;

misc_reg_err:
	dev_err(&pdev->dev, "Error: Cannot register misc device\n");
	return -ENOMEM;
}

static int serial_remove(struct platform_device *pdev)
{
	priv_serial_dev_t *priv;

	pr_info("Called serial_remove\n");

	/* power management */
	pr_info("Pstate: %d\n", pdev->dev.power.runtime_status);
	pm_runtime_put_sync(&pdev->dev); /* put reference, idle request and sync (Devices with references held cannot be suspended) */
	pm_runtime_disable(&pdev->dev);
	pr_info("Pstate: %d\n", pdev->dev.power.runtime_status);

	/* misc drv */
	priv = platform_get_drvdata(pdev);
	misc_deregister(&priv->miscdev);

	return 0;
}

/****************************** driver structures *****************************/
static const struct of_device_id uart_match[] = {
	{.compatible = "mh,uart_dev"},
	{/* sentinel */},
};

static const struct dev_pm_ops uart_pm = {
	SET_RUNTIME_PM_OPS(platform_pm_suspend, platform_pm_resume, platform_pm_idle)
};

static struct platform_driver serial_driver = {
	.driver = {
		.name = "UART driver",
		.owner = THIS_MODULE,
		.of_match_table = uart_match,
		.pm = &uart_pm,
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
