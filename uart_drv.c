#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/serial_reg.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>

/*******************************************************************************
* MACROS/DEFINES
*******************************************************************************/
#define UART_BAUD 115200
#define LO_BYTE(x) (x & 0xff)
#define HI_BYTE(x) ((x >> 8) & 0xff)
#define IOCLT_SERIAL_RESET_COUNTER 0
#define IOCLT_SERIAL_GET_COUNTER 1
#define SERIAL_BUFSIZE 16

/*******************************************************************************
* DATA STRUCTURES
*******************************************************************************/
typedef struct _priv_serial_dev_t
{
	void __iomem *iomem_base; /* virtual addr of the memory mapped io region */
	struct miscdevice miscdev;
	long num_sent_char;
	int irq;
	char circ_buf[SERIAL_BUFSIZE];
	int buf_rd; /* current rd ptr in circ buffer */
	int buf_wr; /* current wr ptr in circ buffer */
} priv_serial_dev_t;

/*******************************************************************************
* HELPER FUNCTIONS
*******************************************************************************/

/********************************* Register IO ********************************/

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

/********************************** char R/W **********************************/

static char uart_char_read(priv_serial_dev_t *dev)
{
	char c;

	// TODO Locking
	if (dev->buf_wr != dev->buf_rd) {
		/* there is at least one char in the buffer */
		c = dev->circ_buf[dev->buf_rd];
		dev->buf_rd = (dev->buf_rd + 1) % SERIAL_BUFSIZE;
	} else {
		c = '\0';
	}

	return c;
}

static void uart_char_write(priv_serial_dev_t *dev, char c)
{
	while (!(reg_read(dev, UART_LSR) & UART_LSR_THRE)) {
		/* Optimization barrier, reload variable on each iteration */
		cpu_relax();
	}

	/* write char to register */
	reg_write(dev, c, UART_TX);

	/* update stats */
	dev->num_sent_char++;
}

static void uart_str_write(priv_serial_dev_t *dev, char *s)
{
	char c;

	while ((c = *s++) != '\0')
		uart_char_write(dev, c);
}

/********************************** lifetime **********************************/

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
	reg_write(priv, UART_IER_RDI, UART_IER);          /* enable receiver data interrupt */

	/* Soft reset */
	reg_write(priv, UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT, UART_FCR); /* Clears the RX/TX FIFO and resets its counter logic to 0 */
	reg_write(priv, 0x00, UART_OMAP_MDR1);                                /* MPDESELECT = UART 16x mode */


	// DBG
	uart_str_write(priv, "Hello World, config done!\n\r");
}

static void deinit_uart(struct platform_device *pdev)
{
	priv_serial_dev_t *priv;

	priv = platform_get_drvdata(pdev);

	/* set MODESELECT to Disable */
	reg_write(priv, 0x07, UART_OMAP_MDR1);
}

/*******************************************************************************
* INTERRUPT HANDLER
*******************************************************************************/
irqreturn_t uart_int_handler(int irq, void *dev_id)
{
	int ret;
	struct platform_device *pdev = dev_id;
	priv_serial_dev_t *priv = platform_get_drvdata(pdev);

	/* check wether there is receiver data ready */
	if (reg_read(priv, UART_LSR) & UART_LSR_DR) {
		char c = reg_read(priv, UART_RX);
		//dev_info(&pdev->dev, "Interrupt! Received '%c' (wpos=%d, rpos=%d)\n", c, priv->buf_wr, priv->buf_rd);

		/* store char at wr ptr position in circ buffer */
		priv->circ_buf[priv->buf_wr] = c;
		priv->buf_wr = (priv->buf_wr + 1) % SERIAL_BUFSIZE;

		ret = IRQ_HANDLED;
	} else {
		/* there is no new character in the RX FIFO */
		dev_info(&pdev->dev, "Huh?? I thought there was an interrupt!\n");

		ret = IRQ_NONE;
	}

	return ret;
}

/*******************************************************************************
* MISC DEVICE DRV
*******************************************************************************/
ssize_t f_uart_write(struct file *f, const char *buffer, size_t count, loff_t *ppos)
{
	int i;
	char c;
	struct miscdevice *mdev;
	priv_serial_dev_t *priv;

	mdev = f->private_data;
	priv = container_of(mdev, priv_serial_dev_t, miscdev);

	/* print buffer charwise */
	for (i = 0; i < count; i++) {

		if (copy_from_user(&c, buffer+i, 1))
			return -EFAULT;

		uart_char_write(priv, c);

		/* handle CR */
		if (c == '\n')
			uart_char_write(priv, '\r');
	}

	return count;
}

ssize_t f_uart_read(struct file *f, char *buffer, size_t count, loff_t *off)
{
	int i;
	char c;
	struct miscdevice *mdev;
	priv_serial_dev_t *priv;

	mdev = f->private_data;
	priv = container_of(mdev, priv_serial_dev_t, miscdev);

	/* read charwise and copy to userspace buffer */
	i = 0;
	while (i + (*off) < count) {
		c = uart_char_read(priv);
		if (c == '\0')
			/* no new data */
			break;

		/* echo back */
		uart_char_write(priv, c);

		/**
		 * fill userspace buffer
		 * If read is called multiple times, the buffer contains the data that
		 * was written to it in previous calls.
		 * 'off' is offset in buffer to the first empty element from previous
		 * call to read, i.e. off-1 is the last element written by the prev call
		 */
		if (copy_to_user(buffer + (*off) + i++, &c, 1))
			return -EFAULT;
	}

	/* advance offset by number of read bytes */
	(*off) += i;

	return i;
}

long f_uart_u_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int __user *argp;
	struct miscdevice *mdev;
	priv_serial_dev_t *priv;

	mdev = f->private_data;
	priv = container_of(mdev, priv_serial_dev_t, miscdev);
	argp = (void __user *)arg;

	switch (cmd) {
		case IOCLT_SERIAL_RESET_COUNTER:
			priv->num_sent_char = 0;
			break;
		case IOCLT_SERIAL_GET_COUNTER:
			/**
			 * copies a single simple value from kernel space to user space.
			 * It supports simple types like char, int and long.
			 * copy_to_user() is more general and copies an arbitrary
			 * amount of data to userspace.
			 */
			put_user(priv->num_sent_char, argp);
			break;
		default:
			ret = -EINVAL;
			break;
	}

	return ret;
}

static struct file_operations fops = {
	.write 			= f_uart_write,
	.read  			= f_uart_read,
	.unlocked_ioctl = f_uart_u_ioctl,
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

	pr_alert("%s: Base = 0x%x\n", __func__, (int)THIS_MODULE->core_layout.base);

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

	/* misc drv */
	priv->miscdev.fops 	= &fops;
	priv->miscdev.minor	= MISC_DYNAMIC_MINOR; /*dynamically set minor number */
	priv->miscdev.name 	= devm_kasprintf(&pdev->dev, GFP_KERNEL, "serial-%x", res->start); /* allocate buffer and print to it */

	ret = misc_register(&priv->miscdev);
	if (ret)
		goto misc_reg_err;

	dev_info(&pdev->dev, "Created misc device \"%s\"\n", priv->miscdev.name);

	/* get IRQ form device tree */
	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0)
		goto irq_dts_err;

	/* request a threaded irq, that is freed on driver detach */
	ret = devm_request_irq(&pdev->dev, priv->irq, uart_int_handler, 0,
						   "uart_int_handler", pdev);
	if (ret)
		goto request_irq_err;

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

irq_dts_err:
	dev_err(&pdev->dev, "Error: Cannot not get IRQ from DTS\n");
	misc_deregister(&priv->miscdev);
	return ret;

request_irq_err:
	dev_err(&pdev->dev, "Error: Cannot request IRQ\n");
	misc_deregister(&priv->miscdev);
	return ret;
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
