

#include <linux/input.h>	/* BUS_SPI */
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/cdev.h>

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/pm.h>
#include <linux/types.h>
#include "adxl34x.h"

#define MAX_SPI_FREQ_HZ		5000000
#define MAX_FREQ_NO_FIFODELAY	1500000
#define ADXL34X_CMD_MULTB	(1 << 6)
#define ADXL34X_CMD_READ	(1 << 7)
#define ADXL34X_WRITECMD(reg)	(reg & 0x3F)
#define ADXL34X_READCMD(reg)	(ADXL34X_CMD_READ | (reg & 0x3F))
#define ADXL34X_READMB_CMD(reg) (ADXL34X_CMD_READ | ADXL34X_CMD_MULTB \
					| (reg & 0x3F))

#define ADXL34X_IOC_MAGIC    'a'
#define ADXL34X_IOCINIT      _IOWR(ADXL34X_IOC_MAGIC, 0, void *)
#define ADXL34X_IOCEXIT      _IOWR(ADXL34X_IOC_MAGIC, 1, void *)

struct adxl34x_info_msg {
    int max_speed_hz;
    u16 mode;
    u16 chip_select;
    int irq_pin;
};

static int adxl34x_major;
static struct cdev adxl34x_cdev;
static struct class *adxl34x_class;

static int adxl34x_spi_read(struct device *dev, unsigned char reg)
{
	struct spi_device *spi = to_spi_device(dev);
	unsigned char cmd;

	cmd = ADXL34X_READCMD(reg);

	return spi_w8r8(spi, cmd);
}

static int adxl34x_spi_write(struct device *dev,
			     unsigned char reg, unsigned char val)
{
	struct spi_device *spi = to_spi_device(dev);
	unsigned char buf[2];

	buf[0] = ADXL34X_WRITECMD(reg);
	buf[1] = val;

	return spi_write(spi, buf, sizeof(buf));
}

static int adxl34x_spi_read_block(struct device *dev,
				  unsigned char reg, int count,
				  void *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	ssize_t status;

	reg = ADXL34X_READMB_CMD(reg);
	status = spi_write_then_read(spi, &reg, 1, buf, count);

	return (status < 0) ? status : 0;
}

static const struct adxl34x_bus_ops adxl34x_spi_bops = {
	.bustype	= BUS_SPI,
	.write		= adxl34x_spi_write,
	.read		= adxl34x_spi_read,
	.read_block	= adxl34x_spi_read_block,
};

static int adxl34x_spi_probe(struct spi_device *spi)
{
	struct adxl34x *ac;

	/* don't exceed max specified SPI CLK frequency */
	if (spi->max_speed_hz > MAX_SPI_FREQ_HZ) {
		dev_err(&spi->dev, "SPI CLK %d Hz too fast\n", spi->max_speed_hz);
		return -EINVAL;
	}

	ac = adxl34x_probe(&spi->dev, spi->irq,
			   spi->max_speed_hz > MAX_FREQ_NO_FIFODELAY,
			   &adxl34x_spi_bops);

	if (IS_ERR(ac))
		return PTR_ERR(ac);

	spi_set_drvdata(spi, ac);

	return 0;
}


static int adxl34x_spi_remove(struct spi_device *spi)
{
	struct adxl34x *ac = spi_get_drvdata(spi);

	return adxl34x_remove(ac);
}

static int __maybe_unused adxl34x_spi_suspend(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct adxl34x *ac = spi_get_drvdata(spi);

	adxl34x_suspend(ac);

	return 0;
}

static int __maybe_unused adxl34x_spi_resume(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct adxl34x *ac = spi_get_drvdata(spi);

	adxl34x_resume(ac);

	return 0;
}

static SIMPLE_DEV_PM_OPS(adxl34x_spi_pm, adxl34x_spi_suspend,
			 adxl34x_spi_resume);

static struct spi_driver adxl34x_driver = {
	.driver = {
		.name = "adxl34x",
		.owner = THIS_MODULE,
		.pm = &adxl34x_spi_pm,
	},
	.probe   = adxl34x_spi_probe,
	.remove  = adxl34x_spi_remove,
};


/****************************add***************************/

static struct spi_board_info spi_slave_info[] = {
	{
		.modalias	   = "adxl34x",		
		.bus_num	   = 2, 			 
		.max_speed_hz  = 2000000,
		.mode		   = SPI_MODE_3,
		.chip_select   = 0, 
		.irq           = 0, 
	},
	
	{
		.modalias	   = "adxl34x",		
		.bus_num	   = 2, 			 
		.max_speed_hz  = 2000000,
		.mode		   = SPI_MODE_3,
		.chip_select   = 1, 
		.irq           = 0,
	},
		
};

static struct spi_device *add_device[2];

static int adxl34x_register(int chip_select)
{
	//printk(KERN_INFO"%s OK.\n",__func__);	

	//dev
	struct spi_master *master = NULL;

	master = spi_busnum_to_master(spi_slave_info->bus_num);
	if (!master) 
		return -EINVAL;
	add_device[chip_select] = spi_new_device(master, &spi_slave_info[chip_select]);
	if (!add_device[chip_select])
		return -EINVAL;
	
	//drv
	spi_register_driver(&adxl34x_driver);
	
	return 0;
}

static int adxl34x_unregister(int chip_select)
{
	//printk(KERN_INFO"%s OK.\n",__func__);	
	//dev
	spi_unregister_device(add_device[chip_select]);

	//drv
	spi_unregister_driver(&adxl34x_driver);
	
	return 0;
}


static int adxl34x_open(struct inode *inode, struct file *file) 
{
	//printk(KERN_INFO"%s OK.\n",__func__);	
	return 0;
}
		

static long  adxl34x_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct adxl34x_info_msg ad_info_msg;
	
	switch(cmd) {
	case ADXL34X_IOCINIT:
		if (copy_from_user(&ad_info_msg, (struct adxl34x_info_msg __user *)arg, sizeof(ad_info_msg)))
		{
			printk(KERN_ERR "ADXL34X_IOCINIT: copy_from_user() fail.\n");
			return -EINVAL;
		}

		//spi_slave_info->max_speed_hz = ad_info_msg.max_speed_hz;
		//spi_slave_info->mode = ad_info_msg.mode;
		spi_slave_info[ad_info_msg.chip_select].chip_select = ad_info_msg.chip_select;

		ret = gpio_request(ad_info_msg.irq_pin, "irq_pin");   
		if (ret)
			printk("gpio_request() fail.\n");
		gpio_direction_input(ad_info_msg.irq_pin);

		spi_slave_info[ad_info_msg.chip_select].irq = gpio_to_irq(ad_info_msg.irq_pin);
		
		adxl34x_register(spi_slave_info[ad_info_msg.chip_select].chip_select);
		break;	
		
	case ADXL34X_IOCEXIT:
		if (copy_from_user(&ad_info_msg, (struct adxl34x_info_msg __user *)arg, sizeof(ad_info_msg)))
		{
			printk(KERN_ERR "ADXL34X_IOCEXIT: copy_from_user() fail.\n");
			return -EINVAL;
		}
		//free_irq(gpio_to_irq(ad_info_msg.irq_pin), NULL);
		gpio_free(ad_info_msg.irq_pin);

		spi_slave_info[ad_info_msg.chip_select].chip_select = ad_info_msg.chip_select;
		adxl34x_unregister(spi_slave_info[ad_info_msg.chip_select].chip_select);
		
		break;

	default:
		ret = -ENOTTY;
		break;
	}
	
	return ret;
}

static int adxl34x_release(struct inode *inode, struct file *file) 
{
	return 0;
}


static struct file_operations adxl34x_fops = {
	.owner = THIS_MODULE,
    .open = adxl34x_open,
	.unlocked_ioctl = adxl34x_ioctl,
    .release = adxl34x_release,
};

static int adxl34x_drv_init(void)  
{  
  	int ret;
	dev_t adxl34x_devid;
	
	//printk(KERN_INFO"%s OK.\n",__func__);
	if(alloc_chrdev_region(&adxl34x_devid, 0, 1, "adxl345") < 0)
    {
        printk(KERN_ERR"Unable to alloc_chrdev_region.\n");
        return -EINVAL;
    } 
    adxl34x_major = MAJOR(adxl34x_devid);
	cdev_init(&adxl34x_cdev, &adxl34x_fops);        
    ret = cdev_add(&adxl34x_cdev, adxl34x_devid, 1);
    if (ret < 0)
    {
        printk(KERN_ERR "Unable to cdev_add.\n");
        goto error;
    }
        
    adxl34x_class = class_create(THIS_MODULE, "adxl345"); 
    device_create(adxl34x_class, NULL, MKDEV(adxl34x_major, 0), NULL, "adxl345"); 

	return 0;
error:
    unregister_chrdev_region(MKDEV(adxl34x_major, 0), 1);
    return -EINVAL;
	
}  
  
static void adxl34x_drv_exit(void)  
{  
    //printk(KERN_INFO"%s OK.\n",__func__);
	
	device_destroy(adxl34x_class,  MKDEV(adxl34x_major, 0));
    class_destroy(adxl34x_class);
 
    unregister_chrdev_region(MKDEV(adxl34x_major, 0), 1);
    cdev_del(&adxl34x_cdev);
}  

module_init(adxl34x_drv_init);
module_exit(adxl34x_drv_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("hceng <huangcheng.job@foxmail.com>");
MODULE_DESCRIPTION("adxl34x driver.");
MODULE_VERSION("v2.0");

