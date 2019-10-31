
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/irqflags.h>
#include <linux/ioctl.h> 
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/sched.h>

#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <media/rc-core.h>

#define GPIO_IRDA_DRIVER_NAME	"gpio-irda-rc"
#define GPIO_IRDA_DEVICE_NAME	"gpio_irda-ir"

#define IRDA_IOC_MAGIC    'i'
#define IRDA_IOCINIT      _IOW(IRDA_IOC_MAGIC, 0, int)

static int irda_major;
static struct cdev irda_cdev;
static struct class *irda_class;


struct gpio_rc_dev {
	struct rc_dev *rcdev;
	int gpio_nr;
	bool active_low;
	struct timer_list flush_timer;
};

static struct rc_dev *rcdev;
static struct gpio_rc_dev *gpio_dev;

static irqreturn_t gpio_ir_recv_irq(int irq, void *dev_id)
{
	struct gpio_rc_dev *gpio_dev = dev_id;
	int gval;
	int rc = 0;
	enum raw_event_type type = IR_SPACE;

	gval = gpio_get_value(gpio_dev->gpio_nr);

	if (gval < 0)
		goto err_get_value;

	if (gpio_dev->active_low)
		gval = !gval;

	if (gval == 1)
		type = IR_PULSE;

	rc = ir_raw_event_store_edge(gpio_dev->rcdev, type);
	if (rc < 0)
		goto err_get_value;

	mod_timer(&gpio_dev->flush_timer,
		  jiffies + nsecs_to_jiffies(gpio_dev->rcdev->timeout));

	ir_raw_event_handle(gpio_dev->rcdev);

err_get_value:
	return IRQ_HANDLED;
}

static void flush_timer(unsigned long arg)
{
	struct gpio_rc_dev *gpio_dev = (struct gpio_rc_dev *)arg;
	DEFINE_IR_RAW_EVENT(ev);

	ev.timeout = true;
	ev.duration = gpio_dev->rcdev->timeout;
	ir_raw_event_store(gpio_dev->rcdev, &ev);
	ir_raw_event_handle(gpio_dev->rcdev);
}


static int irda_init(int data_pin)
{
	int rc;

	gpio_dev = kzalloc(sizeof(struct gpio_rc_dev), GFP_KERNEL);
	if (!gpio_dev)
		return -ENOMEM;

	rcdev = rc_allocate_device();
	if (!rcdev) {
		rc = -ENOMEM;
		goto err_allocate_device;
	}
	rcdev->priv = gpio_dev;
	rcdev->driver_type = RC_DRIVER_IR_RAW;
	rcdev->input_name = GPIO_IRDA_DEVICE_NAME;
	rcdev->input_phys = GPIO_IRDA_DEVICE_NAME "/input0";
	rcdev->input_id.bustype = BUS_HOST;
	rcdev->input_id.vendor = 0x0001;
	rcdev->input_id.product = 0x0001;
	rcdev->input_id.version = 0x0100;
	rcdev->dev.parent = NULL;
	rcdev->driver_name = GPIO_IRDA_DRIVER_NAME;
	rcdev->min_timeout = 0;
	rcdev->timeout = MS_TO_NS(125);
	rcdev->max_timeout = 10 * MS_TO_NS(125);
	rcdev->allowed_protocols = RC_BIT_NEC;//RC_BIT_ALL;
	rcdev->map_name = "rc-100ask-nec";
	
	gpio_dev->rcdev = rcdev;
	gpio_dev->gpio_nr = data_pin;
	gpio_dev->active_low = 1;

	setup_timer(&gpio_dev->flush_timer, flush_timer, (unsigned long)gpio_dev);

	rc = gpio_request(gpio_dev->gpio_nr, "gpio-irda-recv");
	if (rc < 0)
		goto err_gpio_request;
	rc  = gpio_direction_input(gpio_dev->gpio_nr);
	if (rc < 0)
		goto err_gpio_direction_input;

	rc = rc_register_device(rcdev);
	if (rc < 0) {
		printk("failed to register rc device\n");
		goto err_register_rc_device;
	}
	
	rc = request_any_context_irq(gpio_to_irq(gpio_dev->gpio_nr),
				gpio_ir_recv_irq,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
					"gpio-irda-irq", gpio_dev);
	if (rc < 0)
		goto err_request_irq;
	
	return 0;
	
err_request_irq:
	rc_unregister_device(rcdev);
	rcdev = NULL;
err_register_rc_device:
err_gpio_direction_input:
	gpio_free(gpio_dev->gpio_nr);
err_gpio_request:
	rc_free_device(rcdev);
err_allocate_device:
	kfree(gpio_dev);
	return rc;

}

static int irda_open(struct inode *inode, struct file *file) 
{
	//printk(KERN_INFO"%s OK.\n",__func__);	
	
	return 0;
}

static ssize_t irda_read(struct file* file, char __user *buf,
        size_t count, loff_t* fpos)
{
	//printk(KERN_INFO"%s OK.\n",__func__);

    return 0;
}
		

static long  irda_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int data_pin = 0;
	
	switch(cmd) {
	case IRDA_IOCINIT:
		if (copy_from_user(&data_pin, (int *)arg, sizeof(int)))
		{
			printk(KERN_ERR "IRDA_IOCINIT: copy_from_user() fail.\n");
			return -EINVAL;
		}
		irda_init(data_pin);

		break;	
	default:
		ret = -ENOTTY;
		break;
	}
	
	return ret;
}

static int irda_release(struct inode *inode, struct file *file) 
{
	free_irq(gpio_to_irq(gpio_dev->gpio_nr), gpio_dev);
	del_timer_sync(&gpio_dev->flush_timer);
	rc_unregister_device(gpio_dev->rcdev);
	gpio_free(gpio_dev->gpio_nr);
	kfree(gpio_dev);

	return 0;
}


static struct file_operations irda_fops = {
	.owner = THIS_MODULE,
    .open = irda_open,
    .read = irda_read,
	.unlocked_ioctl = irda_ioctl,
    .release = irda_release,
};

static int irda_drv_init(void)  
{  
  	int ret;
	dev_t irda_devid;
	
	printk(KERN_INFO"%s OK.\n",__func__);
	if(alloc_chrdev_region(&irda_devid, 0, 1, "irda") < 0)
    {
        printk(KERN_ERR"Unable to alloc_chrdev_region.\n");
        return -EINVAL;
    } 
    irda_major = MAJOR(irda_devid);
	cdev_init(&irda_cdev, &irda_fops);        
    ret = cdev_add(&irda_cdev, irda_devid, 1);
    if (ret < 0)
    {
        printk(KERN_ERR "Unable to cdev_add.\n");
        goto error;
    }
        
    irda_class = class_create(THIS_MODULE, "irda"); 
    device_create(irda_class, NULL, MKDEV(irda_major, 0), NULL, "irda"); 

	return 0;
error:
    unregister_chrdev_region(MKDEV(irda_major, 0), 1);
    return -EINVAL;
	
}  
  
static void irda_drv_exit(void)  
{  
    printk(KERN_INFO"%s OK.\n",__func__);
	
	device_destroy(irda_class,  MKDEV(irda_major, 0));
    class_destroy(irda_class);
 
    unregister_chrdev_region(MKDEV(irda_major, 0), 1);
    cdev_del(&irda_cdev);
}  

module_init(irda_drv_init);
module_exit(irda_drv_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("hceng <huangcheng.job@foxmail.com>");
MODULE_DESCRIPTION("irda driver.");
MODULE_VERSION("v2.0");


