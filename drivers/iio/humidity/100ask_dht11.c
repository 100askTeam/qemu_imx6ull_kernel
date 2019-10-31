
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

#define DHT11_IOC_MAGIC    'd'
#define DHT11_IOCINIT      _IOW(DHT11_IOC_MAGIC, 0, int)
#define DHT11_IOCGVALUE    _IOWR(DHT11_IOC_MAGIC, 2, void *)

struct dht11_value_msg {
    u8 humdity;
    u8 temperature;
};

static int dht11_major;
static struct cdev dht11_cdev;
static struct class *dht11_class;

struct dht11 {
    struct mutex lock;  
    int da_pin;
    u8 value[5];
    u8 humdity;
    u8 temperature;
    struct timer_list timer;
    struct work_struct work;

};
static struct dht11 dh;

static unsigned char read_byte(void)  
{
    unsigned char i, count, dat = 0;
    unsigned long flags;
    local_irq_save(flags); //Save interrupt
    
    for(i=0; i<8; i++)          
    {      
        count = 0;
        while(0 == gpio_get_value(dh.da_pin)) //Waiting for 50us low level end
        {
            udelay(5);
            count ++;
            if (count > 12)
                goto time_out;
        }
      
        udelay(30); //Delay 30us, if it is still high, the data is 1, otherwise it is 0.
      
        dat <<= 1;                   
      
        if(1 == gpio_get_value(dh.da_pin))  
		{
            dat += 1;
         
			count = 0;
			while(1 == gpio_get_value(dh.da_pin)) //Waiting low level end
			{
				udelay(5);
				count ++;
				if (count > 10)
					goto time_out;
			}
		}
    }  
    local_irq_restore(flags); 
	
    return dat;
    
time_out:
    local_irq_restore(flags); 
	//printk(KERN_WARNING"read_byte time_out.\n"); 
    return 0; 
}


//Work queue callback function for read DHT11
static void dht11_work_callback(struct work_struct *work)
{
    int i, count;
	
	gpio_direction_output(dh.da_pin, 1);
	
    mutex_lock(&dh.lock);
    //Start signal
    gpio_set_value(dh.da_pin, 1);
    udelay(2);
    gpio_set_value(dh.da_pin, 0);
    mdelay(20);  //Low level hold time cannot be less than 18ms
    gpio_set_value(dh.da_pin, 1); //Pull up 20-40us
    udelay(30);    
	  
    gpio_direction_input(dh.da_pin);
	

#if 0	
	count = 0;
	while(1 == gpio_get_value(dh.da_pin))
	{
		udelay(5);
		count ++;
		if (count > 5)
			goto time_out;
	}
		
#endif
	
    if (0 == gpio_get_value(dh.da_pin)) //Read response signal
    {
        count = 0;
        while(0 == gpio_get_value(dh.da_pin)) //Waiting for the response signal to end(80us)   
        {
            udelay(5);
            count ++;
            if (count > 20)
                goto time_out;
        }
        
        count = 0;
        while(1 == gpio_get_value(dh.da_pin)) //Waiting for DHT11 to pull up end(80us)
        {
            udelay(5);
            count ++;
            if (count > 20)
                goto time_out;
        }
		
        for (i=0; i<5; i++) //Start reading 40 bits of data
            dh.value[i] = read_byte();
    }
    else
        printk(KERN_WARNING"DHT11 response error.\n"); 
    //checksum
	
	//printk(KERN_INFO"dh.value[0]=%d dh.value[1]=%d dh.value[2]=%d dh.value[3]=%d dh.value[4]=%d \n", dh.value[0], dh.value[1], dh.value[2], dh.value[3], dh.value[4]);	
	
    if ((dh.value[0] + dh.value[1] + dh.value[2] + dh.value[3]) == dh.value[4]) 
    {
        dh.humdity = dh.value[0];
        dh.temperature = dh.value[2];
    }
    else
	{
		//printk(KERN_WARNING"DHT11 checksum error.\n"); 
	}
        
    mutex_unlock(&dh.lock); 
    return;
time_out:
    printk(KERN_WARNING"DHT11 timeout error.\n"); 
    mutex_unlock(&dh.lock); 
    return;
}

//Timercallback function for callback work queue 
static void dht11_timer_callback(unsigned long data)
{
    schedule_work(&dh.work);
    mod_timer(&dh.timer, jiffies + (1200 * HZ/1000)); //Modify a timer's timeout
}

static int dht11_open(struct inode *inode, struct file *file) 
{
	//printk(KERN_INFO"%s OK.\n",__func__);	
	
	return 0;
}

static ssize_t dht11_read(struct file* file, char __user *buf,
        size_t count, loff_t* fpos)
{
	//printk(KERN_INFO"%s OK.\n",__func__);

    return 0;
}

static long  dht11_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct dht11_value_msg dh_value_msg;

	switch(cmd) {
	case DHT11_IOCINIT:

		mutex_init(&dh.lock);
		mutex_lock(&dh.lock);

		//Timer
	    init_timer(&dh.timer);
	    dh.timer.function = dht11_timer_callback;
	    dh.timer.expires = jiffies + (1500 * HZ/1000); //1.5s (must > 1s)
	    dh.timer.data = ((unsigned long)0);
	    add_timer(&dh.timer);
	    //Workqueue
	    INIT_WORK(&dh.work, dht11_work_callback);

		mutex_unlock(&dh.lock);
	
		if (copy_from_user(&dh.da_pin, (int *)arg, sizeof(int)))
		{
			printk(KERN_ERR "DHT11_IOCINIT: copy_from_user() fail.\n");
			return -EINVAL;
		}
		 
	    if (gpio_request(dh.da_pin, "dht11_da_pin"))
	    {
			printk(KERN_ERR "DHT11_IOCINIT: gpio_request() fail.\n");
	        return -EBUSY;
	    }   
		break;	
		
	case DHT11_IOCGVALUE:
		dh_value_msg.humdity = dh.humdity;
		dh_value_msg.temperature = dh.temperature;
			
		if (copy_to_user((struct dht11_value_msg __user *)arg, &dh_value_msg, sizeof(dh_value_msg)))
			return -EFAULT;
		break;	
	
	default:
		ret = -ENOTTY;
		break;
	}
	
	return ret;
}

static int dht11_release(struct inode *inode, struct file *file) 
{
	//printk(KERN_INFO"%s OK.\n",__func__);	
	mutex_lock(&dh.lock);
	
	if(dh.da_pin)
		gpio_free(dh.da_pin);

	del_timer(&dh.timer);
	cancel_work_sync(&dh.work);

	mutex_unlock(&dh.lock);

	return 0;
}


static struct file_operations dht11_fops = {
	.owner = THIS_MODULE,
    .open = dht11_open,
    .read = dht11_read,
	.unlocked_ioctl = dht11_ioctl,
    .release = dht11_release,
};

static int dht11_drv_init(void)  
{  
  	int ret;
	dev_t dht11_devid;
	
	printk(KERN_INFO"%s OK.\n",__func__);
	if(alloc_chrdev_region(&dht11_devid, 0, 1, "dht11") < 0)
    {
        printk(KERN_ERR"Unable to alloc_chrdev_region.\n");
        return -EINVAL;
    } 
    dht11_major = MAJOR(dht11_devid);
	cdev_init(&dht11_cdev, &dht11_fops);        
    ret = cdev_add(&dht11_cdev, dht11_devid, 1);
    if (ret < 0)
    {
        printk(KERN_ERR "Unable to cdev_add.\n");
        goto error;
    }
        
    dht11_class = class_create(THIS_MODULE, "dht11"); 
    device_create(dht11_class, NULL, MKDEV(dht11_major, 0), NULL, "dht11"); 

	return 0;
error:
    unregister_chrdev_region(MKDEV(dht11_major, 0), 1);
    return -EINVAL;
	
}  
  
static void dht11_drv_exit(void)  
{  
    printk(KERN_INFO"%s OK.\n",__func__);
	
	device_destroy(dht11_class,  MKDEV(dht11_major, 0));
    class_destroy(dht11_class);
 
    unregister_chrdev_region(MKDEV(dht11_major, 0), 1);
    cdev_del(&dht11_cdev);
}  

module_init(dht11_drv_init);
module_exit(dht11_drv_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("hceng <huangcheng.job@foxmail.com>");
MODULE_DESCRIPTION("dht11 driver.");
MODULE_VERSION("v2.0");
