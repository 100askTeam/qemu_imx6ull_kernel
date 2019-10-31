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


#define DS18B20_IOC_MAGIC    'd'
#define DS18B20_IOCINIT      _IOW(DS18B20_IOC_MAGIC, 0, int)
#define DS18B20_IOCGVALUE    _IOWR(DS18B20_IOC_MAGIC, 1, void *)
#define DS18B20_IOCGINFO     _IOWR(DS18B20_IOC_MAGIC, 2, void *)


struct ds18b20_value_msg {
	u8 value[2];
};

struct ds18b20_info_msg {
    int dq_pin;
    u8 family_code;
    u8 serial_num[6];
    u8 crc;
};


static int ds18b20_major;
static struct cdev ds18b20_cdev;
static struct class *ds18b20_class;

struct ds18b20 {
    struct mutex lock;  
    int dq_pin;
    u8 value[2];
    u8 family_code;
    u8 serial_num[6];
    u8 crc;
};
static struct ds18b20 ds;


static int ds18b20_init(void)
{
    int ret = 1;
    
	gpio_direction_output(ds.dq_pin, 1);
	
    mutex_lock(&ds.lock);

    gpio_set_value(ds.dq_pin, 1);
    udelay(2);
    gpio_set_value(ds.dq_pin, 0); //Low level 480us for reset
    udelay(480);                      
    gpio_set_value(ds.dq_pin, 1); //Pull high release bus
	
	gpio_direction_input(ds.dq_pin); //Read response pulse
	
    udelay(35);
    
    ret = gpio_get_value(ds.dq_pin);
    udelay(250);  //Waiting for the corresponding end
    
    mutex_unlock(&ds.lock);
    
    return ret;
}


static void write_byte(unsigned char data)
{
    int i = 0;
    unsigned long flags;

    mutex_lock(&ds.lock);
    
    local_irq_save(flags); //Save interrupt
    //local_irq_disable(); //Close all interrupts

    gpio_direction_output(ds.dq_pin, 1); 

    for (i = 0; i < 8; i ++)
    {
        gpio_set_value(ds.dq_pin, 1); 
        udelay(2);    
        gpio_set_value(ds.dq_pin, 0); //Start at a low level greater than 1us
        udelay(5);
        
        gpio_set_value(ds.dq_pin, data & 0x01);  
        udelay(60); //Write cycle is greater than 60us
        
        data >>= 1;   
    }
    local_irq_restore(flags); //Recovery interrupt
    //local_irq_enable(); //Open all interrupts

    mutex_unlock(&ds.lock); 

}

static unsigned char read_byte(void)    
{    
    int i;    
    unsigned long flags;
    unsigned char data = 0;    

    mutex_lock(&ds.lock);
    
    local_irq_save(flags);
    //local_irq_disable();
	
    for (i = 0; i < 8; i++)    
    {    
        gpio_direction_output(ds.dq_pin, 1);    
        udelay(2);    
        gpio_set_value(ds.dq_pin, 0); //Start at a low level greater than 1us
        udelay(1);    
        
        //gpio_set_value(ds.dq_pin, 1); //Pull high release bus 
        //udelay(1); 
      
        data >>= 1;   
        gpio_direction_input(ds.dq_pin);
        if (gpio_get_value(ds.dq_pin)) //Must be read within 15us after being pulled low
            data |= 0x80;  
      
        udelay(60); //Read cycle is greater than 60us;   
    }    
    
    local_irq_restore(flags); 
    //local_irq_enable();

    mutex_unlock(&ds.lock);

    return data;    
} 


static int ds18b20_get_sensor_value(void)
{
    int ret;

    ret = ds18b20_init(); //Reset initialization of the DS18B20 before each read and write
    if(0 != ret)
    {
        printk(KERN_ERR"%s ds18b20_init error.\n",__func__);
        return -1;    
    }
    write_byte(0xCC); //Skip commands for ROM operations
    write_byte(0x44); //Start the DS18B20 acquisition temperature
	
    ret = ds18b20_init(); 
    if(0 != ret)
    {
        printk(KERN_ERR"%s ds18b20_init error.\n",__func__);
        return -1;    
    }
    write_byte(0xCC); //Skip commands for ROM operations   
    write_byte(0xBE); //Read the data in the DS18B20 register

    ds.value[0] = read_byte(); //Low byte
    ds.value[1] = read_byte(); //High byte 

    return ret;
}

static int ds18b20_get_sensor_info(void)
{
    int i, ret;

    ret = ds18b20_init();  

    if(0 != ret)
    {
        //printk(KERN_ERR"%s ds18b20_init error.\n",__func__);
        return -1;    
    }

    write_byte(0x33); //Read ROM command
    
    ds.family_code = read_byte(); //The lower 8 bits is the family code(28h)
    
    for (i=0; i<6; i++) //The middle 48 bits are the unique serial number
        ds.serial_num[i] = read_byte();
  
    ds.crc = read_byte(); //The high 8 bits are the CRC check data.

    return ret;
}



static int ds18b20_open(struct inode *inode, struct file *file) 
{
	//printk(KERN_INFO"%s OK.\n",__func__);	
	mutex_init(&ds.lock);
	
	return 0;
}


static ssize_t ds18b20_read(struct file* file, char __user *buf,
        size_t count, loff_t* fpos)
{
	//printk(KERN_INFO"%s OK.\n",__func__);
    return 0;
}

static long  ds18b20_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct ds18b20_value_msg ds_value_msg;
	struct ds18b20_info_msg ds_info_msg;

	switch(cmd) {
	case DS18B20_IOCINIT:
		if (copy_from_user(&ds.dq_pin, (int *)arg, sizeof(int)))
		{
			printk(KERN_ERR "DS18B20_IOCINIT: copy_from_user() fail.\n");
			return -EINVAL;
		}
	    if (gpio_request(ds.dq_pin, "ds18b20_dq_pin"))
	    {
			printk(KERN_ERR "DS18B20_IOCINIT: gpio_request() fail.\n");
	        return -EBUSY;
	    }   

		ret = ds18b20_init(); //Reset initialization of the DS18B20 before each read and write
		if(0 != ret)
		{
			printk(KERN_ERR"%s ds18b20_init error.\n",__func__);
			return -1;    
		}
		write_byte(0xCC); //Skip commands for ROM operations
		write_byte(0x44); //Start the DS18B20 acquisition temperature
	
		
		break;	
	case DS18B20_IOCGVALUE:
		ret = ds18b20_get_sensor_value();
		if (ret == 0)
		{
			ds_value_msg.value[0] = ds.value[0];
			ds_value_msg.value[1] = ds.value[1];
			//printk("ds.value[0]=%d ds.value[1]=%d \n", ds.value[0], ds.value[0]);
		}
		else
		{
			//printk(KERN_ERR "DS18B20_IOCGVALUE: ds18b20_get_sensor_value() fail.\n");
			return -EBUSY;
		}
			
		if (copy_to_user((struct ds18b20_value_msg __user *)arg, &ds_value_msg, sizeof(ds_value_msg)))
			return -EFAULT;
		break;	
	case DS18B20_IOCGINFO:
		if (ds18b20_get_sensor_info() == 0)
		{
			ds_info_msg.dq_pin = ds.dq_pin;
			ds_info_msg.family_code = ds.family_code;
			memcpy(&ds_info_msg.serial_num ,&ds.serial_num,sizeof(ds.serial_num));
			ds_info_msg.crc = ds.crc;
		}
		else
		{
			//printk(KERN_ERR "DS18B20_IOCGINFO: ds18b20_get_sensor_info() fail.\n");
			return -EBUSY;
		}
		if (copy_to_user((struct ds18b20_info_msg __user *)arg, &ds_info_msg, sizeof(ds_info_msg)))
			 return -EFAULT;
		break;	
	default:
		ret = -ENOTTY;
		break;
	}
	
	return ret;
}

static int ds18b20_release(struct inode *inode, struct file *file) 
{
	//printk(KERN_INFO"%s OK.\n",__func__);	

	if(ds.dq_pin)
		gpio_free(ds.dq_pin);
	
	return 0;
}


static struct file_operations ds18b20_fops = {
	.owner = THIS_MODULE,
    .open = ds18b20_open,
    .read = ds18b20_read,
	.unlocked_ioctl = ds18b20_ioctl,
    .release = ds18b20_release,
};

static int ds18b20_drv_init(void)  
{  
  	int ret;
	dev_t ds18b20_devid;
	
	printk(KERN_INFO"%s OK.\n",__func__);
	if(alloc_chrdev_region(&ds18b20_devid, 0, 1, "ds18b20") < 0)
    {
        printk(KERN_ERR"Unable to alloc_chrdev_region.\n");
        return -EINVAL;
    } 
    ds18b20_major = MAJOR(ds18b20_devid);
	cdev_init(&ds18b20_cdev, &ds18b20_fops);        
    ret = cdev_add(&ds18b20_cdev, ds18b20_devid, 1);
    if (ret < 0)
    {
        printk(KERN_ERR "Unable to cdev_add.\n");
        goto error;
    }
        
    ds18b20_class = class_create(THIS_MODULE, "ds18b20"); 
    device_create(ds18b20_class, NULL, MKDEV(ds18b20_major, 0), NULL, "ds18b20"); 

	return 0;
error:
    unregister_chrdev_region(MKDEV(ds18b20_major, 0), 1);
    return -EINVAL;
	
}  
  
static void ds18b20_drv_exit(void)  
{  
    printk(KERN_INFO"%s OK.\n",__func__);
	
	device_destroy(ds18b20_class,  MKDEV(ds18b20_major, 0));
    class_destroy(ds18b20_class);
 
    unregister_chrdev_region(MKDEV(ds18b20_major, 0), 1);
    cdev_del(&ds18b20_cdev);
}  

module_init(ds18b20_drv_init);
module_exit(ds18b20_drv_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("hceng <huangcheng.job@foxmail.com>");
MODULE_DESCRIPTION("ds18b20 driver.");
MODULE_VERSION("v2.0");