#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/div64.h>

#include <asm/mach/map.h>

#define LCD_XRES  500
#define LCD_YRES  300
#define LCD_BPP   16
#define FSL_IMX6UL_LCDIF_ADDR 0x021C8000

static int s3c_lcdfb_setcolreg(unsigned int regno, unsigned int red,
			     unsigned int green, unsigned int blue,
			     unsigned int transp, struct fb_info *info);


struct lcd_regs {
	unsigned long	fb_base_phys;
	unsigned long	fb_xres;
	unsigned long	fb_yres;
	unsigned long	fb_bpp;
};

static struct fb_ops s3c_lcdfb_ops = {
	.owner		= THIS_MODULE,
	.fb_setcolreg	= s3c_lcdfb_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};


static struct fb_info *s3c_lcd;
static volatile struct lcd_regs* lcd_regs;
static u32 pseudo_palette[16];


/* from pxafb.c */
static inline unsigned int chan_to_field(unsigned int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}


static int s3c_lcdfb_setcolreg(unsigned int regno, unsigned int red,
			     unsigned int green, unsigned int blue,
			     unsigned int transp, struct fb_info *info)
{
	unsigned int val;
	
	if (regno > 16)
		return 1;

	/* ��red,green,blue��ԭɫ�����val */
	val  = chan_to_field(red,	&info->var.red);
	val |= chan_to_field(green, &info->var.green);
	val |= chan_to_field(blue,	&info->var.blue);
	
	//((u32 *)(info->pseudo_palette))[regno] = val;
	pseudo_palette[regno] = val;
	return 0;
}

static int lcd_init(void)
{
	printk("%s %s %d\n", __FILE__, __FUNCTION__, __LINE__);
	/* 1. ����һ��fb_info */
	s3c_lcd = framebuffer_alloc(0, NULL);

	/* 2. ���� */
	/* 2.1 ���ù̶��Ĳ��� */
	strcpy(s3c_lcd->fix.id, "mylcd");
	s3c_lcd->fix.smem_len = LCD_XRES*LCD_YRES*LCD_BPP/8;
	s3c_lcd->fix.type     = FB_TYPE_PACKED_PIXELS;
	s3c_lcd->fix.visual   = FB_VISUAL_TRUECOLOR; /* TFT */
	s3c_lcd->fix.line_length = LCD_XRES*2;
	
	/* 2.2 ���ÿɱ�Ĳ��� */
	s3c_lcd->var.xres           = LCD_XRES;
	s3c_lcd->var.yres           = LCD_YRES;
	s3c_lcd->var.xres_virtual   = LCD_XRES;
	s3c_lcd->var.yres_virtual   = LCD_YRES;
	s3c_lcd->var.bits_per_pixel = LCD_BPP;

	/* RGB:565 */
	s3c_lcd->var.red.offset     = 11;
	s3c_lcd->var.red.length     = 5;
	
	s3c_lcd->var.green.offset   = 5;
	s3c_lcd->var.green.length   = 6;

	s3c_lcd->var.blue.offset    = 0;
	s3c_lcd->var.blue.length    = 5;

	s3c_lcd->var.activate       = FB_ACTIVATE_NOW;
	
	
	/* 2.3 ���ò������� */
	s3c_lcd->fbops              = &s3c_lcdfb_ops;
	
	/* 2.4 ���������� */
	s3c_lcd->pseudo_palette = pseudo_palette;
	//s3c_lcd->screen_base  = ;  /* �Դ�������ַ */ 
	s3c_lcd->screen_size   = LCD_XRES*LCD_YRES*LCD_BPP/8;

	/* 3. Ӳ����صĲ��� */
	/* 3.1 ����GPIO����LCD */
	
	/* 3.2 ����LCD�ֲ�����LCD������, ����VCLK��Ƶ�ʵ� */
	lcd_regs = ioremap(FSL_IMX6UL_LCDIF_ADDR, sizeof(struct lcd_regs));
	
	/* 3.3 �����Դ�(framebuffer), ���ѵ�ַ����LCD������ */
	s3c_lcd->screen_base = dma_alloc_writecombine(NULL, s3c_lcd->fix.smem_len, (dma_addr_t *)&s3c_lcd->fix.smem_start, GFP_KERNEL);

	/* ����LCD������ */
	printk("%s %s %d\n", __FILE__, __FUNCTION__, __LINE__);
	lcd_regs->fb_base_phys = s3c_lcd->fix.smem_start;
	lcd_regs->fb_xres      = LCD_XRES;
	lcd_regs->fb_yres      = LCD_YRES;
	lcd_regs->fb_bpp       = LCD_BPP;
	
	
	//s3c_lcd->fix.smem_start = xxx;  /* �Դ�������ַ */
	/* ����LCD */

	/* 4. ע�� */
	printk("%s %s %d\n", __FILE__, __FUNCTION__, __LINE__);
	register_framebuffer(s3c_lcd);
	
	return 0;
}

static void lcd_exit(void)
{
	unregister_framebuffer(s3c_lcd);
	dma_free_writecombine(NULL, s3c_lcd->fix.smem_len, s3c_lcd->screen_base, s3c_lcd->fix.smem_start);
	iounmap(lcd_regs);
	framebuffer_release(s3c_lcd);
}

module_init(lcd_init);
module_exit(lcd_exit);

MODULE_LICENSE("GPL");


