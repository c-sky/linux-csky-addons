/*
 * C-SKY SoCs LCDC driver
 *
 * Copyright (C) 2017 C-SKY MicroSystems Co.,Ltd.
 *
 * Author: Lei Ling <lei_ling@c-sky.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include "csky-fb.h"

#define DRIVER_NAME "csky_fb"

#define VSYNC_TIMEOUT_MSEC	100


static void csky_fb_enable_irq(struct csky_fb_info *info, u32 mask)
{
	u32 tmp;
	tmp = readl(info->iobase + CSKY_LCD_INT_MASK);
	tmp |= mask;
	writel(tmp, info->iobase + CSKY_LCD_INT_MASK);
	return;
}

static void csky_fb_disable_irq(struct csky_fb_info *info, u32 mask)
{
	u32 tmp;
	tmp = readl(info->iobase + CSKY_LCD_INT_MASK);
	tmp &= ~mask;
	writel(tmp, info->iobase + CSKY_LCD_INT_MASK);
	return;
}

/*
 * sleep until next VSYNC interrupt or timeout
 */
static int csky_fb_wait_for_vsync(struct fb_info *fbinfo)
{
	struct csky_fb_info *info = fbinfo->par;
	unsigned long count;
	int ret;

	count = info->vsync_info.count;
	csky_fb_enable_irq(info, CSKY_LCDINT_MASK_BAU);
	ret = wait_event_interruptible_timeout(
				info->vsync_info.wait,
				count != info->vsync_info.count,
				msecs_to_jiffies(VSYNC_TIMEOUT_MSEC));
	if (ret == 0)
		return -ETIMEDOUT;

	return 0;
}

static void csky_fb_set_pixel_format(struct csky_fb_info *info,
				     enum csky_fb_pixel_format fmt)
{
	u32 control;

	control = readl(info->iobase + CSKY_LCD_CONTROL);
	control &= ~CSKY_LCDCON_DFS_MASK_SHIFTED;
	control |= fmt;
	writel(control, info->iobase + CSKY_LCD_CONTROL);

	info->pixel_fmt = fmt;
	return;
}

static void csky_fb_set_lcd_pbase(struct csky_fb_info *info,
				  struct csky_fb_lcd_pbase_yuv *pbase)
{
	writel(pbase->y, info->iobase + CSKY_LCD_PBASE_Y);
	writel(pbase->u, info->iobase + CSKY_LCD_PBASE_U);
	writel(pbase->v, info->iobase + CSKY_LCD_PBASE_V);
	info->pbase_yuv.y = pbase->y;
	info->pbase_yuv.u = pbase->u;
	info->pbase_yuv.v = pbase->v;
	return;
}

/*
 * enable/disable lcdc
 */
static void csky_fb_lcd_enable(struct csky_fb_info *info, bool enable)
{
	u32 control;

	control = readl(info->iobase + CSKY_LCD_CONTROL);

	if (enable) {
		control |= CSKY_LCDCON_LEN;
		info->lcdc_enabled = true;
	}
	else {
		control &= ~CSKY_LCDCON_LEN;
		info->lcdc_enabled = false;
	}

	writel(control, info->iobase + CSKY_LCD_CONTROL);
	return;
}

static void csky_fb_lcd_reset(struct csky_fb_info *info)
{
	reset_control_assert(info->rst);
	mdelay(1); /* delay >1us */
	reset_control_deassert(info->rst);
	mdelay(1);

	info->lcdc_enabled = false;
	return;
}

static int csky_fb_init_registers(struct fb_info *fbinfo)
{
	struct csky_fb_info *info = fbinfo->par;
	u32 videosize;

	u32 control = info->pixel_fmt |
		      CSKY_LCDCON_OUT_24BIT |
		      CSKY_LCDCON_WML_8WORD |
		      CSKY_LCDCON_VBL_16CYCLES |
		      CSKY_LCDCON_PAS_TFT;
	u32 timing0 = (((info->vm.hback_porch - 1) & 0xff) << 24) |
		      (((info->vm.hfront_porch - 1) & 0xff) << 16) |
		      (((info->vm.hsync_len - 1) & 0x3f) << 10) |
		      ((info->vm.hactive - 1) & 0x3ff);
	u32 timing1 = (((info->vm.vback_porch - 1) & 0xff) << 24) |
		      (((info->vm.vfront_porch - 1) & 0xff) << 16) |
		      (((info->vm.vsync_len - 1) & 0x3f) << 10) |
		      ((info->vm.vactive - 1) & 0x3ff);
	u32 timing2 = 0;

	if (info->pixel_clock_pol)
		timing2 |= CSKY_LCDTIM2_PCP_FALLING;
	if (info->hsync_pulse_pol)
		timing2 |= CSKY_LCDTIM2_HSP_ACT_LOW;
	if (info->vsync_pulse_pol)
		timing2 |= CSKY_LCDTIM2_VSP_ACT_LOW;
	if (info->pixel_clk_src)
		timing2 |= CSKY_LCDTIM2_CLKS_EXT;
	timing2 |= info->pcd & 0xff;

	writel(timing0, info->iobase + CSKY_LCD_TIMING0);
	writel(timing1, info->iobase + CSKY_LCD_TIMING1);
	writel(timing2, info->iobase + CSKY_LCD_TIMING2);

	/* disable all the lcdc interrupts */
	writel(0x0, info->iobase + CSKY_LCD_INT_MASK);

	videosize = ((info->vm.vactive - 1) << 11) |
		    (info->vm.hactive - 1);
	writel(videosize, info->iobase + CSKY_LCD_VIDEOSIZE);

	/* set base address */
	writel(fbinfo->fix.smem_start, info->iobase + CSKY_LCD_PBASE);
	writel(info->pbase_yuv.y, info->iobase + CSKY_LCD_PBASE_Y);
	writel(info->pbase_yuv.u, info->iobase + CSKY_LCD_PBASE_U);
	writel(info->pbase_yuv.v, info->iobase + CSKY_LCD_PBASE_V);

	/* enable lcdc */
	control |= CSKY_LCDCON_LEN;
	writel(control, info->iobase + CSKY_LCD_CONTROL);

	/* skip the vsync interrupt triggered by enabling the LCDC */
	csky_fb_wait_for_vsync(fbinfo);

	info->lcdc_enabled = true;
	return 0;
}

static int csky_fb_check_var(struct fb_var_screeninfo *var,
			     struct fb_info *fbinfo)
{
	struct csky_fb_info *info = fbinfo->par;

	var->xres = info->vm.hactive;
	var->yres = info->vm.vactive;
	var->xres_virtual = var->xres;
	var->yres_virtual = var->yres * 2; /* double buffer supported */

	var->width = info->vm.hactive;
	var->height = info->vm.vactive;

	var->pixclock = info->vm.pixelclock;
	var->left_margin = info->vm.hback_porch;
	var->right_margin = info->vm.hfront_porch;
	var->upper_margin = info->vm.vback_porch;
	var->lower_margin = info->vm.vfront_porch;
	var->hsync_len = info->vm.hsync_len;
	var->vsync_len = info->vm.vsync_len;

	var->transp.offset = 0;
	var->transp.length = 0;
	switch (var->bits_per_pixel) {
	case 32:
		var->red.length = 8;
		var->red.offset = 16;
		var->green.length = 8;
		var->green.offset = 8;
		var->blue.length = 8;
		var->blue.offset = 0;
		var->transp.length = 0; /* no support for transparency */
		var->transp.offset = 24;
		break;
	default:
		dev_err(info->dev, "bits_per_pixel(%d) not supported\n",
			var->bits_per_pixel);
		break;
	}
	return 0;
}

static int csky_fb_set_par(struct fb_info *fbinfo)
{
	return 0;
}

static int csky_fb_blank(int blank_mode, struct fb_info *fbinfo)
{
	struct csky_fb_info *info = fbinfo->par;

	switch (blank_mode) {
	case FB_BLANK_UNBLANK:
		/* enable/init lcdc */
		if (!info->lcdc_enabled)
			csky_fb_init_registers(fbinfo);
		break;
	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		/* disable/reset lcdc */
		if (info->lcdc_enabled) {
			csky_fb_wait_for_vsync(fbinfo);
			csky_fb_lcd_reset(info);
		}
		break;
	default:
		break;
	}
	return 0;
}

static int csky_fb_setcolreg(unsigned regno,
			     unsigned red, unsigned green, unsigned blue,
			     unsigned transp, struct fb_info *info)
{
	return 0;
}

static int csky_fb_pan_display(struct fb_var_screeninfo *var,
			       struct fb_info *fbinfo)
{
	struct csky_fb_info *info = fbinfo->par;
	unsigned long base;

	/*
	 * adjust the position of visible screen
	 * note: xoffset not supported
	 */
	base = fbinfo->fix.smem_start +
	       var->yoffset * fbinfo->fix.line_length;
	writel(base, info->iobase + CSKY_LCD_PBASE);
	return 0;
}

static int csky_fb_ioctl(struct fb_info *fbinfo,
			 unsigned int cmd,
			 unsigned long arg)
{
	struct csky_fb_info *info = fbinfo->par;
	void __user *argp = (void __user *)arg;
	int ret = 0;

	switch (cmd) {
	case FBIO_WAITFORVSYNC:
		ret = csky_fb_wait_for_vsync(fbinfo);
		break;

	case CSKY_FBIO_SET_PIXEL_FMT:
		{
			enum csky_fb_pixel_format fmt;

			if (copy_from_user(&fmt, argp, sizeof(fmt))) {
				ret = -EFAULT;
				break;
			}

			if (fmt == info->pixel_fmt)
				break;

			csky_fb_set_pixel_format(info, fmt);
			break;
		}

	case CSKY_FBIO_GET_PIXEL_FMT:
		{
			if (copy_to_user(argp, &info->pixel_fmt,
					 sizeof(info->pixel_fmt)))
				ret = -EFAULT;
			break;
		}

	case CSKY_FBIO_SET_PBASE_YUV:
		{
			struct csky_fb_lcd_pbase_yuv base;

			if (copy_from_user(&base, argp, sizeof(base))) {
				ret = -EFAULT;
				break;
			}

			if ((base.y == info->pbase_yuv.y) &&
			    (base.u == info->pbase_yuv.u) &&
			    (base.v == info->pbase_yuv.v))
				break;

			csky_fb_set_lcd_pbase(info, &base);
			break;
		}
	default:
		ret = -ENOIOCTLCMD;
	}

	return ret;
}

static struct fb_ops csky_fb_ops = {
	.owner          = THIS_MODULE,
	.fb_check_var   = csky_fb_check_var,
	.fb_set_par     = csky_fb_set_par,
	.fb_blank       = csky_fb_blank,
	.fb_setcolreg   = csky_fb_setcolreg,
	.fb_pan_display = csky_fb_pan_display,
	.fb_ioctl       = csky_fb_ioctl,
	.fb_fillrect    = cfb_fillrect,
	.fb_copyarea    = cfb_copyarea,
	.fb_imageblit   = cfb_imageblit,
};

static int csky_fb_map_video_memory(struct fb_info *fbinfo)
{
	struct csky_fb_info *info = fbinfo->par;
	dma_addr_t map_dma;
	unsigned map_size = PAGE_ALIGN(fbinfo->fix.smem_len);

	fbinfo->screen_base = dma_alloc_coherent(info->dev, map_size,
						 &map_dma, GFP_KERNEL);
	if (fbinfo->screen_base == NULL) {
		return -ENOMEM;
	}

	memset(fbinfo->screen_base, 0, map_size);
	fbinfo->fix.smem_start = map_dma;
	fbinfo->fix.smem_len = map_size;

	info->pbase_yuv.y = fbinfo->fix.smem_start;
	info->pbase_yuv.u = info->pbase_yuv.y;
	info->pbase_yuv.v = info->pbase_yuv.y;
	return 0;
}

static inline void csky_fb_unmap_video_memory(struct fb_info *fbinfo)
{
	struct csky_fb_info *info = fbinfo->par;

	dma_free_coherent(info->dev,
			  PAGE_ALIGN(fbinfo->fix.smem_len),
			  fbinfo->screen_base,
			  fbinfo->fix.smem_start);
	return;
}

static irqreturn_t csky_fb_irq(int irq, void *dev_id)
{
	struct csky_fb_info *info = dev_id;
	unsigned long status;

	raw_spin_lock(&info->slock);

	status = readl(info->iobase + CSKY_LCD_INT_STAT);
	/* clear interrupts */
	writel(status, info->iobase + CSKY_LCD_INT_STAT);

	if (status & CSKY_LCDINT_STAT_LDD) {
		dev_info(info->dev, "LCD_INT: LCD has been disabled\n");
	}

	if (status & CSKY_LCDINT_STAT_BAU) { /* VSYNC interrupt */
		info->vsync_info.count++;
		wake_up_interruptible(&info->vsync_info.wait);
		/* disable vsync interrupt */
		csky_fb_disable_irq(info, CSKY_LCDINT_MASK_BAU);
	}

	if (status & CSKY_LCDINT_STAT_BER) {
		dev_info(info->dev, "LCD_INT: Bus error\n");
	}

	if (status & CSKY_LCDINT_STAT_LFU) {
		dev_info(info->dev, "LCD_INT: Line FIFO underrun\n");
	}

	raw_spin_unlock(&info->slock);
	return IRQ_HANDLED;
}

static int csky_fb_probe(struct platform_device *pdev)
{
	int irq;
	struct resource *mem;
	void __iomem *iobase;
	struct reset_control *rst;
	u32 bits_per_pixel;
	u32 pixel_clk_src; /* pixel clock source */
	u32 pcd; /* pixel clock divider. f=HCLK/2(pcd+1) */
	u32 hsync_pulse_pol; /* HSYNC pulse polarity */
	u32 vsync_pulse_pol; /* VSYNC pulse polarity */
	u32 pixel_clock_pol; /* pixel clock polarity */
	struct device *dev = &pdev->dev;
	struct videomode vm;
	struct fb_info *fbinfo;
	struct csky_fb_info *info;
	int ret;

	/* get resources */

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iobase = devm_ioremap_resource(dev, mem);
	if (IS_ERR(iobase))
		return PTR_ERR(iobase);

	rst = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(rst)) {
		dev_err(&pdev->dev, "Failed to get reset control\n");
		return PTR_ERR(rst);
	}

	ret = of_property_read_u32(dev->of_node, "bits-per-pixel",
				   &bits_per_pixel);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to get property bits-per-pixel\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node,
				   "pixel-clock-source", &pixel_clk_src);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Failed to get property pixel-clock-source\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "pcd", &pcd);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to get property pcd\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node,
				   "hsync-pulse-pol", &hsync_pulse_pol);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to get property hsync-pulse-pol\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node,
				   "vsync-pulse-pol", &vsync_pulse_pol);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to get property vsync-pulse-pol\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node,
				   "pixel-clock-pol", &pixel_clock_pol);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to get property pixel-clock-pol\n");
		return ret;
	}

	ret = of_get_videomode(dev->of_node, &vm, OF_USE_NATIVE_MODE);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get videomode from DT\n");
		return ret;
	}

	/* allocate a fb_info */
	fbinfo = framebuffer_alloc(sizeof(struct csky_fb_info), &pdev->dev);
	if (!fbinfo)
		return -ENOMEM;
	platform_set_drvdata(pdev, fbinfo);

	/* initialize the fb_info */

	info = fbinfo->par;
	spin_lock_init(&info->slock);
	info->dev = &pdev->dev;
	info->iobase = iobase;
	info->rst = rst;
	info->irq = irq;
	memcpy(&info->vm, &vm, sizeof(vm));
	info->pixel_clk_src = pixel_clk_src;
	info->pcd = pcd;
	info->hsync_pulse_pol = hsync_pulse_pol;
	info->vsync_pulse_pol = vsync_pulse_pol;
	info->pixel_clock_pol = pixel_clock_pol;
	init_waitqueue_head(&info->vsync_info.wait);
	info->pixel_fmt = CSKY_LCDCON_DFS_RGB;

	strcpy(fbinfo->fix.id, DRIVER_NAME);
	fbinfo->fix.smem_len = vm.hactive * vm.vactive *
			       (bits_per_pixel / 8) *
			       2; /* double buffer supported */
	fbinfo->fix.type = FB_TYPE_PACKED_PIXELS;
	fbinfo->fix.type_aux = 0;
	fbinfo->fix.visual = FB_VISUAL_TRUECOLOR;
	fbinfo->fix.xpanstep = 0;
	fbinfo->fix.ypanstep = 1; /* double buffer supported */
	fbinfo->fix.ywrapstep = 0;
	fbinfo->fix.line_length = vm.hactive * (bits_per_pixel / 8);
	fbinfo->fix.mmio_start = mem->start;
	fbinfo->fix.mmio_len = mem->end - mem->start + 1;
	fbinfo->fix.accel = FB_ACCEL_NONE;

	fbinfo->var.xres = vm.hactive;
	fbinfo->var.yres = vm.vactive;
	fbinfo->var.xres_virtual = fbinfo->var.xres;
	fbinfo->var.yres_virtual = fbinfo->var.yres;
	fbinfo->var.xoffset = 0;
	fbinfo->var.yoffset = 0;
	fbinfo->var.bits_per_pixel = bits_per_pixel;
	fbinfo->var.grayscale = 0;
	fbinfo->var.nonstd = 0;
	fbinfo->var.activate = FB_ACTIVATE_NOW;
	fbinfo->var.accel_flags = 0;
	fbinfo->var.vmode = FB_VMODE_NONINTERLACED;

	fbinfo->fbops = &csky_fb_ops;
	fbinfo->flags = FBINFO_FLAG_DEFAULT;

	/* allocate video memory */
	ret = csky_fb_map_video_memory(fbinfo);
	if (ret) {
		dev_err(&pdev->dev, "Failed to allocate video memory\n");
		goto RELEASE_FBINFO;
	}

	csky_fb_check_var(&fbinfo->var, fbinfo);

	/* csky_fb_init_registers(fbinfo); */

	ret = register_framebuffer(fbinfo);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register framebuffer device\n");
		goto FREE_VIDEO_MEMORY;
	}

	ret = request_irq(irq, csky_fb_irq, 0, pdev->name, info);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get irq %d, err %d\n", irq, ret);
		goto UNREGISTER_FB;
	}

	dev_info(&pdev->dev, "fb%d: %s frame buffer device\n",
		 fbinfo->node, fbinfo->fix.id);
	return 0;

UNREGISTER_FB:
	unregister_framebuffer(fbinfo);
FREE_VIDEO_MEMORY:
	csky_fb_unmap_video_memory(fbinfo);
RELEASE_FBINFO:
	framebuffer_release(fbinfo);
	return ret;
}

static int csky_fb_remove(struct platform_device *pdev)
{
	struct fb_info *fbinfo = platform_get_drvdata(pdev);
	struct csky_fb_info *info = fbinfo->par;

	csky_fb_lcd_reset(info);

	free_irq(info->irq, info);

	unregister_framebuffer(fbinfo);

	csky_fb_unmap_video_memory(fbinfo);

	platform_set_drvdata(pdev, NULL);

	framebuffer_release(fbinfo);
	return 0;
}

static const struct of_device_id csky_fb_of_dev_id[] = {
	{ .compatible = "csky,eragon-lcdc", },
	{ }
};
MODULE_DEVICE_TABLE(of, csky_fb_of_dev_id);

static struct platform_driver csky_fb_driver = {
	.probe	= csky_fb_probe,
	.remove	= csky_fb_remove,
	.driver	= {
		.name = DRIVER_NAME,
		.of_match_table = csky_fb_of_dev_id,
	}
};

module_platform_driver(csky_fb_driver);

MODULE_DESCRIPTION("C-SKY SoCs LCDC Driver");
MODULE_AUTHOR("Lei Ling <lei_ling@c-sky.com>");
MODULE_LICENSE("GPL v2");