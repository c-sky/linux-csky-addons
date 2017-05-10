/*
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
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
#include "csky-fb.h"

#define DRIVER_NAME "csky_fb"

/*
 * enable/disable lcdc
 */
static void csky_fb_lcd_enable(struct csky_fb_info *info, bool enable)
{
	u32 control;

	control = readl(info->iobase + CSKY_LCD_CONTROL);

	if (enable)
		control |= CSKY_LCDCON_LEN;
	else
		control &= ~CSKY_LCDCON_LEN;

	writel(control, info->iobase + CSKY_LCD_CONTROL);
	return;
}

static int csky_fb_check_var(struct fb_var_screeninfo *var,
			     struct fb_info *fbinfo)
{
	struct csky_fb_info *info = fbinfo->par;

	var->xres_virtual = var->xres = info->vm.hactive;
	var->yres_virtual = var->yres = info->vm.vactive;

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
		printk("error! bits_per_pixel(%d) not supported\n",
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
		/* turn on lcdc */
		csky_fb_lcd_enable(info, true);
		break;
	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		/* turn off lcdc */
		csky_fb_lcd_enable(info, false);
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

static int csky_fb_mmap(struct fb_info *fbinfo,
			struct vm_area_struct *vma)
{
	return remap_vmalloc_range(vma, (void *)fbinfo->fix.smem_start, vma->vm_pgoff);
}

static struct fb_ops csky_fb_ops = {
	.owner          = THIS_MODULE,
	.fb_check_var   = csky_fb_check_var,
	.fb_set_par     = csky_fb_set_par,
	.fb_blank       = csky_fb_blank,
	.fb_setcolreg   = csky_fb_setcolreg,
	.fb_mmap	= csky_fb_mmap,
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

static void csky_fb_set_lcdaddr(struct fb_info *fbinfo)
{
	struct csky_fb_info *info = fbinfo->par;

	printk("LCD_PBASE = 0x%X\n", fbinfo->fix.smem_start);
	writel(fbinfo->fix.smem_start, info->iobase + CSKY_LCD_PBASE);
	return;
}

static int csky_fb_init_registers(struct fb_info *fbinfo)
{
	struct csky_fb_info *info = fbinfo->par;
	u32 videosize;

	u32 control = CSKY_LCDCON_DFS_RGB |
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

	writel(control, info->iobase + CSKY_LCD_CONTROL);
	writel(timing0, info->iobase + CSKY_LCD_TIMING0);
	writel(timing1, info->iobase + CSKY_LCD_TIMING1);
	writel(timing2, info->iobase + CSKY_LCD_TIMING2);
	writel(0x0, info->iobase + CSKY_LCD_INT_MASK);
	videosize = ((info->vm.vactive - 1) << 11) |
		    (info->vm.hactive - 1);
	writel(videosize, info->iobase + CSKY_LCD_VIDEOSIZE);

	csky_fb_set_lcdaddr(fbinfo);

	/* enable lcdc */
	control = readl(info->iobase + CSKY_LCD_CONTROL);
	control |= CSKY_LCDCON_LEN;
	writel(control, info->iobase + CSKY_LCD_CONTROL);
	return 0;
}

static irqreturn_t csky_fb_irq(int irq, void *dev_id)
{
	struct csky_fb_info *info = dev_id;
	unsigned long status = readl(info->iobase + CSKY_LCD_INT_STAT);

	/* clear interrupts */
	writel(status, info->iobase + CSKY_LCD_INT_STAT);

	if (status & CSKY_LCDINT_STAT_LDD) {
		printk("LCD_INT: LCD has been disabled\n");
	}

	if (status & CSKY_LCDINT_STAT_BAU) {
		printk("LCD_INT: Base address update\n");
	}

	if (status & CSKY_LCDINT_STAT_BER) {
		printk("LCD_INT: Bus error\n");
	}

	if (status & CSKY_LCDINT_STAT_LFU) {
		printk("LCD_INT: Line FIFO underrun\n");
	}

	return IRQ_HANDLED;
}

static int csky_fb_probe(struct platform_device *pdev)
{
	int irq;
	struct resource *mem;
	void __iomem *iobase;
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

	ret = of_property_read_u32(dev->of_node, "bits-per-pixel",
				   &bits_per_pixel);
	if (ret < 0) {
		printk("error! failed to get property bits-per-pixel\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "pixel-clock-source", &pixel_clk_src);
	if (ret < 0) {
		printk("error! failed to get property pixel-clock-source\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "pcd", &pcd);
	if (ret < 0) {
		printk("error! failed to get property pcd\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "hsync-pulse-pol", &hsync_pulse_pol);
	if (ret < 0) {
		printk("error! failed to get property hsync-pulse-pol\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "vsync-pulse-pol", &vsync_pulse_pol);
	if (ret < 0) {
		printk("error! failed to get property vsync-pulse-pol\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "pixel-clock-pol", &pixel_clock_pol);
	if (ret < 0) {
		printk("error! failed to get property pixel-clock-pol\n");
		return ret;
	}

	ret = of_get_videomode(dev->of_node, &vm, OF_USE_NATIVE_MODE);
	if (ret) {
		printk("error! failed to get videomode from DT\n");
		return ret;
	}

	/* allocate a fb_info */
	fbinfo = framebuffer_alloc(sizeof(struct csky_fb_info), &pdev->dev);
	if (!fbinfo)
		return -ENOMEM;
	platform_set_drvdata(pdev, fbinfo);

	/* initialize the fb_info */

	info = fbinfo->par;
	info->dev = &pdev->dev;
	info->iobase = iobase;
	info->irq = irq;
	memcpy(&info->vm, &vm, sizeof(vm));
	info->pixel_clk_src = pixel_clk_src;
	info->pcd = pcd;
	info->hsync_pulse_pol = hsync_pulse_pol;
	info->vsync_pulse_pol = vsync_pulse_pol;
	info->pixel_clock_pol = pixel_clock_pol;

	strcpy(fbinfo->fix.id, DRIVER_NAME);
	fbinfo->fix.smem_len = vm.hactive * vm.vactive * (bits_per_pixel / 8);
	fbinfo->fix.type = FB_TYPE_PACKED_PIXELS;
	fbinfo->fix.type_aux = 0;
	fbinfo->fix.visual = FB_VISUAL_TRUECOLOR;
	fbinfo->fix.xpanstep = 0;
	fbinfo->fix.ypanstep = 0; /* double buffer not supported */
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
		printk("error! Failed to allocate video memory\n");
		goto RELEASE_FBINFO;
	}

#if 1
{
	u32 i, j;
	i=j=0;
/*
	for (;j<160;j++) // 0xb6e00000
		for (i=0;i<800;i++)
			*(unsigned int *)(fbinfo->screen_base + (j*800+i)*4) = 0x00ff0000;
	for (;j<320;j++) // 0xb6e7d000
		for (i=0;i<800;i++)
			*(unsigned int *)(fbinfo->screen_base + (j*800+i)*4) = 0x0000ff00;
	for (;j<480;j++) // 0xb6efa000
		for (i=0;i<800;i++)
			*(unsigned int *)(fbinfo->screen_base + (j*800+i)*4) = 0x000000ff;
*/
	memset(fbinfo->screen_base, 0, 800*480*4);
	j=0;
	for (i=0;i<800;i++)
		*(unsigned int *)(fbinfo->screen_base + (j*800+i)*4) = 0x00ff0000;
	j=479;
	for (i=0;i<800;i++)
		*(unsigned int *)(fbinfo->screen_base + (j*800+i)*4) = 0x00ff0000;
	i=0;
	for (j=0;j<480;j++)
		*(unsigned int *)(fbinfo->screen_base + (j*800+i)*4) = 0x00ff0000;
	i=799;
	for (j=0;j<480;j++)
		*(unsigned int *)(fbinfo->screen_base + (j*800+i)*4) = 0x00ff0000;
}
#endif
	csky_fb_init_registers(fbinfo);

	csky_fb_check_var(&fbinfo->var, fbinfo);
	ret = register_framebuffer(fbinfo);
	if (ret < 0) {
		printk("error! Failed to register framebuffer device\n");
		goto FREE_VIDEO_MEMORY;
	}

	ret = request_irq(irq, csky_fb_irq, 0, pdev->name, info);
	if (ret) {
		printk("error! cannot get irq %d - err %d\n", irq, ret);
		goto UNREGISTER_FB;
	}

	printk("fb%d: %s frame buffer device\n",
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

	csky_fb_lcd_enable(info, false);

	unregister_framebuffer(fbinfo);

	csky_fb_unmap_video_memory(fbinfo);

	free_irq(info->irq, info);

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

MODULE_DESCRIPTION("C-SKY framebuffer driver");
MODULE_LICENSE("GPL v2");
