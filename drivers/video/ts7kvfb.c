/*
 * linux/drivers/video/ts7kvfb.c
 *
 * Framebuffer device for the TS-7KV board.
 *
 * Based on TS-7300 FB driver By Fotis Loukos <fotisl@csd.auth.gr>
 *
 * Based on skeletonfb.c for new api by James Simmons and the original driver
 * for the 2.4 series by Eddie Dawydiuk.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include "ts7kvfb.h"


/* The width and height below may change. */
static struct fb_fix_screeninfo ts7kvfb_fix __devinitdata = {
    .id             = "TS7KV", 
    .type           = FB_TYPE_PACKED_PIXELS,
    .accel          = FB_ACCEL_NONE,
    .smem_start     = VIDEORAM_BASE,
    .mmio_start     = VIDEOREGS_BASE,
    .mmio_len       = 10,
    .visual         = FB_VISUAL_MONO10,
};

static struct fb_var_screeninfo ts7kvfb_var __devinitdata = {
    /* TS7KV_DEF_WIDTH, TS7KV_DEF_HEIGHT, TS7KV_DEF_WIDTH, TS7KV_DEF_HEIGHT,*/
    0, 0, 0, 0,
    0, 0, TS7KV_BPP, 0,
    { 11, 5, 0 }, { 5, 5, 0 }, { 0, 5, 0 }, { 0, 0, 0 },
    0, FB_ACTIVATE_NOW, -1, -1, FB_ACCEL_NONE,
    31746, 128, 24, 28, 9, 40, 3, 0, FB_VMODE_NONINTERLACED, 0
};

static struct platform_device *ts7kvfb_device;

/*
 * Set a color register. There is no hardware palette so we use the pseudo_palette
 * we allocated. The only format supported is 16bit 555.
 */
static int ts7kvfb_setcolreg(unsigned regno, unsigned red, unsigned green,
			   unsigned blue, unsigned transp,
			   struct fb_info *info)
{
    u16 v;

    if(regno >= PALETTE_COLORS)
       return -EINVAL;

    if(info->var.grayscale)
       red = green = blue = (red * 77 + green * 151 + blue * 28) >> 8;

    /* It's 555 with an unused bit between green and blue. */
    v = ((red & 0xf800) >> 0) | ((green & 0xf800) >> 5) |
        ((blue & 0xf800) >> 11);

    ((u16 *)(info->pseudo_palette))[regno] = v;

    return 0;
}


inline void ts7kv_access_begin(struct ts7kvfb_par *par) {
  par->SWIN_7KV = ioread16(par->fpga_regs + REG_DEVSEL);
  iowrite16((ioread16(par->fpga_regs + REG_DEVSEL) & 0xFF0F) | 0x50,par->fpga_regs + REG_DEVSEL);
}

inline void ts7kv_access_end(struct ts7kvfb_par *par) {
  iowrite16(par->SWIN_7KV,par->fpga_regs + REG_DEVSEL);
}

/*
 * Blank the display. This is supported by the hardware accelarator.
 */
static int ts7kvfb_blank(int blank_mode, struct fb_info *info)
{
    u16 vidctrl, newctrl;
    struct ts7kvfb_par *par = info->par;

    /* DEBUG!!! */
    printk(KERN_DEBUG "ts7kvfb: Using hardware accelerated blank.\n");

    ts7kv_access_begin(par);
    vidctrl = ioread16(par->fpga_regs + VIDCTRL);

    switch(blank_mode) {
    case FB_BLANK_UNBLANK:
    case FB_BLANK_NORMAL:
        newctrl = vidctrl | 0x300;
        break;
    case FB_BLANK_HSYNC_SUSPEND:
        newctrl = (vidctrl | 0x100) & ~0x200;
        break;
    case FB_BLANK_VSYNC_SUSPEND:
        newctrl = (vidctrl | 0x200) & ~0x100;
        break;
    case FB_BLANK_POWERDOWN:
        newctrl = vidctrl & ~0x300;
        break;
    default:
        return !0;
        break;
    }

    iowrite16(newctrl, par->fpga_regs + VIDCTRL);

    /* Blanking is done by filling the whole screen with black color. */
    if(blank_mode != FB_BLANK_UNBLANK) {
        vidctrl = ioread16(par->fpga_regs + VIDCTRL);
        vidctrl = (vidctrl & 0xff7f) | (1 << 6);
        iowrite16(vidctrl, par->fpga_regs + VIDCTRL);
        iowrite16(((par->width - 1) >> 7) << 13, par->fpga_regs + BLTCTRL);
        iowrite16((par->height & 0x1ff) | ((par->width - 1) << 9),
                par->fpga_regs + BLTSZ);
        iowrite16(0, par->fpga_regs + SRCBLT);
        /*
         * Use a write memory barrier to ensure that everything is written
         * before writting to the DSTBLT register. Order is important!
         */
        wmb();
        iowrite16(0, par->fpga_regs + DSTBLT);
	/*
        while(((u16 *) par->fpga_regs)[VIDCTRL] & 0x400)
            cpu_relax();
	*/
    }
    ts7kv_access_end(par);

    return 0;
}

/*
 * Fill a rectangle with a color. Supported by the hardware accelerator.
 */
void ts7kvfb_fillrect(struct fb_info *info, const struct fb_fillrect *rect)
{
    struct ts7kvfb_par *par = info->par;
    u16 vidctrl;
    u32 dst;

    if(info->state != FBINFO_STATE_RUNNING)
        return;

    /*
     * If hardware acceleration is disabled then we use the default unaccelerated
     * function for filling rectangles.
     */
    if(info->flags & FBINFO_HWACCEL_DISABLED) {
        cfb_fillrect(info, rect);
        return;
    }

    /* DEBUG!!! */
    printk(KERN_DEBUG "ts7kvfb: Using hardware accelerated fillrect.\n");

    /* Convert to the address needed by the accelerator. */
    dst = rect->dy * par->width + rect->dx;

    ts7kv_access_begin(par);
    vidctrl = ioread16(par->fpga_regs + VIDCTRL) | (1 << 6);
    iowrite16(vidctrl & 0xff7f, par->fpga_regs + VIDCTRL);
    iowrite16(((dst >> 16) << 6) | (((rect->width - 1) >> 7) << 13),
            par->fpga_regs + BLTCTRL);
    iowrite16((rect->height & 0x1ff) | ((rect->width - 1) << 9),
            par->fpga_regs + BLTSZ);
    iowrite16((u16) rect->color & 0xffff, par->fpga_regs + SRCBLT);
    /*
     * Use a write memory barrier to ensure that everything is written
     * before writting to the DSTBLT register. Order is important!
     */
    wmb();
    iowrite16(dst & 0xffff, par->fpga_regs + DSTBLT);

    /* Since we implemented the sync function this is not needed, right? */
    /*
    while(ioread16(par->fpga_regs + VIDCTRL) & 0x400)
        cpu_relax();
     */
    ts7kv_access_end(par);
}

/*
 * Copy a rectangular area for one place to another. Supported by the hardware
 * accelerator.
 */
void ts7kvfb_copyarea(struct fb_info *info, const struct fb_copyarea *area) 
{
    struct ts7kvfb_par *par = info->par;
    u16 vidctrl, direction;
    u32 src, dst;

    if(info->state != FBINFO_STATE_RUNNING)
        return;

    /*
     * If hardware acceleration is disabled then we use the default unaccelerated
     * function for copying areas.
     */
    if(info->flags & FBINFO_HWACCEL_DISABLED) {
        cfb_copyarea(info, area);
        return;
    }

    /* DEBUG!!! */
    printk(KERN_DEBUG "ts7kvfb: Using hardware accelerated copyarea.\n");

    /* Convert to the address needed by the accelerator. */
    src = area->sy * par->width + area->sx;
    dst = area->dy * par->width + area->dx;

    /*
     * Swap and change direction so overlapping source and destination areas don't
     * mess up while drawing.
     */
    if(src < dst) {
        src += (area->height - 1) * (par->width - 1);
        dst += (area->height - 1) * (par->width - 1);
        direction = 1 << 7;
    } else {
        direction = 0;
    }
    ts7kv_access_begin(par);
    vidctrl = ioread16(par->fpga_regs + VIDCTRL);
    iowrite16((vidctrl & 0xff3f) | direction, par->fpga_regs + VIDCTRL);
    iowrite16((src >> 16) | ((dst >> 16) << 6) | (((area->width - 1) >> 7) << 13),
            par->fpga_regs + BLTCTRL);
    iowrite16((area->height & 0x1ff) | ((area->width - 1) << 9),
            par->fpga_regs + BLTSZ);
    iowrite16(src & 0xffff, par->fpga_regs + SRCBLT);
    /*
     * Use a write memory barrier to ensure that everything is written
     * before writting to the DSTBLT register. Order is important!
     */
    wmb();
    iowrite16(dst & 0xffff, par->fpga_regs + DSTBLT);

    /* Since we implemented the sync function this is not needed, right? */
    /*
    while(ioread16(par->fpga_regs + VIDCTRL) & 0x400)
        cpu_relax();
     */
    ts7kv_access_end(par);
}


/*
 * Wait till our blitting operations have finished.
 */
int ts7kvfb_sync(struct fb_info *info)
{
    struct ts7kvfb_par *par = info->par;

    ts7kv_access_begin(par);
    while(ioread16(par->fpga_regs + VIDCTRL) & 0x400)
        cpu_relax();
    ts7kv_access_end(par);

    return 0;
}

/*
 * Frame buffer operations.
 * We use our own fillrect and copyarea operations but the default imageblit since
 * it's not supported by the hardware accelerator.
 */

static struct fb_ops ts7kvfb_ops = {
	.owner          = THIS_MODULE,
	.fb_setcolreg   = ts7kvfb_setcolreg,
	.fb_blank       = ts7kvfb_blank,
	.fb_fillrect    = ts7kvfb_fillrect,
	.fb_copyarea    = ts7kvfb_copyarea,
	.fb_imageblit   = cfb_imageblit,
	.fb_sync        = ts7kvfb_sync,
};

/*
 * Find the mode of the framebuffer, either 640x480 or 800x600. In order to do this
 * we fill the first two lines with color 0xFF and draw a rectangle 2 pixels tall
 * by 1 pixel wide of color 0x00. Then search the position of the second pixel
 * with color 0x00. The first one is the first pixel and the second one either
 * pixel 640 or 800.
 */
static int ts7kvfb_detectmode(struct ts7kvfb_par *par)
{
    int pos;

    ts7kv_access_begin(par);
    iowrite16(0x40, par->fpga_regs + VIDCTRL);
    iowrite16(((TS7KV_MAXWIDTH - 1) >> 7) << 13, par->fpga_regs + BLTCTRL);
    iowrite16(2 | ((TS7KV_MAXWIDTH - 1) << 9), par->fpga_regs + BLTSZ);
    iowrite16(0xFFFF, par->fpga_regs + SRCBLT);
    iowrite16(0, par->fpga_regs + DSTBLT);

    while(ioread16(par->fpga_regs + VIDCTRL) & 0x400)
        cpu_relax();

    iowrite16(0x40, par->fpga_regs + VIDCTRL);
    iowrite16(0, par->fpga_regs + BLTCTRL);
    iowrite16(2, par->fpga_regs + BLTSZ);
    iowrite16(0, par->fpga_regs + SRCBLT);
    /*
     * Use a write memory barrier to ensure that everything is written
     * before writting to the DSTBLT register. Order is important!
     */
    wmb();
    iowrite16(0, par->fpga_regs + DSTBLT);

    while(ioread16(par->fpga_regs + VIDCTRL) & 0x400)
        cpu_relax();
    ts7kv_access_end(par);

    for(pos = 2; pos < 4 * TS7KV_MAXWIDTH; pos++)
        if(ioread8(par->video_mem + pos) == 0x0)
            break;

    if(pos == 4 * TS7KV_MAXWIDTH) {
        printk(KERN_ERR "ts7kvfb: Could not detect screen resolution!\n");
        return -1;
    }

    if((pos != 1280) && (pos != 1600)) {
        printk(KERN_ERR "ts7kvfb: Unsupported screen resolution! Only 640x480 "
                "and 800x600 are supported.\n");
        return -1;
    }

    /* It's a fixed aspect ratio. */
    par->width = pos / 2;
    par->height = par->width * 3/4;
    printk(KERN_INFO "ts7kvfb: Detected mode %ix%i.\n", par->width, par->height);

    return 0;
}

static int ts7kvfb_detect_board(void **vid_regs,void **vid_mem) {
  int i;
  unsigned int base_addr;
  unsigned char *base;
  
  for (i=0;i<4;i++) {
    base_addr = IO8_BASE + REGS_CTRL_BASE + REGS_CTRL_SIZE * i;
    base = ioremap(base_addr,REGS_CTRL_SIZE); // doesn't have to be 4k page aligned???
    if (!base) {
      printk(KERN_ERR "ts7kvfb: cannot get control register I/O port!\n");
      return 0;
    }
    if (base[REG_ID0] == TS7KV_ID0 && base[REG_ID1] == TS7KV_ID1
	&& (base[REG_CTRL0] & DONE_MASK) == DONE_MASK) {


    /* Request memory regions. */
    if(request_region(VIDEORAM_BASE, VIDEORAM_SIZE, "ts7kvfb") == NULL) {
        printk(KERN_ERR "ts7kvfb: Cannot get I/O ports!\n");
	return 0;
    }

    if(request_region(VIDEOREGS_BASE, VIDEOREGS_SIZE, "ts7kvfb") == NULL) {
        printk(KERN_ERR "ts7kvfb: Cannot get I/O ports!\n");
	return 0;
    }



      *vid_regs = ioremap(IO16_BASE + REGS_VID_BASE + REGS_VID_SIZE * i,REGS_VID_SIZE);
      if (*vid_regs == 0) {
	printk(KERN_ERR "ts7kvfb: could not get video register I/O ports!\n");
	return 0;
      }
      *vid_mem  = ioremap(MEM16_BASE + MEM_VID_BASE + MEM_VID_SIZE * i,MEM_VID_SIZE);
      if (*vid_mem == 0) {
	printk(KERN_ERR "ts7kvfb: could not get video memory!\n");
	return 0;
      }
      return 1;
    }
  }
  return 0;
}

static void off104(void);

/*
 * Initialization stuff.
 */
static int __devinit ts7kvfb_probe(struct platform_device *device)
{
    int retval = -EIO;
    struct fb_info *info;
    struct ts7kvfb_par *par;
   
    /* Dynamically allocate info and par. */
    info = framebuffer_alloc(sizeof(*par), &device->dev);

    if(!info) {
        printk(KERN_ERR "ts7kvfb: Cannot allocate memory for framebuffer!\n");
        retval = -ENOMEM;
        goto err0;
    }

    par = info->par;

    if (!ts7kvfb_detect_board(&par->fpga_regs,&par->video_mem)) {
      printk("TS-7KV not detected\n");
      retval = -ENODEV;
      goto err0;
    }

    /*
     * We find the width and height here because we need the framebuffer
     * memory and the fpga registers and we need to do this before
     * registering the framebuffer.
     */
    if(ts7kvfb_detectmode(par) < 0) {
        retval = -EINVAL;
        goto err4;
    }

    /* Set width and height info. */
    ts7kvfb_fix.line_length = par->width * (TS7KV_BPP >> 3);
    ts7kvfb_fix.smem_len = par->width * par->height * (TS7KV_BPP >> 3);
    ts7kvfb_var.xres = par->width;
    ts7kvfb_var.xres_virtual = par->width;
    ts7kvfb_var.yres = par->height;
    ts7kvfb_var.yres_virtual = par->height;

    /* Allocate palette. */
    info->pseudo_palette = kmalloc(PALETTE_COLORS * sizeof(u16), GFP_KERNEL);
    if(!info->pseudo_palette) {
        printk(KERN_ERR "ts7kvfb: Cannot allocate memory for palette!\n");
        retval = -ENOMEM;
        goto err5;
    }

    /*
     * We set the screen_base to the virtual memory address for the framebuffer.
     * Resource address and screen_size is standard.
     */
    info->screen_base = par->video_mem;
    info->screen_size = VIDEORAM_SIZE;
    info->fbops = &ts7kvfb_ops;
    info->fix = ts7kvfb_fix;
    info->flags = FBINFO_DEFAULT | FBINFO_HWACCEL_COPYAREA |
        FBINFO_HWACCEL_FILLRECT;

    /* Allocate colormap. */
    if(fb_alloc_cmap(&info->cmap, CMAP_LEN, 0) < 0) {
        printk(KERN_ERR "ts7kvfb: Cannot allocate colormap!\n");
        retval = -ENOMEM;
        goto err6;
    }

    info->var = ts7kvfb_var;

    /* We're done so register the framebuffer. */
    if(register_framebuffer(info) < 0) {
        printk(KERN_ERR "ts7kvfb: Cannot register framebuffer!\n");
        retval = -EINVAL;
        goto err7;
    }

    /* Blank the screen. */
    ts7kv_access_begin(par);
    iowrite16(0x340, par->fpga_regs + VIDCTRL);
    iowrite16((par->width >> 7) << 13, par->fpga_regs + BLTCTRL);
    iowrite16((par->height & 0x1ff) | (par->width << 9),
        par->fpga_regs + BLTSZ);
    iowrite16(0, par->fpga_regs + SRCBLT);
    /*
     * Use a write memory barrier to ensure that everything is written
     * before writting to the DSTBLT register. Order is important!
     */
    wmb();
    iowrite16(0, par->fpga_regs + DSTBLT);
    ts7kv_access_end(par);

    printk(KERN_INFO "ts7kvfb: Framebuffer device loaded and initialized.\n");
    platform_set_drvdata(device, info);

    return 0;

err7:
    fb_dealloc_cmap(&info->cmap);
err6:
    kfree(info->pseudo_palette);
err5:
    iounmap(par->fpga_regs);
err4:
    iounmap(par->video_mem);
    /*
err3:
    release_region(VIDEOREGS_BASE, VIDEOREGS_SIZE);
err2:
    release_region(VIDEORAM_BASE, VIDEORAM_SIZE);
err1:
    framebuffer_release(info);
    */
err0:
    off104();
    return retval;
}

/*
 *  Cleanup
 */
static int ts7kvfb_remove(struct platform_device *device)
{
    struct fb_info *info = platform_get_drvdata(device);

    if(info) {
        struct ts7kvfb_par *par = info->par;
        unregister_framebuffer(info);
        fb_dealloc_cmap(&info->cmap);
        kfree(info->pseudo_palette);
        iounmap(par->fpga_regs);
        iounmap(par->video_mem);
        release_region(VIDEOREGS_BASE, VIDEOREGS_SIZE);
        release_region(VIDEORAM_BASE, VIDEORAM_SIZE);
        framebuffer_release(info);
    }

    return 0;
}

static struct platform_driver ts7kvfb_driver = {
	.probe      = ts7kvfb_probe,
	.remove     = ts7kvfb_remove,
    .driver     = {
        .name   = "ts7kvfb",
    }
};

static unsigned long saved_A=0, saved_B=0;

static void on104(void) {
  volatile unsigned long *tmp;
  tmp = ((unsigned long *) __arm_ioremap(0xE8000000, 4096,0));
  if (tmp == (unsigned long *)0xE8000000) {
    printk(KERN_ERR "tsuart1: cannot map 0xE8000000\n");
    return;
  }
  saved_A = tmp[0x30/4];
  saved_B = tmp[0x34/4];
  tmp[0x30/4] = 0x55555555;
  tmp[0x34/4] = 0x55555555;
  __iounmap(tmp);
}

static void off104(void) {
  volatile unsigned long *tmp;
  tmp = ((unsigned long *) __arm_ioremap(0xE8000000, 4096,0));
  if (tmp == (unsigned long *)0xE8000000) {
    printk(KERN_ERR "tsuart1: cannot map 0xE8000000\n");
    return;
  }
  tmp[0x30/4] = saved_A;
  tmp[0x34/4] = saved_B;
  __iounmap(tmp);
}

/*
 * Module loading.
 */
static int __devinit ts7kvfb_init(void)
{
    int ret = 0;

    printk(KERN_INFO "ts7kvfb: Loading framebuffer device.\n");
    on104();

    ret = platform_driver_register(&ts7kvfb_driver);

    if(!ret) {
        ts7kvfb_device = platform_device_alloc("ts7kvfb", 0);

        if(ts7kvfb_device)
            ret = platform_device_add(ts7kvfb_device);
        else
            ret = -ENOMEM;

        if(ret) {
            platform_device_put(ts7kvfb_device);
            platform_driver_unregister(&ts7kvfb_driver);
        }
    }

    if (ret) {
      off104();
    }
    return ret;
}

/*
 * Module unloading.
 */
static void __devexit ts7kvfb_exit(void)
{
    platform_device_unregister(ts7kvfb_device);
    platform_driver_unregister(&ts7kvfb_driver);
    printk(KERN_INFO "ts7kvfb: Framebuffer device unloaded.\n");
}


/*
 * Module stuff.
 */

module_init(ts7kvfb_init);
module_exit(ts7kvfb_exit);

MODULE_AUTHOR("Fotis Loukos <fotisl@csd.auth.gr>");
MODULE_DESCRIPTION("TS-7KV framebuffer driver");
MODULE_LICENSE("GPL");
