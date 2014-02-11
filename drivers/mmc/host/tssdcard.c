#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/kernel.h>	/* printk() */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>		/* everything... */
#include <linux/errno.h>	/* error codes */
#include <linux/timer.h>
#include <linux/types.h>	/* size_t */
#include <linux/fcntl.h>	/* O_ACCMODE */
#include <linux/hdreg.h>	/* HDIO_GETGEO */
#include <linux/kdev_t.h>
#include <linux/vmalloc.h>
#include <linux/genhd.h>
#include <linux/blkdev.h>
#include <linux/buffer_head.h>	/* invalidate_bdev */
#include <linux/bio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <asm/cacheflush.h>
#include <linux/suspend.h>
#include <linux/kthread.h>
#include <asm/atomic.h>
#include <linux/semaphore.h>
#include <asm/current.h>
//#include <asm/arch-ep93xx/ts72xx.h>

// Ian added
#include <linux/proc_fs.h>
#include <linux/kallsyms.h>

typedef struct request_queue request_queue_t;   // This was removed sometime between 2.6.21 and 2.6.32


#define my_inb(x) readb(x)
#define my_outb(x,y) writeb(x,y)
#define my_inw(x) readw(x)
#define my_outw(x,y) writew(x,y)
#define my_inl(x) readl(x)
#define my_outl(x,y) writel(x,y)

#define SDDAT           1

extern void __udelay(unsigned int);
#define udelay(x) __udelay((x))
#define INVALIDATE_DELAY	30*HZ	/* After this much idle time, the driver will simulate a media change. */
#define KERNEL_SECTOR_SIZE	512	/* The kernel talks to us in terms of small sectors, always. */
#define SD_DEFNAME		"tssdcard"
#define SD_DEFIOSIZE		4
#define SD_BLKSIZE		1024	//nsectors?
#define SD_HARDSEC		512
#define SD_SHIFT		4	//max 16 partitions 2^4
#define SD_NUMPART		1 << 4
#define TSSDCARD_MINORS		SD_NUMPART
#define MINOR_SHIFT		SD_SHIFT
#define DEVNUM(kdevnum)		(MINOR(kdev_t_to_nr(kdevnum)) >> MINOR_SHIFT

enum {
  RM_SIMPLE  = 0,  /* The extra-simple request function */
  RM_FULL    = 1,  /* The full-blown version */
  RM_NOQUEUE = 2,  /* Use make_request */
};


#ifdef CONFIG_MACH_TS72XX
#ifdef CONFIG_MACH_TS7350
#include "tssdcore2.c"
#else
#include "tssdcore.c"
#endif

#define THREAD_ENABLE		1
#define DMA_CHANNEL             1
#define REQUEST_MODE		RM_FULL
#define DEBUG_ENABLE		0
#define NDEVICES                1

#define DMA_BUFSIZE		1024
#define DMA_M2M_0_SIZE		0x40
#define DMA_M2M_1_SIZE		0x40
#ifdef CONFIG_MACH_TS7350
#define TS7XXX_SDCARD1		0x600FF000
#define SYSCONFPGA_BASE         0x600FF080
#define DMA_ENABLE              1
#else
#define TS7XXX_SDCARD1		0x13000000
#define DMA_ENABLE              0
#endif
#define TS7XXX_SDCARD2		0x72000020

#ifndef SYSCON_OFFSET
#define SYSCON_OFFSET 		0x130000
#define SYSCON_BASE             (0x80800000|SYSCON_OFFSET)
#define SYSCON_SCRREG0 		0x0040
#define SYSCON_SCRREG1 		0x0044
#define SYSCON_SWLOCK 		0x00C0
#define SYSCON_PWRCNT 		0x0004
#define SYSCON_DEVCFG 		0x0080
#define SYSCON_PWRCNT_DMA_M2MCH0 0x04000000
#define SYSCON_PWRCNT_DMA_M2MCH1 0x08000000
#define SYSCON_DEVCFG_D0onG 	0x20000000
#define SYSCON_DEVCFG_D1onG 	0x40000000
#endif
#ifndef DMA_OFFSET
#define DMA_OFFSET 		0x000000
#define DMA_BASE 		(0x80000000|DMA_OFFSET)
#define DMAMM_0_CONTROL 	(DMA_BASE+0x0100)
#define DMAMM_1_CONTROL 	(DMA_BASE+0x0140)
#define DMA_M2M_0_BASE 		DMAMM_0_CONTROL
#define DMA_M2M_1_BASE 		DMAMM_1_CONTROL
#define M2M_OFFSET_CONTROL 	0x0000
#define M2M_OFFSET_INTERRUPT 	0x0004
#define M2M_OFFSET_STATUS 	0x000C
#define M2M_OFFSET_BCR0	 	0x0010
#define M2M_OFFSET_BCR1 	0x0014
#define M2M_OFFSET_SAR_BASE0 	0x0018
#define M2M_OFFSET_SAR_BASE1 	0x001C
#define M2M_OFFSET_SAR_CURRENT0 0x0024
#define M2M_OFFSET_SAR_CURRENT1 0x0028
#define M2M_OFFSET_DAR_BASE0 	0x002C
#define M2M_OFFSET_DAR_BASE1 	0x0030
#define M2M_OFFSET_DAR_CURRENT0 0x0034
#define M2M_OFFSET_DAR_CURRENT1 0x003C
#endif

#endif


#ifdef CONFIG_MACH_TS78XX
#include "tssdcore2.c"

#define DMA_ENABLE 		1
#define THREAD_ENABLE		1
#define DMA_CHANNEL		0
#define REQUEST_MODE		RM_NOQUEUE
#define DEBUG_ENABLE		0
#define NDEVICES                2               /* TS-7800 ndevices=2 for Micro and Full size sockets */


#define TS7800_IRQ_DMA		65
#define TS7800_IRQ_WAIT		66
#define TS7XXX_SDCARD1		0xE8000100
#define TS7800_DMAADDR		0xE8000400
#define TS7800_FPGASDDAT	0x80000104
#define TS7800_CPUDOORBELL	   (ORION5X_REGS_VIRT_BASE + 0x20400) /*0xF1020400*/
#define TS7800_FPGADOORBELL	0xE8000200
#define TS7800_FPGAMSIMASK	   0xE8000204
#define TS7800_FPGAIRQRAW	   0xE8000208
#define DOORBELL_DMABIT		(0x1 << 1)
#define DOORBELL_WAITBIT	(0x1 << 2)


extern int dma_op (unsigned long dev_addr, unsigned long dat, unsigned int len, short rw);
extern void __attribute__((naked)) xdma_clean_range(char *start, char *end);
extern void __attribute__((naked)) xdma_inv_range(char * start, char * end);

#endif

static char *name = SD_DEFNAME;
static unsigned int io = TS7XXX_SDCARD1;
static int dmaenable = DMA_ENABLE;
static int threnable = THREAD_ENABLE;
static int dmach = DMA_CHANNEL;
static int reqmode = REQUEST_MODE;
static int dbgenable = DEBUG_ENABLE;
static int ndevices = NDEVICES;
static int tssdcard_major = 0;
//static int nsectors = SD_BLKSIZE;
//static int bqt7800 = 0;

module_param(name, charp, 0);
module_param(io, int, 0);
module_param(dmaenable, int, 0);
module_param(threnable, int, 0);
module_param(dmach, int, 0);
module_param(reqmode, int, 0);
module_param(dbgenable, int, 0);
//module_param(bqt7800, int, 0);

struct tssdcard_dev {
  struct sdcore tssdcore;
  char *devname;
  int size;			/* Device size in sectors */
  struct gendisk *gd;		/* The gendisk structure */
  //struct timer_list timer;
  short media_change;		/* Flag a media change? */
  int irq, irq_wait;
  unsigned long dmaregs;
  unsigned long sysconfpga;
  int dmach;
  atomic_t users;		/* How many users */
  int parksect;
  struct request_queue *queue;	/* The device request queue */
  struct bio *bio;
  struct bio *biotail;
  struct task_struct *thread;
  wait_queue_head_t event;
  spinlock_t lock;

  // Ian added
  int write_activity;

};

extern unsigned char mvMacAddr [6];

static wait_queue_head_t qwait;
static struct semaphore sem;
static atomic_t busy;
static struct tssdcard_dev *devices = NULL;
static struct block_device_operations tssdcard_ops;

#define DMA_DECLBASE(x)	unsigned long dmaregs = (x);
#ifdef CONFIG_MACH_TS72XX
#define DMA_WRITE(x, y)	*(volatile uint32_t *)(dmaregs + M2M_OFFSET_ ## x) = (y)
#define DMA_READ(x) *(volatile uint32_t *)(dmaregs + M2M_OFFSET_ ## x)
#endif
#ifdef CONFIG_MACH_TS78XX
#endif

#ifdef CONFIG_MACH_TS72XX
static void __attribute__((naked))
xdma_clean_range(char *start, char *end)
{
        asm volatile (
        "bic     r0, r0, #31                                            \n\
1:      mcr     p15, 0, r0, c7, c10, 1          @ clean D entry         \n\
        add     r0, r0, #32                                             \n\
        cmp     r0, r1                                                  \n\
        blo     1b                                                      \n\
        mcr     p15, 0, r0, c7, c10, 4          @ drain WB              \n\
        mov     pc, lr"
        ::);
}

static void __attribute__((naked))
xdma_inv_range(char * start, char * end)
{
        asm volatile (
        "tst     r0, #31                                                \n\
        mcrne   p15, 0, r0, c7, c10, 1          @ clean D entry         \n\
        tst     r1, #31                                                 \n\
        mcrne   p15, 0, r1, c7, c10, 1          @ clean D entry         \n\
        bic     r0, r0, #31                                             \n\
1:      mcr     p15, 0, r0, c7, c6, 1           @ invalidate D entry    \n\
        add     r0, r0, #32                                             \n\
        cmp     r0, r1                                                  \n\
        blo     1b                                                      \n\
        mcr     p15, 0, r0, c7, c10, 4          @ drain WB              \n\
        mov     pc, lr"
        ::);
}
#endif


static irqreturn_t int_handler(int irq, void *vidp, struct pt_regs regs)
{
  struct tssdcard_dev *dev = (struct tssdcard_dev *)vidp;
  BUG_ON(irq != dev->irq_wait);
  disable_irq_nosync(irq);
  wake_up_interruptible(&qwait);
  return IRQ_HANDLED;
}

pte_t * vmalloc_to_pte(void * vmalloc_addr)
{
  static volatile struct mm_struct *p_init_mm = NULL;
  unsigned long addr = (unsigned long) vmalloc_addr;
  //volatile int *v = *(int *)vmalloc_addr;
  pmd_t *pmd;
  pte_t *pte = NULL;
  pgd_t *pgd;

  if (p_init_mm == NULL)
     p_init_mm = (struct mm_struct *)kallsyms_lookup_name("init_mm");

  if (p_init_mm == NULL)
   panic("Symbol init_mm not found in kernel\n");

  //pgd = pgd_offset_k(addr);    <<<< init_mm no longer exported by kernel
  pgd = pgd_offset(p_init_mm, addr);

  if (!pgd_none(*pgd)) {
    pmd = pmd_offset(pgd, addr);
    if (!pmd_none(*pmd)) {
      pte = pte_offset_kernel(pmd, addr);
    }
  }
  return pte;
}

#if (0)
static int tssdcard_prosoft_powerok(void *os_arg)
{
	if(!prosoft_powerok()) {
		printk("Power fail detected, write protecting SD card\n");
		return 0;
	}
}
#endif

/*
static int tssdcard_powerok(void *os_arg)
{
  int ret=1;
  unsigned long regval;
  //printk("powerok\n");
  if (bqt7800) {
    regval = readl(0xE800003C); //PC104 config
    if ( (regval&(0x3<<16)) != 0 )
      writel((regval&~(0x3<<16)), 0xE800003C);
    regval = readl(0xE800002C); //GPIO direction
    if ( (regval&(0x1<<17)) != 0 )
     writel(regval&~(0x1<<17), 0xE800002C);
    ret = ( (readl(0xE800001C)&(0x1<<17)) != 0 ); //GPIO data
    //printk("ret %d\n", ret);
  }
  return ret;
}
*/

#ifdef TS_PVT_SOLAR
static int tssdcard_powerok(void *os_arg)
{ // PVT solar
  int ret=1;
  unsigned long regval;
  struct tssdcard_dev *dev = (struct tssdcard_dev *)os_arg;

  regval = readl(dev->sysconfpga+0x18); //PC104 config
  if ( (regval&(0x3<<22)) != 0 )
    writel((regval&~(0x3<<22)), dev->sysconfpga+0x18);
  regval = readl(dev->sysconfpga+0x10); //GPIO direction
  if ( (regval&(0x1<<22)) != 0 )
    writel(regval&~(0x1<<22), dev->sysconfpga+0x10);
  ret = ( (readl(dev->sysconfpga+0x8)&(0x1<<22)) != 0 ); //GPIO data
  //printk("tssdcard_powerok=%d (%X)\n", ret,readl(dev->sysconfpga+0x8));
  return ret;
}
#endif

static void tssdcard_irqwait(void *os_arg, unsigned int debug)
{
  struct tssdcard_dev *dev = (struct tssdcard_dev *)os_arg;
  int err;

#ifdef CONFIG_MACH_TS7350
  int i=0;
  while (readl(dev->sysconfpga+0x6)&0x100) {
    i++;
    //if (i>100) schedule();
  }
#endif

#ifdef CONFIG_MACH_TS78XX
  DEFINE_WAIT(wait);
  prepare_to_wait(&qwait, &wait, TASK_INTERRUPTIBLE);
  err = request_irq(dev->irq_wait, (void*)int_handler, IRQF_DISABLED, dev->devname, dev);

  if (err)
  {
      printk("Error in tssdcard_irqwait(), call to request_irq() failed with errno %d\n", err);
      printk("irq parameter: %d\n", dev->irq_wait);
  }
  else
  {
     if (!(readl(TS7800_CPUDOORBELL)&DOORBELL_WAITBIT))
        schedule();

     free_irq(dev->irq_wait, dev);
   }

  finish_wait(&qwait, &wait);
#endif

}

static int tssdcard_dmastream(void *os_arg, unsigned char *dat, unsigned int buflen)
{
  struct tssdcard_dev *dev = (struct tssdcard_dev *)os_arg;
  static unsigned char dummymem[8];
  static unsigned long dummymem_paddr = 0;

#ifdef CONFIG_MACH_TS72XX
  unsigned long dmaregs = dev->dmaregs;

#ifdef CONFIG_MACH_TS7350

  if (dat) {
    while (buflen > (0xfff * 2)) {
      printk("dmastream %d\n", buflen);
      tssdcard_dmastream(os_arg, dat, (0xfff * 2));
      buflen -= (0xfff * 2);
      dat += (0xfff * 2);
    }
  }

  if (dev->tssdcore.sd_state & SDDAT_TX)
    while(dma_op (0x4, (int)dat, buflen, SD_WRITE) < 0) schedule();
  else if (dev->tssdcore.sd_state & SDDAT_RX)
    while(dma_op (0x4, (int)dat, buflen, SD_READ) < 0) schedule();

#else
  DMA_WRITE(CONTROL, 0x11085000);
  DMA_WRITE(SAR_BASE0, io|SDDAT );

  if (dat) {
    DMA_WRITE(DAR_BASE0, virt_to_bus(dat));
  }
  else {
    if (dummymem_paddr == 0) {
      pte_t *pte = vmalloc_to_pte(dummymem);
      dummymem_paddr = pte_pfn(*pte) << PAGE_SHIFT;
      dummymem_paddr += (unsigned long)dummymem & 0xfff;
    }
    DMA_WRITE(DAR_BASE0, dummymem_paddr);
  }

  DMA_WRITE(BCR0, buflen);
  DMA_WRITE(CONTROL, 0x11085008);
  while (!(DMA_READ(STATUS) & 0x40)) ;
  DMA_WRITE(INTERRUPT, 0x0);
#endif
#endif

#ifdef CONFIG_MACH_TS78XX
  unsigned long bufadr, doaddr;
  short rw = READ;

  if (dat) {
    while (buflen > (0xfff * 4)) {
      tssdcard_dmastream(os_arg, dat, (0xfff * 4));
      buflen -= (0xfff * 4);
      dat += (0xfff * 4);
    }
    if ((unsigned long)dat >= VMALLOC_START)
      bufadr = (vmalloc_to_pfn(dat) << PAGE_SHIFT) | ((unsigned int)dat & 0xfff);
    else
      bufadr = virt_to_phys((char *)dat);
  }
  else {
    if (dummymem_paddr == 0) {
      pte_t *pte = vmalloc_to_pte(dummymem);
      dummymem_paddr = pte_pfn(*pte) << PAGE_SHIFT;
      dummymem_paddr += (unsigned long)dummymem & 0xfff;
    }
  }
  if (dat) doaddr = bufadr;
  else doaddr = dummymem_paddr;

  if (dev->tssdcore.sd_state & SDDAT_TX) {
    rw = WRITE;
    if (dat) xdma_clean_range((char *)dat, (char *)dat + buflen);
  }
  else if (dev->tssdcore.sd_state & SDDAT_RX) {
    rw = READ;
    if (dat) xdma_inv_range(dat, dat + buflen);
  }
  while(dma_op (TS7800_FPGASDDAT, doaddr, buflen, rw) < 0) schedule();
#endif

  return 0;
}

static void /*int*/ tssdcard_dmaprep(void *os_arg, unsigned char *buf, unsigned int buflen)
{
 if(dbgenable)printk("dmaprep\n");
#ifdef CONFIG_MACH_TS72XX
#ifdef CONFIG_MACH_TS7350
#else
  //invalidate_dcache_range(buf, buf + buflen);
  //dmac_inv_range(buf, buf + buflen);
  //dmac_clean_range(buf, buf + buflen);
  dmac_flush_range(buf, buf + buflen);
#endif
#endif

#ifdef CONFIG_MACH_TS78XX
  xdma_inv_range(buf, buf + buflen);
#endif

  //return 0;
}

static void tssdcard_delay(void *arg, unsigned int us)
{
  udelay(us);
}



/*
 * Handle an I/O request.
 */
static inline void tssdcard_transfer(struct tssdcard_dev *dev, unsigned long sector,
    unsigned long nsect, char *buffer, int rw)
{
  int ret;

  if(dbgenable)printk("size:%d sector:%ld amount:%ld rw:%d\n", dev->size, sector, nsect, rw);

  if ( (sector + nsect) > (dev->size*512/KERNEL_SECTOR_SIZE) ) {
    printk (KERN_NOTICE "Beyond-end write (%ld %ld)\n", sector, nsect);
    return;
  }

  //del_timer(&dev->timer);
  switch (rw) {
  case WRITE:

    // Ian added
    dev->write_activity = 1;

    ret = sdwrite(&dev->tssdcore, sector, buffer, nsect);
    if (ret && !dev->tssdcore.sd_wprot) {
      sdreset(&dev->tssdcore);
      ret = sdwrite(&dev->tssdcore, sector, buffer, nsect);
    }
    //if (ret == 0) mark_buffer_uptodate(buffer, 1);
    //dev->timer.expires = jiffies + (HZ / 2);
    //add_timer(&dev->timer);
    break;

  case READ:
  case READA:
    ret = sdread(&dev->tssdcore, sector, buffer, nsect);
    if (ret) {
      sdreset(&dev->tssdcore);
      ret = sdread(&dev->tssdcore, sector, buffer, nsect);
    }
    dev->parksect = sector + nsect - 1;

    //sdread will park at sector+nsect+1, if we read the last sector
    //this means we are parked on an invalid sector so reset the card.
    if ( (sector + nsect) >= (dev->size*512/KERNEL_SECTOR_SIZE) )
      sdreset(&dev->tssdcore);

    break;
  }

}


static inline void tssdcard_handle_simplereq(struct tssdcard_dev *dev)
{
  struct request *req;

  while ((req = blk_peek_request(dev->queue)) != NULL) {
    if (! blk_fs_request(req)) {
      printk (KERN_NOTICE "Skip non-fs request\n");
      blk_end_request_cur(req, 0);
      continue;
    }
    tssdcard_transfer(dev, blk_rq_pos(req), blk_rq_cur_sectors(req), req->buffer, rq_data_dir(req));

    /*
    if (!end_that_request_first(req, 1, blk_rq_cur_sectors(req))) {
      blk_start_request(req);
      end_that_request_last(req, 1);
    }
    */

    __blk_end_request (req, 1, blk_rq_cur_sectors(req) << 9);

  }
}

static inline void tssdcard_handle_fullreq(struct tssdcard_dev *dev)
{
  struct request *req;
  struct bio *bio;
  struct bio_vec *bvec;
  int sectors_xferred;
  sector_t sector, end_sector, n_sectors;
  char *buffer;
  struct req_iterator iter;

  end_sector = 0;

  spin_lock(dev->queue->queue_lock);

  while ((req = blk_peek_request(dev->queue))) {
     if (! blk_fs_request(req)) {
        printk (KERN_NOTICE "Skip non-fs request\n");
        blk_end_request_cur(req, 0);
        continue;
     }

     bio = NULL;
     sector = 0;

     sectors_xferred = 0;

     rq_for_each_segment(bvec, req, iter) {

     if (bio != iter.bio)   {  // next bio
         bio = iter.bio;
         sector = bio->bi_sector;
         end_sector = (bio->bi_sector) + (bio->bi_size >> 9) - 1;

         if((bio->bi_size % 512) != 0)
            panic("Invalid transfer, bi_size 512 != 0\n");
      }

      if((sector + (bvec->bv_len >> 9)) > end_sector)
         n_sectors = end_sector - sector + 1;
      else
         n_sectors = bvec->bv_len >> 9;

      if(n_sectors == 0)
         continue;

      buffer = kmap(bvec->bv_page) + bvec->bv_offset;

      spin_unlock(dev->queue->queue_lock);

      if (buffer) {
         tssdcard_transfer(dev, sector, n_sectors, buffer, bio_data_dir(iter.bio));
         sector += n_sectors;
         kunmap(bvec->bv_page);
      }
      else
         printk("Error in tssdcard_handle_fullreq(), kmap() returned NULL\n");

      spin_lock(dev->queue->queue_lock);

    }

    blk_start_request(req);
    blk_end_request_all(req, 0);

  }

  spin_unlock(dev->queue->queue_lock);

}

static inline void tssdcard_add_bio(struct tssdcard_dev *dev, struct bio *bio)
{
  //printk("%s, 0x%08lX, 0x%08lX\n", __func__, dev, bio);

  spin_lock(&dev->lock);
  if (dev->biotail) {
    dev->biotail->bi_next = bio;
    dev->biotail = bio;
  } else
  dev->bio = dev->biotail = bio;
  spin_unlock(&dev->lock);
}

static inline struct bio *tssdcard_get_bio(struct tssdcard_dev *dev)
{
  struct bio *bio;
  spin_lock(&dev->lock);
  if ((bio = dev->bio)) {
    if (bio == dev->biotail)
      dev->biotail = NULL;
    dev->bio = bio->bi_next;
    bio->bi_next = NULL;
  }
  spin_unlock(&dev->lock);
  return bio;
}

static inline void tssdcard_handle_bio(struct tssdcard_dev *dev, struct bio *bio)
{
  struct bio_vec *bvec;
  sector_t sector, end_sector, n_sectors;
  int i;
  char *buffer;

  sector = bio->bi_sector;
  end_sector = (bio->bi_sector) + (bio->bi_size >> 9) - 1;

  if((bio->bi_size % 512) != 0)
    panic("Invalid transfer, bi_size 512 != 0\n");

  bio_for_each_segment(bvec, bio, i) {
    if((sector + (bvec->bv_len >> 9)) > end_sector)
        n_sectors = end_sector - sector + 1;
    else
       n_sectors = bvec->bv_len >> 9;
    if(n_sectors == 0) continue;

    buffer = kmap(bvec->bv_page) + bvec->bv_offset;
    tssdcard_transfer(dev, sector, n_sectors, buffer, bio_data_dir(bio));
    sector += n_sectors;
    kunmap(bvec->bv_page);
  }

  bio_endio(bio, /*bio->bi_size,*/ 0);

  return;
}


static inline void handle_request (request_queue_t *q, struct bio *bio)
{
  struct tssdcard_dev *dev = q->queuedata;

  if (reqmode != RM_NOQUEUE) spin_unlock_irq(&dev->lock);

  if (threnable) {
    if (reqmode == RM_NOQUEUE && bio)
      tssdcard_add_bio(dev, bio);
    wake_up(&dev->event);
  }
  else {
    if (down_interruptible(&sem))
      return; // -ERESTARTSYS;
    atomic_inc(&busy);
    if (atomic_read(&busy) > 1)
      panic("recursive make_request!\n");

    if (reqmode == RM_NOQUEUE) {
      if(bio) tssdcard_handle_bio(dev, bio);
    } else if (reqmode == RM_FULL)
      tssdcard_handle_fullreq(dev);
    else if (reqmode == RM_SIMPLE)
      tssdcard_handle_simplereq(dev);

    atomic_dec(&busy);
    up(&sem);
  }

  if (reqmode != RM_NOQUEUE) spin_lock_irq(&dev->lock);

  return;
}

/* Request Mode RM_SIMPLE=0 */
static void tssdcard_request(request_queue_t *q)
{
  return handle_request(q, NULL);
}

/* Request Mode RM_FULL=1 Clustering */
static void tssdcard_full_request(request_queue_t *q)
{
  return handle_request(q, NULL);
}

/* Request Mode RM_NOQUEUE=2 */
static int tssdcard_make_request(request_queue_t *q, struct bio *bio)
{
   //printk("%s, 0x%08lX 0x%08lX\n", __func__, q, bio);

  handle_request(q, bio);
  return 0;
}

/*
 * Thread code.
 */
static int tssdcard_thread(void *data)
{
  struct tssdcard_dev *dev = data;
  struct bio *bio;

  current->flags |= PF_NOFREEZE;
  //set_user_nice(current, -20); /* Highest priority */
  //set_user_nice(current, 19); /* Lower priority */

  while (!kthread_should_stop()) {
    wait_event_interruptible(dev->event, dev->bio ||
         (reqmode != RM_NOQUEUE && blk_peek_request(dev->queue)!=NULL) ||
         kthread_should_stop());
    if (!dev->queue && !dev->bio)
      continue;

    //if (down_trylock(&sem))
    if (down_interruptible(&sem))
      continue;
    atomic_inc(&busy);
    if (atomic_read(&busy) > 1)
      panic("recursive make_request!\n");

    if (reqmode == RM_NOQUEUE) {
      bio = tssdcard_get_bio(dev);
      if(bio) tssdcard_handle_bio(dev, bio);
    } else if (reqmode == RM_FULL)
      tssdcard_handle_fullreq(dev);
    else if (reqmode == RM_SIMPLE)
      tssdcard_handle_simplereq(dev);

    atomic_dec(&busy);
    up(&sem);
  }
  return 0;
}


/*
 * Open and close.
 */
//static int tssdcard_open(struct inode *inode, struct file *filp)
static int tssdcard_open(struct block_device *bdev, fmode_t mode)
{
  struct tssdcard_dev *dev = bdev->bd_disk->private_data;

 // filp->private_data = dev;

  if (!atomic_read(&dev->users))
    check_disk_change(bdev);
  atomic_inc(&dev->users);
  if (threnable && dev->thread == NULL && atomic_read(&dev->users)) {
    dev->thread = kthread_create(tssdcard_thread, dev, dev->devname);
    if (IS_ERR(dev->thread))
      dev->thread = NULL;
    else
      wake_up_process(dev->thread);
  }

  return 0;
}

//static int tssdcard_release(struct inode *inode, struct file *filp)
static int tssdcard_release(struct gendisk *disk, fmode_t mode)
{
  struct tssdcard_dev *dev = disk->private_data;

  atomic_dec(&dev->users);
  if (atomic_read(&dev->users) == 0) {
    if (dev->thread != NULL) {
      kthread_stop(dev->thread);
      dev->thread = NULL;
    }
  }

  return 0;
}

int tssdcard_media_changed(struct gendisk *gd)
{
  struct tssdcard_dev *dev = gd->private_data;
  char buf[512];
  int ret;
  if (down_interruptible(&sem)) return -ERESTARTSYS;
  ret = sdread(&dev->tssdcore, 1, buf, 1);
  up(&sem);
  return ret;
}

int tssdcard_revalidate(struct gendisk *gd)
{
  struct tssdcard_dev *dev = gd->private_data;
  dev->size = sdreset(&dev->tssdcore);
  if (dev->size)
    set_capacity(dev->gd, dev->size * 512 / KERNEL_SECTOR_SIZE);
  return 0;
}


static void tssdcard_sdcommit(unsigned long ldev)
{
  struct tssdcard_dev *dev = (struct tssdcard_dev *) ldev;
  char buf[512];
  if (down_interruptible(&sem)) return; // -ERESTARTSYS;
  sdread(&dev->tssdcore, dev->parksect, buf, 1);
  up(&sem);
  return;
}

static int tssdcard_shutdownhook(struct notifier_block *NotifierBlock,
    unsigned long Event, void *Buffer)
{
  int i;
  if (!(Event == SYS_RESTART || Event == SYS_HALT || Event == SYS_POWER_OFF))
    return NOTIFY_DONE;
  for (i = 0; i< ndevices ; i++)
    tssdcard_sdcommit((unsigned long)(devices+i));
  return NOTIFY_OK;
}

static struct notifier_block shutdownhook_notifier = { tssdcard_shutdownhook, NULL, 0 };


#define TSSDCARD_IOC_MAGIC 't'
#define TSSDCARD_GET_WRITE_ACTIVITY _IOR(TSSDCARD_IOC_MAGIC, 1, int)
#define TSSDCARD_GET_SECTOR_COUNT   _IOR(TSSDCARD_IOC_MAGIC, 2, int)


/*
 * The ioctl() implementation
 */

// int tssdcard_ioctl(struct inode *inode, struct file *filp,
//    unsigned int cmd, unsigned long arg)

int tssdcard_ioctl(struct block_device *bdev, fmode_t mode, unsigned int cmd,
		    unsigned long arg)
{
  struct hd_geometry geo;

  // Ian added
  // struct tssdcard_dev *dev = filp->private_data;
  struct tssdcard_dev *dev = bdev->bd_disk->private_data;


  switch(cmd) {
    case HDIO_GETGEO:
	/*
         * get geometry: we have to fake one...  trim the size to a
         * multiple of 512 (256k): tell we have 32 sectors, 16 heads,
         * whatever cylinders.
         */

        // TODO: This should match what the SD cards look like in a
        // USB adapter dongle.  -- JO

      geo.cylinders = dev->size / (16 * 32);
      geo.heads = 16;
      geo.sectors = 32;
      geo.start = 0;
      if (copy_to_user((void __user *) arg, &geo, sizeof(geo)))
        return -EFAULT;
      return 0;

  // Ian added
  case TSSDCARD_GET_WRITE_ACTIVITY:
    {
       if (dev==NULL)
       {
          printk("Error: dev is NULL in tssdcard\n");
          return -EFAULT;
       }

       if (copy_to_user((void __user *) arg, &dev->write_activity, sizeof(dev->write_activity)))
        return -EFAULT;
      dev->write_activity = 0;
      return 0;
    }

    // Ian added
    case TSSDCARD_GET_SECTOR_COUNT:
       if (dev==NULL)
       {
          printk("Error: dev is NULL in tssdcard\n");
          return -EFAULT;
       }
       if (copy_to_user((void __user *) arg, &dev->size, sizeof(dev->size)))
        return -EFAULT;

      return 0;


    //default:
      /*
       * For ioctls we don't understand, let the block layer handle them.
       */
      //return blkdev_ioctl(inode, filp, cmd, arg);

  }

  return -ENOTTY; /* unknown command */
}



/*
 * The device operations structure.
 */
static struct block_device_operations tssdcard_ops = {
  .owner		= THIS_MODULE,
  .open			= tssdcard_open,
  .release		= tssdcard_release,
  .media_changed	= tssdcard_media_changed,
  .revalidate_disk	= tssdcard_revalidate,
  .ioctl		= tssdcard_ioctl
};


/*
 * Set up our internal device.
 */
static void setup_device(struct tssdcard_dev *dev, int which)
{
  static unsigned long addr = 0;
  static unsigned long dmaaddr = 0;
  unsigned int scratchreg0;
  unsigned long conreg;

  printk("%s: Technologic Systems SD card controller, address 0x%x\n", name, io);

  memset (dev, 0, sizeof (struct tssdcard_dev));

#ifdef CONFIG_MACH_TS72XX
  //IO Remapping
  dev->tssdcore.sd_regstart = (unsigned long)__ioremap(io, SD_DEFIOSIZE, 0);

  //verify previous init
  conreg = (unsigned long)__ioremap(SYSCON_BASE, 0xFF, 0);
  scratchreg0 = my_inl(conreg|SYSCON_SCRREG0);
  if (scratchreg0) {
    dev->tssdcore.sdboot_token = scratchreg0;
    my_outl(0, conreg|SYSCON_SCRREG0);
  }
#endif

#ifdef CONFIG_MACH_TS78XX
  //IO Remapping (use the same virtual address for all LUNs)
  if (!addr) addr = (unsigned long)ioremap(io, SD_DEFIOSIZE);
  dev->tssdcore.sd_regstart = addr;
  dev->tssdcore.sd_lun = which;

  //verify previous init
  conreg = (unsigned long)0;
  scratchreg0 = 0;
  if (scratchreg0) {
    dev->tssdcore.sdboot_token = scratchreg0;
    my_outl(0, 0);
  }

#if (0)
  if (board_is_prosoft())
	dev->tssdcore.os_powerok = tssdcard_prosoft_powerok;
#endif

#endif

#ifdef TS_PVT_SOLAR
  dev->tssdcore.os_powerok = tssdcard_powerok;
#endif

  //dev and tssdcore struct initialization
  dev->tssdcore.os_arg = dev;
  dev->tssdcore.os_delay = tssdcard_delay;
  dev->tssdcore.os_dmastream = NULL;
  dev->tssdcore.os_dmaprep = NULL;
  dev->tssdcore.sd_writeparking = 1;
  dev->irq = 0;
  dev->dmach = dmach;
  dev->dmaregs = 0;
  dev->parksect = 0;

  atomic_set(&dev->users,0);

  //SD Card size and Reset
  dev->size = sdreset(&dev->tssdcore);
  if (dev->size == 0) {
    printk("%s: no card found.\n", name);
    //kfree(dev);
    //dev = NULL;
    return;
  }

  dev->gd = alloc_disk(TSSDCARD_MINORS);
  if (! dev->gd) {
    printk (KERN_NOTICE "alloc_disk failure\n");
    return;
  }
  dev->devname = kmalloc(32, GFP_KERNEL);
  strcpy(dev->devname,name);
  snprintf (dev->gd->disk_name, 32, strcat(dev->devname,"%c"), which+'a');
  strcpy(dev->devname,dev->gd->disk_name);
  printk("%s: card /dev/%s has %d sectors (LUN %d)\n", name, dev->devname, dev->size, which);

  //DMA initialization
#ifdef CONFIG_MACH_TS72XX
  if (dmaenable) {
    if (dev->dmach) {
      dev->dmaregs = (unsigned long) __ioremap(DMA_M2M_1_BASE, DMA_M2M_1_SIZE, 0);
      dev->irq = IRQ_EP93XX_DMAM2M1;
    }
    else {
      dev->dmaregs = (unsigned long) __ioremap(DMA_M2M_0_BASE, DMA_M2M_0_SIZE, 0);
      dev->irq = IRQ_EP93XX_DMAM2M0;
    }

    dev->tssdcore.os_dmastream = tssdcard_dmastream;
    dev->tssdcore.os_dmaprep = tssdcard_dmaprep;

#ifdef CONFIG_MACH_TS7350
    dev->tssdcore.sd_lun = which;
    dev->tssdcore.os_irqwait = tssdcard_irqwait;
    dev->sysconfpga = (unsigned long)__ioremap(SYSCONFPGA_BASE, 0xFF, 0);
#else
#endif

    // Enable M2M DMA channel 1 power
    my_outl(0xaa, conreg|SYSCON_SWLOCK);
    if (dmach)
      my_outl(my_inl(conreg|SYSCON_PWRCNT)|SYSCON_PWRCNT_DMA_M2MCH1, conreg|SYSCON_PWRCNT);
    else
      my_outl(my_inl(conreg|SYSCON_PWRCNT)|SYSCON_PWRCNT_DMA_M2MCH0, conreg|SYSCON_PWRCNT);

    // Enable M2M DMA channel 1 DRQ pins
    my_outl(0xaa, conreg|SYSCON_SWLOCK);
    if (dmach)
      my_outl(my_inl(conreg|SYSCON_DEVCFG)|SYSCON_DEVCFG_D1onG, conreg|SYSCON_DEVCFG);
    else
      my_outl(my_inl(conreg|SYSCON_DEVCFG)|SYSCON_DEVCFG_D0onG, conreg|SYSCON_DEVCFG);
  } else {
#ifdef CONFIG_MACH_TS7350
    dev->tssdcore.sd_lun = which;
    dev->tssdcore.os_irqwait = tssdcard_irqwait;
    dev->sysconfpga = (unsigned long)__ioremap(SYSCONFPGA_BASE, 0xFF, 0);
#endif
  }

  __iounmap((unsigned long *)conreg);
#endif

#ifdef CONFIG_MACH_TS78XX
  if (dmaenable) {
    if (!dmaaddr) dmaaddr = (unsigned long)ioremap(TS7800_DMAADDR, 0xFF);
    dev->dmaregs = dmaaddr;

    dev->irq = TS7800_IRQ_DMA;
    dev->irq_wait = TS7800_IRQ_WAIT;

    dev->tssdcore.os_dmastream = tssdcard_dmastream;
    dev->tssdcore.os_dmaprep = tssdcard_dmaprep;
    dev->tssdcore.os_irqwait = tssdcard_irqwait;
  }
#endif

  spin_lock_init(&dev->lock);

  //init_timer(&dev->timer);
  //dev->timer.data = (unsigned long) dev;
  //dev->timer.function = tssdcard_sdcommit;

  dev->thread = NULL;
  if (threnable)
    init_waitqueue_head(&dev->event);

  dev->bio = dev->biotail = NULL;

  //The I/O queue, depending on whether we are using our own make_request function or not.
  switch (reqmode) {
    case RM_NOQUEUE:
      dev->queue = blk_alloc_queue(GFP_KERNEL);
      if (dev->queue == NULL) {
         printk("Can't allocate queue in %s\n", __func__);
         return;
      }
      blk_queue_make_request(dev->queue, tssdcard_make_request);
      break;

    case RM_SIMPLE:
      dev->queue = blk_init_queue(tssdcard_request, &dev->lock);
      if (dev->queue == NULL) return;
      break;

    case RM_FULL:
      dev->queue = blk_init_queue(tssdcard_full_request, &dev->lock);

      // Ian added
      if (elevator_init(dev->queue, "noop"))
         printk("Error: Could not set elevator to noop in %s\n", __func__);

      if (dev->queue == NULL) return;
      break;

    default:
      printk(KERN_NOTICE "Bad request mode %d, using simple\n", reqmode);
      dev->queue = blk_init_queue(tssdcard_request, &dev->lock);
      if (dev->queue == NULL) return;
      break;
  }

  blk_queue_logical_block_size(dev->queue, 512);
  dev->queue->queuedata = dev;

  // Ian added
  dev->write_activity = 0;

  //And the gendisk structure.
  dev->gd->major = tssdcard_major;
  dev->gd->first_minor = which*TSSDCARD_MINORS;
  dev->gd->flags = GENHD_FL_REMOVABLE;
  dev->gd->fops = &tssdcard_ops;
  dev->gd->queue = dev->queue;
  dev->gd->private_data = dev;

  set_capacity(dev->gd, dev->size * 512 / KERNEL_SECTOR_SIZE);
  add_disk(dev->gd);

  //register_reboot_notifier(&shutdownhook_notifier);

  return;
}

// Ian added
static int tssdcard_proc_read(char *buf, char **start, off_t offset,
      int count, int *eof, void *data)
{
   struct tssdcard_dev *dev;
   int len, i;

   for (len=0, i=1; i<ndevices; i++) {
    dev = devices + i;
    if (dev == NULL) continue;

   len += sprintf(buf+len, "write activity: %d\n", dev->write_activity);

   }

   *eof=1;
   return len;
}

static int __init tssdcard_init(void)
{
  int i;

  tssdcard_major = register_blkdev(tssdcard_major, name);
  if (tssdcard_major <= 0) {
    printk(KERN_WARNING "tssdcard: unable to get major number\n");
    return -EBUSY;
  }

  devices = kmalloc(ndevices*sizeof (struct tssdcard_dev), GFP_KERNEL);
  if (devices == NULL)
    goto out_unregister;

  sema_init(&sem, 1);
  atomic_set(&busy,0);

#ifdef CONFIG_MACH_TS78XX
  init_waitqueue_head(&qwait);
#endif

  for (i=0; i<ndevices; i++)
    setup_device(devices + i, i);

  // Ian added
  create_proc_read_entry("tssdcard", 0, NULL, tssdcard_proc_read, devices);


  return 0;

out_unregister:

   printk("Error in %s\n", __func__);

  unregister_blkdev(tssdcard_major, name);
  return -ENOMEM;
}

static void tssdcard_exit(void)
{
  struct tssdcard_dev *dev;
  int i;

  for (i=0; i<ndevices; i++) {
    dev = devices + i;
    if (dev == NULL) continue;

    //del_timer_sync(&dev->timer);

    if (dev->gd) {
      del_gendisk(dev->gd);
      put_disk(dev->gd);
    }

    if (dev->queue) {
      if (reqmode == RM_NOQUEUE)
         kobject_put (&dev->queue->kobj);
         /* blk_put_queue() is no longer an exported symbol */
      else
        blk_cleanup_queue(dev->queue);
    }

    if (dev->thread != NULL) {
      kthread_stop(dev->thread);
      dev->thread = NULL;
    }

#ifdef CONFIG_MACH_TS72XX
    __iounmap((unsigned long *) dev->tssdcore.sd_regstart);
    if (dmaenable) __iounmap((unsigned long *) dev->dmaregs);
#ifdef CONFIG_MACH_TS7350
    __iounmap((unsigned long *) dev->sysconfpga);
#endif
#endif

#ifdef CONFIG_MACH_TS78XX
    if (dmaenable) {
      free_irq(dev->irq_wait, dev);
      if (i==0) iounmap((unsigned long *) dev->dmaregs);
    }

    if (i==0) iounmap((unsigned long *) dev->tssdcore.sd_regstart);
#endif

  }

  remove_proc_entry("tssdcard", NULL);

  unregister_blkdev(tssdcard_major, name);
  kfree(devices);
}

module_init(tssdcard_init);
module_exit(tssdcard_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Technologic Systems - Ronald Gomes <ronald@embeddedARM.com>");
MODULE_DESCRIPTION("SD Card driver for Technologic Systems ARM9 boards");

