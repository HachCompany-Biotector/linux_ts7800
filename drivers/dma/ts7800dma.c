
#include <linux/init.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/kernel.h> 
#include <linux/errno.h>      
#include <linux/timer.h>
#include <linux/types.h>        
#include <linux/kdev_t.h>
#include <linux/vmalloc.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <asm/atomic.h>
#include <linux/semaphore.h>
#include <asm/current.h>
#include <linux/fs.h>

#define TS7800_IRQ_DMA          65
#define TS7800_CPUDOORBELL      (ORION5X_REGS_VIRT_BASE + 0x20400) /*0xF1020400*/
#define TS7800_FPGAMSIMASK      0xE8000204
#define TS7800_DMAADDR		0xE8000400
#define DOORBELL_DMABIT         0x1 << 1
#define DMASRC_OFFSET		0x1 << 2
#define DMADST_OFFSET		0x1 << 3
#define DMACMD_OFFSET		0x1 << 0

struct ts7800dma_dev {
  unsigned long dmaregs;
  struct semaphore sem;
  wait_queue_head_t qdma;
  int irq;
};

static struct ts7800dma_dev *ts7800dma = NULL;
EXPORT_SYMBOL(ts7800dma);

void __attribute__((naked))
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
EXPORT_SYMBOL(xdma_clean_range);

void __attribute__((naked))
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
EXPORT_SYMBOL(xdma_inv_range);

static irqreturn_t int_handler(int irq, void *vidp, struct pt_regs regs)
{
  struct ts7800dma_dev *dev = (struct ts7800dma_dev *)vidp;
  BUG_ON(irq != dev->irq);
  disable_irq_nosync(irq);
  wake_up(&dev->qdma);
  return IRQ_HANDLED;
}


int dma_op (unsigned long dev_addr, unsigned long dat, unsigned int len, short rw)
{
  struct ts7800dma_dev *dev = ts7800dma;
  unsigned long msireg, i;

  //printk("dma_op()\n");
  //printk("dma_op(0x%08lX, 0x%08lX, %d, %s)\n", dev_addr, dat, len, rw==READ?"READ":"WRITE");
  //printk("dev = 0x%08lX\n", dev);
  
  if (down_interruptible(&dev->sem))
    return -ERESTARTSYS;
 
  writel(dat, dev->dmaregs+4 );
 
  writel(dev_addr, dev->dmaregs+8);

  if (rw == READ) 
    writel((len/4)|(1<<12)|(4<<16)|(2<<19), dev->dmaregs);
  else if (rw == WRITE) 
    writel((len/4)|(4<<16)|(2<<19), dev->dmaregs);
 
  DEFINE_WAIT(wait);

  prepare_to_wait(&dev->qdma, &wait, TASK_UNINTERRUPTIBLE);

  //printk("1\n");
  
  request_irq(dev->irq, int_handler, IRQF_DISABLED, NULL, dev);

  if (!(readl((virt_to_phys(TS7800_CPUDOORBELL))?TS7800_CPUDOORBELL:0xF1020400) &DOORBELL_DMABIT))
    schedule();
 
//printk("2\n");
 
  free_irq(dev->irq, dev);

  //printk("3\n");
  finish_wait(&dev->qdma, &wait);

  //printk("4\n");
  
  up(&dev->sem);

  //printk("dma_op() done\n");
  
  return 0;
}
EXPORT_SYMBOL(dma_op);


static int __init dma_bus_init(void)
{
  struct ts7800dma_dev *dev = NULL;  
  printk("TS-7800 DMA controller init. \n");
  dev = kmalloc(sizeof (struct ts7800dma_dev), GFP_KERNEL);
  memset (dev, 0, sizeof (struct ts7800dma_dev));
  dev->dmaregs = (unsigned long)ioremap(TS7800_DMAADDR, 0xFF);; 
  init_MUTEX(&dev->sem);
  init_waitqueue_head(&dev->qdma);  
  dev->irq = TS7800_IRQ_DMA;
  ts7800dma = dev;
  return 0;
}
subsys_initcall(dma_bus_init);



