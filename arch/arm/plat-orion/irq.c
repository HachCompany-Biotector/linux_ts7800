/*
 * arch/arm/plat-orion/irq.c
 *
 * Marvell Orion SoC IRQ handling.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <plat/irq.h>


#ifdef CONFIG_MACH_TS78XX
#define TS78XX_FPGA_REGS_VIRT_BASE	0xff900000
static unsigned int ts7800_fpga_msi_mask;
#endif


static void orion_irq_mask(u32 irq)
{
   
#if (0)   
	void __iomem *maskaddr = get_irq_chip_data(irq);
	u32 mask;

	mask = readl(maskaddr);
	mask &= ~(1 << (irq & 31));
	writel(mask, maskaddr);
	
#else

	if(irq < 32)
		MV_REG_BIT_RESET(MV_IRQ_MASK_REG, (1 << irq) );
	else if (irq < 64)
	{
#ifdef MICON_INT_EDGE
		// for edge sensitive
		if (irq==32+2){
			MV_REG_BIT_RESET(MV_GPP_IRQ_MASK_REG-4, (1 << (irq - 32)) );
		}else{
			MV_REG_BIT_RESET(MV_GPP_IRQ_MASK_REG, (1 << (irq - 32)) );
		}
#else
		MV_REG_BIT_RESET(MV_GPP_IRQ_MASK_REG, (1 << (irq - 32)) );
#endif
	} else {
		MV_REG_BIT_RESET(MV_DOORBELL_MASK_REG, (1 << (irq - 64)));
#ifdef CONFIG_MACH_TS78XX
		/* Update MSI mask reg on FPGA */
		//printk(KERN_INFO "mask irq %d\n", irq);
		ts7800_fpga_msi_mask = *(volatile unsigned int *)((TS78XX_FPGA_REGS_VIRT_BASE+0x204));
		ts7800_fpga_msi_mask &= ~(1 << (irq - 64));
		*(volatile unsigned int *)((TS78XX_FPGA_REGS_VIRT_BASE+0x204)) = ts7800_fpga_msi_mask;
#endif
	}
	return;

#endif
}


static void mv_ack_irq(unsigned int irq)
{
	if (irq > 63) {
		//printk(KERN_INFO "ack irq %d\n", irq);
		MV_REG_BIT_RESET(MV_DOORBELL_REG, (1 << (irq - 64)));
	}
}


static void orion_irq_unmask(u32 irq)
{
#if (0)   
	void __iomem *maskaddr = get_irq_chip_data(irq);
	u32 mask;

	mask = readl(maskaddr);
	mask |= 1 << (irq & 31);
	writel(mask, maskaddr);
#else

	if(irq < 32)
		MV_REG_BIT_SET(MV_IRQ_MASK_REG, (1 << irq) );
	else if (irq < 64) 
	{
#ifdef MICON_INT_EDGE
		// for edge sensitive
		if (irq==32+2){
			MV_REG_BIT_SET(MV_GPP_IRQ_MASK_REG-4, (1 << (irq - 32)) );
		}else{
			MV_REG_BIT_SET(MV_GPP_IRQ_MASK_REG, (1 << (irq - 32)) );
		}
#else
		MV_REG_BIT_SET(MV_GPP_IRQ_MASK_REG, (1 << (irq - 32)) );
#endif
	} else {
		MV_REG_BIT_SET(MV_DOORBELL_MASK_REG, (1 << (irq - 64)));
#ifdef CONFIG_MACH_TS78XX
		/* Update MSI mask reg on FPGA */
		//printk(KERN_INFO "unmask irq %d\n", irq);
		MV_REG_BIT_RESET(MV_DOORBELL_REG, (1 << (irq - 64)));
		ts7800_fpga_msi_mask = *(volatile unsigned int *)((TS78XX_FPGA_REGS_VIRT_BASE+0x204));
		ts7800_fpga_msi_mask |= (1 << (irq - 64));
		*(volatile unsigned int *)((TS78XX_FPGA_REGS_VIRT_BASE+0x204)) = ts7800_fpga_msi_mask;
#endif
	}
	return;
	
#endif	
}

static struct irq_chip orion_irq_chip = {
	.name		= "orion_irq",
	.ack	= mv_ack_irq,
	.mask		= orion_irq_mask,
	.mask_ack	= orion_irq_mask,
	.unmask		= orion_irq_unmask,
};

void __init orion_irq_init(unsigned int irq_start, void __iomem *maskaddr)
{
	unsigned int i;
		
	/*
	 * Mask all interrupts initially.
	 */
	writel(0, maskaddr);

	/*
	 * Register IRQ sources.
	 */
	for (i = 0; i < NR_IRQS; i++) {
		unsigned int irq = irq_start + i;

		set_irq_chip(irq, &orion_irq_chip);
		set_irq_chip_data(irq, maskaddr);
		set_irq_handler(irq, handle_level_irq);
		irq_desc[irq].status |= IRQ_LEVEL;
		set_irq_flags(irq, IRQF_VALID);
	}
		
}
