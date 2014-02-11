/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysdev.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>
#include <asm/mach/irq.h>
#include <asm/mach/map.h>

#include "mvCtrlEnvLib.h"
#include "mvOs.h"

#ifdef CONFIG_TS7800_PLATFORM
static unsigned int ts7800_fpga_msi_mask;
#endif

static void mv_mask_irq(unsigned int irq)
{
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
#ifdef CONFIG_TS7800_PLATFORM
		/* Update MSI mask reg on FPGA */
		//printk(KERN_INFO "mask irq %d\n", irq);
		ts7800_fpga_msi_mask = *(volatile unsigned int *)(0xe8000204);
		ts7800_fpga_msi_mask &= ~(1 << (irq - 64));
		*(volatile unsigned int *)(0xe8000204) = ts7800_fpga_msi_mask;
#endif
	}
	return;
}

static void mv_ack_irq(unsigned int irq)
{
	if (irq > 63) {
		//printk(KERN_INFO "ack irq %d\n", irq);
		MV_REG_BIT_RESET(MV_DOORBELL_REG, (1 << (irq - 64)));
	}
}

static void mv_unmask_irq(unsigned int irq)
{
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
#ifdef CONFIG_TS7800_PLATFORM
		/* Update MSI mask reg on FPGA */
		//printk(KERN_INFO "unmask irq %d\n", irq);
		MV_REG_BIT_RESET(MV_DOORBELL_REG, (1 << (irq - 64)));
		ts7800_fpga_msi_mask = *(volatile unsigned int *)(0xe8000204);
		ts7800_fpga_msi_mask |= (1 << (irq - 64));
		*(volatile unsigned int *)(0xe8000204) = ts7800_fpga_msi_mask;
#endif
	}
	return;
}

struct irq_chip mv_chip = {
	.name   = "mv",
	.ack	= mv_ack_irq,
	.mask	= mv_mask_irq,
	.unmask = mv_unmask_irq,
};

// TODO: Move GPP stuff into its own handler
void __init mv_init_irq(void)
{
 	u32 i;
                         
	/* Disable all interrupts initially. */
	MV_REG_WRITE(MV_IRQ_MASK_REG, 0x0);
	// Disable GPIO interrupts
	MV_REG_WRITE(MV_GPP_IRQ_MASK_REG, 0x0);

	/* enable GPP in the main cause */
	MV_REG_BIT_SET(MV_IRQ_MASK_REG, (1 << IRQ_GPP_0_7) | (1 << IRQ_GPP_8_15));
	MV_REG_BIT_SET(MV_IRQ_MASK_REG, (1 << IRQ_GPP_16_23) | (1 << IRQ_GPP_24_31));	

	/* enable doorbell in the main cause */
	MV_REG_BIT_SET(MV_IRQ_MASK_REG, (1 << IRQ_DOORBELL));

	ts7800_fpga_msi_mask = *(volatile unsigned int *)(0xe8000204);

    /*
          * Enable PCI irqs.  The actual PCI[AB] decoding is done in
          * entry-macro.S, so we don't need a chained handler for the
          * PCI interrupt source.
          */

	/* Do the core module ones */
	for (i = 0; i < NR_IRQS; i++) {
		set_irq_chip(i, &mv_chip);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID | IRQF_PROBE);
	}
	
	/* TBD. Add support for error interrupts */

	// Clear all interrupts
	MV_REG_WRITE(MV_IRQ_CAUSE_REG, 0x0);
	MV_REG_WRITE(MV_GPP_IRQ_CAUSE_REG, 0x0);

#ifndef CONFIG_TS7800_PLATFORM
	// Disable GPP lines.
	MV_REG_WRITE(GPP_DATA_OUT_EN_REG, 0xFC01000C);
	MV_REG_WRITE(GPP_DATA_IN_POL_REG, 0);
#endif

	return;
}

