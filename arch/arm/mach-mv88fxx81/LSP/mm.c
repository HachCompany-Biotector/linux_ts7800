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
#include <asm/memory.h>
 
#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <asm/mach/irq.h>
#include <asm/mach/map.h>
#include "mvSysHwConfig.h"

/*
 * Logical      Physical
 * e0000000     e0000000	PEX 0 MEM
 * e8000000     e8000000        PEX 1 MEM
 * f1000000     f1000000        Internal Regs
 * f2000000     f2000000        PEX 0 IO
 * f2100000     f2100000        PEX 1 IO
 * 	        f4000000        Extra flash
 */

struct map_desc mv_io_desc[] __initdata = {
	{
		.virtual        = PEX0_MEM_BASE,
		.pfn            = __phys_to_pfn(PEX0_MEM_BASE),
		.length         = PEX0_MEM_SIZE,
		.type           = MT_DEVICE,
	},
	{
		.virtual        = PCI0_MEM_BASE,
		.pfn            = __phys_to_pfn(PCI0_MEM_BASE),
		.length         = PCI0_MEM_SIZE,
		.type           = MT_DEVICE,
	},
	{
		.virtual        = INTER_REGS_BASE,
		.pfn            = __phys_to_pfn(INTER_REGS_BASE),
		.length         = SZ_1M,
		.type           = MT_DEVICE,
	},
	{ 
		.virtual        =  PEX0_IO_BASE,
		.pfn            =  __phys_to_pfn(PEX0_IO_BASE),
		.length         =  PEX0_IO_SIZE,
		.type           =  MT_DEVICE,
	},
	{
		.virtual        = PCI0_IO_BASE,
		.pfn            = __phys_to_pfn(PCI0_IO_BASE),
		.length         = PCI0_IO_SIZE,
		.type           = MT_DEVICE,
	},
 /* For Orion NAS, this one also cover the Internal SRAM */
 	{
 		.virtual        = PEX_CONFIG_RW_WA_BASE,
 		.pfn            = __phys_to_pfn(PEX_CONFIG_RW_WA_BASE),
 		.length         = PEX_CONFIG_RW_WA_SIZE,
 		.type           = MT_DEVICE,
	},
	{
		.virtual        = DEVICE_CS2_BASE,
		.pfn            = __phys_to_pfn(DEVICE_CS2_BASE),
		.length         = DEVICE_CS2_SIZE,
		.type           = MT_DEVICE,
	},
	{
		.virtual        = DEVICE_CS0_BASE,
		.pfn            = __phys_to_pfn(DEVICE_CS0_BASE),
		.length         = DEVICE_CS0_SIZE,
		.type           = MT_DEVICE
	}
};
                                                                                                                                               
void __init mv_map_io(void)
{
	iotable_init(mv_io_desc, ARRAY_SIZE(mv_io_desc));
}

