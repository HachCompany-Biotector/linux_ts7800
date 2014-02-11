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
#include <asm/mach/time.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/m48t86.h>


#if defined(CONFIG_MTD_PHYSMAP) 
#include <linux/mtd/physmap.h>
#endif

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <asm/mach/irq.h>
#include <asm/mach/map.h>
#include <asm/arch/system.h>
#include <asm/arch/orion_ver.h>

#include <linux/tty.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/serialP.h>
#include <linux/serial_reg.h>
#include <asm/serial.h>
#include <linux/serial_8250.h>

#include <asm/arch/serial.h>
#include "asm/arch/time.h"

#include "mvCtrlEnvLib.h"
#include "mvCpuIf.h"
#include "mvBoardEnvLib.h"
#include "mvIdma.h"

#include "buffalo/miconcntl.h"

extern void __init mv_map_io(void);
extern void __init mv_init_irq(void);

#ifdef CONFIG_MV_CESA_TEST
extern void    cesaTestStart(int bufNum, int bufSize);
#endif

extern MV_U32 gBoardId; 
unsigned int mv_orion_ver = 0x0;
u32 mvTclk = 166666667;
u32 mvPrescaler;
u32 mvSysclk = 200000000;
u32 overEthAddr = 0;
u8 mvMacAddr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

volatile static unsigned char *ts7800_rtc = NULL;
static int config_ts7800_rtc = 0;

#ifdef CONFIG_UBOOT_STRUCT
  #define ATAG_MV_UBOOT	0x41000403
  // Marvell uboot parameters
  struct tag_mv_uboot {
	  u32 uboot_version;
	  u32 tclk;
	  u32 sysclk;
	  u32 isUsbHost;
	  u32 overEthAddr;
	  u8  macAddr[6];
  };  
  u32 mvIsUsbHost = 1;
#else
  #ifdef CONFIG_MV_USB_HOST
    u32 mvIsUsbHost = 1;
  #else
    u32 mvIsUsbHost = 0;
  #endif
#endif

EXPORT_SYMBOL(mvMacAddr);
EXPORT_SYMBOL(mvSysclk);
EXPORT_SYMBOL(mvCtrlModelGet);
EXPORT_SYMBOL(mvOsIoUncachedMalloc);
EXPORT_SYMBOL(mvOsIoUncachedFree);

void print_board_info(void)
{
    char name_buff[50];
    printk("\nMarvell LSP Version %s",LSP_VERSION);

    mvBoardNameGet(name_buff);
    printk(" -- %s\n\n",name_buff);
}

/* Add the uart to the console list (ttyS1) . */
static void serial_initialize_ttyS1(void)
{
    struct uart_port        serial_req;

	memset(&serial_req, 0, sizeof(serial_req));
	serial_req.line = 1;
	serial_req.uartclk = BASE_BAUD * 16;
	serial_req.irq = IRQ_UART1;
	serial_req.flags = STD_COM_FLAGS;
	serial_req.iotype = SERIAL_IO_MEM;
	serial_req.membase = (char *)PORT1_BASE;
	serial_req.mapbase = (unsigned long)PORT1_BASE;
	serial_req.regshift = 2;

	if (early_serial_setup(&serial_req) != 0) {
		printk("Early serial init of port 1 failed\n");
	}

	return;
}

/* Add the uart to the console list (ttyS0) . */
static void serial_initialize(void)
{
    struct uart_port        serial_req;

	memset(&serial_req, 0, sizeof(serial_req));
	serial_req.line = 0;
	serial_req.uartclk = BASE_BAUD * 16;
	serial_req.irq = IRQ_UART0;
	serial_req.flags = STD_COM_FLAGS;
	serial_req.iotype = SERIAL_IO_MEM;
	serial_req.membase = (char *)PORT0_BASE;
	serial_req.mapbase = (unsigned long)PORT0_BASE;
	serial_req.regshift = 2;

	if (early_serial_setup(&serial_req) != 0) {
		printk("Early serial init of port 0 failed\n");
	}

    return;
}

static struct platform_device lspro_i2c_controller = {
	.name			= "MV88F5182-I2C",
	.id			= -1,
//	.dev.platform_data	= &lspro_i2c_gpio_pins,
	.num_resources		= 0,
};



static void usb_release(struct device *pdev)
{
    // normally not freed
    kfree(pdev);
} 

#if defined(CONFIG_MTD_PHYSMAP) 

// Establish  flash map for generic flash driver
static struct mtd_partition lspro_flash_resources[] = {
    {
	    .name = "u-boot",
		.offset = 0,
		.size = 0x40000,
	},
};

static struct physmap_flash_data lspro_flash_data = {
	.width = 1,
	.parts = lspro_flash_resources,
	.nr_parts = ARRAY_SIZE(lspro_flash_resources),
};

static struct resource lspro_flash_resource = {
	.start = 0xFF800000,
	.end = 0xFF800000 + 0x400000 - 1,
	.flags = IORESOURCE_MEM,
};

static struct platform_device lspro_flash = {
	.name = "physmap-flash",
	.id	= -1,
	.dev = {
		.platform_data = &lspro_flash_data,
	},
	.num_resources = 1,
	.resource = &lspro_flash_resource,
};

#endif

// Platform.txt refers.
static struct resource lspro_ehci_resources[] = {
	[0] = {
		.start = 0xF1050100,
		.end = 0xF1050100 + 4096,
		.flags = IORESOURCE_DMA,
	},
	[1] = {
		.start = IRQ_USB_CTRL(0),
		.end = IRQ_USB_CTRL(0),
		.flags = IORESOURCE_IRQ,
	},
};

static u64 ehci_dmamask = 0xffffffffULL;

static struct platform_device lspro_ehci_device = {
	.name = "mv5182_ehci",
	.id = 0,
	.dev = {
		.bus_id = "platform",
		.dma_mask = &ehci_dmamask,
		.coherent_dma_mask	= 0xffffffff,
		.release = usb_release,
	},
	.num_resources = ARRAY_SIZE(lspro_ehci_resources),
	.resource = lspro_ehci_resources,
};

static struct resource lspro_ehci_resources1[] = {
	[0] = {
		.start = 0xF10A0100,
		.end = 0xF10A0100 + 4096,
		.flags = IORESOURCE_DMA,
	},
	[1] = {
		.start = IRQ_USB_CTRL(1),
		.end = IRQ_USB_CTRL(1),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device lspro_ehci_device1 = {
	.name = "mv5182_ehci",
	.id = 1,
	.dev = {
		.bus_id = "platform",
		.dma_mask = &ehci_dmamask,
		.coherent_dma_mask	= 0xffffffff,
		.release = usb_release,
	},
	.num_resources = ARRAY_SIZE(lspro_ehci_resources1),
	.resource = lspro_ehci_resources1,
};

static struct platform_device *lspro_devices[] __initdata = {
#if defined(CONFIG_MTD_PHYSMAP) 
	&lspro_flash,
#endif
	&lspro_ehci_device,
	&lspro_ehci_device1,
	&lspro_i2c_controller,
};

static void ts7800_rtc_writebyte(unsigned char value, unsigned long addr)
{
        if(!config_ts7800_rtc) {
                ts7800_rtc = (unsigned char *)
                        ioremap(TS7800_RTC_BASE, TS7800_RTC_SIZE);
                //ensure RTC battery is enabled
                ts7800_rtc[TS7800_RTC_INDEX] = 0xa;
                ts7800_rtc[TS7800_RTC_DATA] = 0x20;
                config_ts7800_rtc = 1;
        }
        ts7800_rtc[TS7800_RTC_INDEX] = addr;
        ts7800_rtc[TS7800_RTC_DATA] = value;

}

static unsigned char ts7800_rtc_readbyte(unsigned long addr)
{
        unsigned char value;

        if(!config_ts7800_rtc) {
                ts7800_rtc = (unsigned char *)
                        ioremap(TS7800_RTC_BASE, TS7800_RTC_SIZE);
                //ensure RTC battery is enabled
                ts7800_rtc[TS7800_RTC_INDEX] = 0xa;
                ts7800_rtc[TS7800_RTC_DATA] = 0x20;
                config_ts7800_rtc = 1;
        }

        ts7800_rtc[TS7800_RTC_INDEX] = addr;
        value = ts7800_rtc[TS7800_RTC_DATA];

        return value;
}

static struct m48t86_ops ts7800_rtc_ops = {

                .readbyte               = ts7800_rtc_readbyte,
                .writebyte              = ts7800_rtc_writebyte,
};

static struct platform_device ts7800_rtc_device = {
        .name                   = "rtc-m48t86",
        .id                     = -1,
        .dev                    = {
                .platform_data          = &ts7800_rtc_ops,
        },
        .num_resources          = 0,
};


static void __init mv_init(void)
{
#ifdef CONFIG_MV_CESA
	MV_CPU_DEC_WIN crypt_win;
#endif
	/* init the Board environment */
	mvBoardEnvInit();

	/* init the controller environment */
	if( mvCtrlEnvInit() ) {
		printk( "Controller env initialization failed.\n" );
		return;
	}

	// Init the CPU windows setting and the access protection windows.
	if( mvCpuIfInit() ) {
		printk( "Cpu Interface initialization failed.\n" );
		return;
	}

	mv_orion_ver = MV_ORION1; /* Orion I */ 

    print_board_info();
	
#ifndef CONFIG_UBOOT_STRUCT
    mvTclk = mvBoardTclkGet();
    mvSysclk = mvBoardSysClkGet();
    printk("\nDetected Tclk %d and SysClk %d \n",mvTclk, mvSysclk);
#endif

#ifdef CONFIG_MV_CESA
	// Initialise hardware crypto device

	if( MV_OK != mvCpuIfTargetWinGet(DEVICE_CS2, &crypt_win) ) {
		printk( "get window failed.\n" );
		return;
	}
	
	crypt_win.addrWin.baseLow = CRYPT_ENG_BASE;
	crypt_win.addrWin.baseHigh = 0;
	crypt_win.addrWin.size = CRYPT_ENG_SIZE;
	crypt_win.enable = 1;
	
	if( MV_OK != mvCpuIfTargetWinEnable(DEVICE_CS2, 0) ) {
		printk( "disable win failed.\n" );
		return;
	}
	
	if( MV_OK != mvCpuIfTargetWinSet(CRYPT_ENG, &crypt_win) ) {
		printk( "set window failed.\n" );
		return;
	}
	
	mvDmaInit();
	
	#ifdef CONFIG_MV_CESA_TEST
	    	cesaTestStart(1, 6500);
	#endif
#endif

	// Do the register here (only useful if a serial connection is added) to see debug stuff
	platform_add_devices(lspro_devices, ARRAY_SIZE(lspro_devices));

	serial_initialize();
	serial_initialize_ttyS1();

	platform_device_register(&ts7800_rtc_device);

	// Setup power-off jump point
//	pm_power_off = miconCntl_PowerOff;
    return;
}

// Returns number of usec since last clock interrupt.  Note that interrupts
// will have been disabled by do_gettimeoffset()
/*
  4-8-8, modified by Technologic Systems
  Fixed two bugs in this code which both caused gettimeofday()'s us
  counter to be non-monotonic:
  1. Multiplication was overflowing a 32-bit integer, so the equation
     governing the return value had to be tweaked
  2. Added a second read of the timer if an interrupt occured to prevent
     a more obscure error.

  WARNING: Assumings mvTclk = 166 666 667
*/
 static unsigned long mv_gettimeoffset(void)
{
        unsigned int ticks = (mvTclk / HZ) - __raw_readl((INTER_REGS_BASE | CNTMR_VAL_REG(LSP_CNTMR)));

	u32 cause  = __raw_readl(INTER_REGS_BASE + BRIDGE_INT_CAUSE_REG);	
	// check if we got an interrupt meanwhile
	cause = MV_ARM_32BIT_LE(cause);
	if (cause & TIMER_BIT_MASK(LSP_CNTMR)) {
          	ticks = (mvTclk / HZ) - __raw_readl((INTER_REGS_BASE | CNTMR_VAL_REG(LSP_CNTMR)));
		ticks += (mvTclk / HZ);
	}
//	return ((unsigned long long)ticks * 25769798) >> 32;
	return ((unsigned long long)ticks * mvPrescaler) >> 32;
}

// IRQ handler for the timer
static irqreturn_t mv_timer_interrupt(int irq, void *dev_id)
{
	u32 cause;
	u32 mask;
	
	// Interrupt safe please
	write_seqlock(&xtime_lock);
	
	cause = __raw_readl(INTER_REGS_BASE | BRIDGE_INT_CAUSE_REG);
	mask = __raw_readl(INTER_REGS_BASE | BRIDGE_INT_MASK_REG); 
	cause = MV_ARM_32BIT_LE(cause);
	mask = MV_ARM_32BIT_LE(mask);
	
	// check if we realy received a timer irq.
	if( ((cause & mask) & TIMER_BIT_MASK(LSP_CNTMR)) != 0 ) {
		// clear the timer irq
		cause = (cause & ~(TIMER_BIT_MASK(LSP_CNTMR)));		
		__raw_writel(MV_ARM_32BIT_LE(cause), (INTER_REGS_BASE | BRIDGE_INT_CAUSE_REG)) ;		
		timer_tick();
	}
	
	// no support for other irqs.
	else { 
		printk("Error IRQ not supported ??\n");
		write_sequnlock(&xtime_lock);
		return IRQ_NONE;
	}

	write_sequnlock(&xtime_lock);
	return IRQ_HANDLED;
}

static struct irqaction mv_timer_irq = {
	.name           = "Timer Tick",
	.flags          = IRQF_DISABLED | IRQF_TIMER,
	.handler        = mv_timer_interrupt
};

// Set up timer interrupt.
static void __init mv_init_timer(void)
{
	u32 timer;
	u32 cntmrCtrl;
	u32 timer_reload = mvTclk / HZ;
	
	switch (mvTclk) {
	case 166666667:
		mvPrescaler = 25769804;
		break;
	case 133333333:
		mvPrescaler = 32212255;
		break;
	case 200000000:
		mvPrescaler = 21474836;
		break;
	default:
		panic("mvTclk invalid value!!!");
		break;
	}
	// load value onto counter\timer
	__raw_writel(timer_reload, (INTER_REGS_BASE | CNTMR_RELOAD_REG(LSP_CNTMR)));
	__raw_writel(timer_reload, (INTER_REGS_BASE | CNTMR_VAL_REG(LSP_CNTMR)));
	// set the counter to load in the first time
	__raw_writel(timer_reload, (INTER_REGS_BASE | CNTMR_VAL_REG(LSP_CNTMR)));
	// read control register
	cntmrCtrl = __raw_readl((INTER_REGS_BASE | CNTMR_CTRL_REG));
	// enable counter\timer
	cntmrCtrl |= CTCR_ARM_TIMER_EN(LSP_CNTMR);
	// Auto mode
	cntmrCtrl |= CTCR_ARM_TIMER_AUTO_EN(LSP_CNTMR);
	__raw_writel(cntmrCtrl, (INTER_REGS_BASE | CNTMR_CTRL_REG));

	// enable only the LSP timer interrupt
	timer = __raw_readl(INTER_REGS_BASE + MV_AHBTOMBUS_IRQ_CAUSE_REG);	
	timer = MV_ARM_32BIT_LE(timer);
	timer |= TIMER_BIT_MASK(LSP_CNTMR);

	__raw_writel(MV_ARM_32BIT_LE(timer), (INTER_REGS_BASE + MV_AHBTOMBUS_IRQ_CAUSE_REG));
	
	// Make irqs happen for the system timer
	setup_irq(TIME_IRQ, &mv_timer_irq);
}

static struct sys_timer mv_timer = {
	.init           = mv_init_timer,
	.offset         = mv_gettimeoffset,
};

// Handle problems with UBoot memory tags and pickup Marvell parameters
static void __init mv_fixup(struct machine_desc *desc, struct tag *tags, char **cmdline, struct meminfo *mi)
{
	struct tag *tag = (struct tag *)tags;
	struct tag_mv_uboot *mv_uboot;
	unsigned int mvUbootVer = 0;
	int i;

	// Skip to the memory tag
	while (tag->hdr.tag != ATAG_MEM) {
		tag = tag_next(tag);
	};
	
#ifndef CONFIG_TS7800_PLATFORM
	// Correct the UBoot memory tag by removing invalid memory blocks
	if (tag->hdr.tag == ATAG_MEM) {
		// bypass next three tags (dirty hack)
		tag->hdr.size = tag->hdr.size * 4;
		tag = tag_next(tag);
	}
#endif

#ifdef CONFIG_TS7800_PLATFORM
	overEthAddr = 0;
	for (i = 0; i < 6; i ++) mvMacAddr[i] = 0;
#endif
	
#ifdef CONFIG_UBOOT_STRUCT
	while (tag->hdr.tag != ATAG_NONE)
	{
		// Locate the Marvell UBoot message
		if (tag->hdr.tag == ATAG_MV_UBOOT) {
			tag->hdr.tag = ATAG_NONE;
			tag->hdr.size = 0;
			
			// Get pointer to our block
			mv_uboot = (struct tag_mv_uboot*)&tag->u;
		    mvTclk = mv_uboot->tclk;
		    mvSysclk = mv_uboot->sysclk;
		    mvUbootVer = mv_uboot->uboot_version;
		    mvIsUsbHost = mv_uboot->isUsbHost;

			// Some clock fixups
		    if(mvTclk == 166000000) mvTclk = 166666667;
		    else if(mvTclk == 133000000) mvTclk = 133333333;
		    else if(mvSysclk == 166000000) mvSysclk = 166666667;

		    printk("Using UBoot passing parameters structure\n");
		    printk("Sys Clk = %d, Tclk = %d\n",mvSysclk ,mvTclk  );

			gBoardId =  (mvUbootVer & 0xff);
			
			if( mvUbootVer > 0x01040100 )  // releases after 1.4.2
				overEthAddr = mv_uboot->overEthAddr;
			else if (mvUbootVer >= 0x01090200) {
				memcpy(mvMacAddr, mv_uboot->macAddr, 6);
			}
			
			memcpy(mvMacAddr, mv_uboot->macAddr, 6);
			break;
		}
		
		tag = tag_next(tag);
	};
#endif
}

MACHINE_START(MV88fxx81 ,"MV-88fxx81")
	.phys_io      = 0xF1000000, 
	.io_pg_offst  = ((0xF1100000)>>17)&0xFFFC,
    .boot_params  = 0x00000100,
	.fixup		  = mv_fixup,
    .map_io       = &mv_map_io,
    .init_irq     = &mv_init_irq,
    .timer        = &mv_timer,
    .init_machine = &mv_init
MACHINE_END

