/* 
	==================================================================================
    ax88796b.c: A SAMSUNG S3C2440 Linux2.6.x Ethernet device driver for ASIX AX88796B chips.
 
    This program is free software; you can distrine_block_inputbute it and/or modify it under
    the terms of the GNU General Public License (Version 2) as published by the Free Software
    Foundation.

    This program is distributed in the hope that it will be useful, but WITHOUT ANY 
	WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
	PARTICULAR PURPOSE.  See the GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    59 Temple Place - Suite 330, Boston MA 02111-1307, USA.

	This program is based on

	ne.c:		A general non-shared-memory NS8390 ethernet driver for linux
				Written 1992-94 by Donald Becker.

	8390.c:     A general NS8390 ethernet driver core for linux.
				Written 1992-94 by Donald Becker.

	Version history:

		12/09/2006	1.0.0.0 - Initial release.


  
	==================================================================================
	Driver Overview
	==================================================================================
	ASIX AX88796B 3-in-1 Local Bus 8/16-bit Fast Ethernet Linux Driver

	The AX88796B Ethernet controller is a high performance and highly integrated
	local CPU bus Ethernet controllers with embedded 10/100Mbps	PHY/Transceiver
	16K bytes SRAM and supports both 8-bit and 16-bit local CPU interfaces for any 
	embedded systems. 

	If you look for more details, please visit ASIX's web site (http://www.asix.com.tw).


	==================================================================================
	COMPILING DRIVER
	==================================================================================
	Prepare: 

		AX88796B Linux Driver.
		Linux Kernel source code.
		Cross-Compiler.

	Getting Start:

		1.Extracting the AX88796B source file by executing the following command.
			[root@localhost]# tar jxvf ax88796b-arm-linux2.4.tar.bz2

		2.Edit the makefile to specifie the path of target platform Linux Kernel source.
		  EX:
				KDIR	:= /work/linux-2.6.17.11

		3.Executing 'make' command to compiler AX88796B Driver.

		4.If the compilation well, the ax88796.ko will be created under the current directory.


	==================================================================================
	DRIVER PARAMETERS
	==================================================================================
	The following parameters can be set when using insmod.
	EX: [root@localhost ax88796b]# insmod  ax88796.ko  mem=0x08000000

	mem=0xNNNNNNNN 
		specifies the physical base address that AX88796B can be accessed.
		Default value '0x08000000'.

	irq=N
		specifies the irq number. Default value '0x27'.

	media=(0=auto, 1=100full, 2=100half, 3=10full, 4=10half)
		Media mode control. Default value 'auto'.		
*/

static const char version1[] = "AX88796B: v1.1.0\n";
static const char version2[] = "AX88796B: 09/20/06 (http://www.asix.com.tw)\n";

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/etherdevice.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/crc32.h>

#include <asm/system.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <linux/vmalloc.h>

#include "ax88796b.h"
#include <asm/page.h>

#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
#include <linux/if_vlan.h>
#endif

#ifdef CONFIG_MACH_TS78XX
#define __TS7XXX_PC104IO8		0xee000000
#define __TS7XXX_PC104IO16	   0xef000000
#define TS7XXX_PC104IO8 (p1)
#define TS7XXX_PC104IO16 (p2)
#define BASE_IRQ 64
#endif

#ifdef CONFIG_MACH_TS72XX
#define __TS7XXX_PC104IO8	0x600ff800
#define __TS7XXX_PC104IO16	0x600ff800
#ifndef CONFIG_MACH_TS7350
#define TS7XXX_PC104IO8         0x11e00000
#define TS7XXX_PC104IO16 	0x21e00000
#endif
#ifdef CONFIG_MACH_TS7350
#define	TS7XXX_PC104IO8		0x600ff800
#define TS7XXX_PC104IO16	0x600ff800
#endif
#define BASE_IRQ 0 
#endif

#define DEFMEM          0x0
#define DEFIRQ          0
#define DEFIOBASE       0x0


/* ---- No user-serviceable parts below ---- */

static int ax_probe(struct net_device *dev);
static int ax_open(struct net_device *dev);
static int ax_close(struct net_device *dev);
static void ax_reset(struct net_device *dev);
static void ax_get_hdr(struct net_device *dev, struct ax_pkt_hdr *hdr, int ring_page);
static void ax_block_input(struct net_device *dev, int count, struct sk_buff *skb, int ring_offset);
static void ax_block_output(struct net_device *dev, const int count, const unsigned char *buf, const int start_page);
static int ethdev_init(struct net_device *dev);
static void ax_init(struct net_device *dev, int startp);
static irqreturn_t ax_interrupt(int irq, void *dev_id, struct pt_regs * regs);
static void ax_tx_intr(struct net_device *dev);
static void ax_tx_err(struct net_device *dev);
static void ax_receive(struct net_device *dev);
static void ax_rx_overrun(struct net_device *dev);
static void ax_trigger_send(struct net_device *dev, unsigned int length, int start_page);
static void set_multicast_list(struct net_device *dev);
static void do_set_multicast_list(struct net_device *dev);
static void ax_watchdog(unsigned long arg);
static struct net_device_stats *get_stats(struct net_device *dev);
static int ax_start_xmit(struct sk_buff *skb, struct net_device *dev);


#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
static void ax_vlan_rx_register(struct net_device *dev, struct vlan_group *grp);
static void ax_vlan_rx_kill_vid(struct net_device *dev, unsigned short vid);
#endif


static int mdio_read(struct net_device *dev, int phy_id, int loc);
static void mdio_write(struct net_device *dev, int phy_id, int loc, int value);
void fool_init (struct net_device *dev) {}

#define	PRINTK(flag, args...) if (flag & DEBUG_FLAGS) printk(args)
#define USE_MEMCPY      0
#define IRQ_EINT11 0

#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
#define AX88796_VLAN_TAG_USED 1
#else
#define AX88796_VLAN_TAG_USED 0
#endif

struct net_device *dev_ax;
static unsigned int media = 0;

#define DRV_NAME        "AX88796B"
static int mem = DEFMEM;
static int irq = DEFIRQ;
static int iobase = DEFIOBASE;

module_param(mem, int, 0);
module_param(irq, int, 0);
module_param(media, int, 0);
module_param(iobase, int, 0);

MODULE_PARM_DESC(mem, "MEMORY base address(es),required");
MODULE_PARM_DESC(irq, "IRQ number(s)");
MODULE_PARM_DESC(media, "Media Mode(0=auto, 1=100full, 2=100half, 3=10full, 4=10half)");

MODULE_DESCRIPTION("ASIX AX88796B Fast Ethernet driver");
MODULE_LICENSE("GPL");


static const struct net_device_ops ax_NetDevOps =
{
   .ndo_init = ax_probe,   
   .ndo_open = ax_open,
   .ndo_stop = ax_close,
   .ndo_tx_timeout = NULL,
   .ndo_start_xmit = ax_start_xmit,
   .ndo_get_stats = get_stats,
   .ndo_set_multicast_list = &set_multicast_list
};


int __init ax_init_module(void)
{
   int ret;

#ifdef MODULE
	struct net_device *dev = alloc_netdev(sizeof(struct ax_device), "eth%d", fool_init);
#else
	struct net_device *dev = alloc_netdev(sizeof(struct ax_device), "eth1", fool_init);
	netdev_boot_setup_check(dev);
#endif
      
	dev->irq = irq;		
   dev->base_addr = mem;
   dev->netdev_ops = &ax_NetDevOps;

	if((ret = register_netdev(dev)) == 0) {
	  dev_ax = dev;	  	  
	  return 0;
	}

	if (dev->base_addr != 0) {
	  PRINTK(WARNING_MSG, "AX88796B: No AX88796B card found at memory = 0x%08lX, error = %d\n", dev->base_addr, ret);
	}
	else {
	  PRINTK(WARNING_MSG, "AX88796B: You must supply \"mem=0xNNNNNNN\" value(s) for AX88796B. Error = %d\n", ret);
	}

	free_netdev(dev);
	return -ENXIO;
}



/*
 * ----------------------------------------------------------------------------
 * Function Name: cleanup_module
 * Purpose: 
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
  
void __exit ax_cleanup_module(void)
{
	struct net_device *dev = dev_ax;
	struct ax_device *ax_local = (struct ax_device *)netdev_priv(dev);
	
	if (ax_local) {
  	  iounmap((void *)ax_local->membase);
	  iounmap((void *)ax_local->membase16);	  	  
	  unregister_netdev(dev);
	}
	free_netdev(dev);			
}


module_init(ax_init_module);
module_exit(ax_cleanup_module);


/*
 * ----------------------------------------------------------------------------
 * Function Name: ax_probe
 * Purpose: 
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int ax_probe(struct net_device *dev)
{
	int i;
	unsigned char SA_prom[32];
	int wordlength = 0;
	int start_page, stop_page;
	int reg0, ret;
	unsigned long base_addr;
	void *address;
	void *address16;	
	int io8base, io16base;
	struct ax_device *ax_local;
	unsigned long *fpga;

	PRINTK(INIT_MSG,  "AX88796B: probe start...\n");
	
#if defined(CONFIG_MACH_TS78XX)	
	// Make sure we have the PC-104 bus...
	fpga = ioremap_nocache(0xE8000000, 4096);
	fpga[0x0C / 4] |= (1<<11);   // Set the TS Special ISA pinout enable bit
	fpga[0x30 / 4] = 0x55555555;
	fpga[0x34 / 4] = 0x55555555;
	fpga[0x38 / 4] = 0x00055555;
	fpga[0x3C / 4] = 0x00055555;
	iounmap(fpga);
#endif

#ifdef MODULE
	if(dev->base_addr == 0 ) {	   
	   unsigned long p1 = (unsigned long)ioremap_nocache(__TS7XXX_PC104IO8,  NE_IO_EXTENT);
	   unsigned long p2 = (unsigned long)ioremap_nocache(__TS7XXX_PC104IO16, NE_IO_EXTENT);
	   
		if ( (inb((unsigned long)TS7XXX_PC104IO8|0x100)&0xF) == 0x5 && 
		  (inb((unsigned long)TS7XXX_PC104IO8|0x104)&0xF) == 0x1 ) {
	          dev->base_addr = 0x200;
		  if ((inb((unsigned long)TS7XXX_PC104IO8|0x108)&0xE) == 0x8) dev->irq = 7;
		  else if ((inb((unsigned long)TS7XXX_PC104IO8|0x108)&0xE) == 0x4) dev->irq = 6;
		  else if ((inb((unsigned long)TS7XXX_PC104IO8|0x108)&0xE) == 0x2) dev->irq = 5;		  
		  
		} else if ( (inb((unsigned long)TS7XXX_PC104IO8|0x110)&0xF) == 0x5 && 
		  (inb((unsigned long)TS7XXX_PC104IO8|0x114)&0xF) == 0x1 ) {
	          dev->base_addr = 0x240;
		  if ((inb((unsigned long)TS7XXX_PC104IO8|0x118)&0xE) == 0x8) dev->irq = 7;
		  else if ((inb((unsigned long)TS7XXX_PC104IO8|0x118)&0xE) == 0x4) dev->irq = 6;
		  else if ((inb((unsigned long)TS7XXX_PC104IO8|0x118)&0xE) == 0x2) dev->irq = 5;		  

		} else if ( (inb((unsigned long)TS7XXX_PC104IO8|0x120)&0xF) == 0x5 && 
		  (inb((unsigned long)TS7XXX_PC104IO8|0x124)&0xF) == 0x1 ) {
	          dev->base_addr = 0x300;
		  if ((inb((unsigned long)TS7XXX_PC104IO8|0x128)&0xE) == 0x8) dev->irq = 7;
		  else if ((inb((unsigned long)TS7XXX_PC104IO8|0x128)&0xE) == 0x4) dev->irq = 6;
		  else if ((inb((unsigned long)TS7XXX_PC104IO8|0x128)&0xE) == 0x2) dev->irq = 5;		  
		  
		} else if ( (inb((unsigned long)TS7XXX_PC104IO8|0x130)&0xF) == 0x5 && 
		  (inb((unsigned long)TS7XXX_PC104IO8|0x134)&0xF) == 0x1 ) {
	          dev->base_addr = 0x340;
		  if ((inb((unsigned long)TS7XXX_PC104IO8|0x138)&0xE) == 0x8) dev->irq = 7;
		  else if ((inb((unsigned long)TS7XXX_PC104IO8|0x138)&0xE) == 0x4) dev->irq = 6;
		  else if ((inb((unsigned long)TS7XXX_PC104IO8|0x138)&0xE) == 0x2) dev->irq = 5;		  		  
		}
		
		iounmap((void*)p1);
		iounmap((void*)p2);  
	}
#endif
   
   if (iobase != 0) {
      io8base = iobase;
      io16base = iobase;
   } else {
      io8base = __TS7XXX_PC104IO8;
      io16base = __TS7XXX_PC104IO16;
   }

	base_addr = dev->base_addr;

#if defined(CONFIG_MACH_TS78XX)
	dev->irq += BASE_IRQ;
#elif defined(CONFIG_MACH_TS72XX)
	if (dev->irq == 7) dev->irq = 40;
	else if (dev->irq == 6) dev->irq = 33;
	else if (dev->irq == 5) dev->irq = 22;
#endif

	/*
	if (check_mem_region(io8base + base_addr, NE_IO_EXTENT))
                return -ENODEV;
        if (!request_mem_region(io8base + base_addr, NE_IO_EXTENT, DRV_NAME))
                return -EBUSY;
	*/	
	
   address = (void *)ioremap_nocache(io8base + base_addr, NE_IO_EXTENT);
 	address16 = (void *)ioremap_nocache(io16base + base_addr, NE_IO_EXTENT);	
	 	 	 	
	if (!address) {
		PRINTK(ERROR_MSG, "AX88796B: Unable to remap memory 0x%08lX\n", (unsigned long)address);
		return -EBUSY;
	}

	reg0 = inb((unsigned long)address);
		
	if (reg0 == 0xFF) {
		ret = -ENODEV;
		goto err_out;
	}

	/* Do a preliminary verification that we have a 8390. */
	{
		int regd;
		outb(E8390_NODMA+E8390_PAGE1+E8390_STOP, ((unsigned long)address) + E8390_CMD);
		regd = inb((unsigned long)address + EN0_COUNTER0);
		outb(0xff, ((unsigned long)address) + EN0_COUNTER0);
		outb(E8390_NODMA+E8390_PAGE0, ((unsigned long)address) + E8390_CMD);
		inb((unsigned long)address + EN0_COUNTER0); // Clear the counter by reading. 
		if (inb((unsigned long)address + EN0_COUNTER0) != 0) {
			outb(reg0, ((unsigned long)address));
			outb(regd, ((unsigned long)address) + EN0_COUNTER0);	// Restore the old values. 
			ret = -ENODEV;
			goto err_out;
		}
	}

	//PRINTK(DRIVER_MSG, "%s", version1);
	//PRINTK(DRIVER_MSG, "%s", version2);

	/* Reset card. Who knows what dain-bramaged state it was left in. */
	{
		unsigned long reset_start_time = jiffies;

		/* DON'T change these to inb/outb or reset will fail on clones. */
		outb(inb((unsigned long)address + EN0_RESET), ((unsigned long)address) + EN0_RESET);
		while ((inb((unsigned long)address + EN0_ISR) & ENISR_RESET) == 0)  {
			if (jiffies - reset_start_time > 2*HZ/100) {
					PRINTK(ERROR_MSG, "AX88796B: not found (no reset ack).\n");
					ret = -ENODEV;
					goto err_out;
			}
		}
		outb(0xff, ((unsigned long)address) + EN0_ISR);		/* Ack all intr. */
	}

	/* Read the 16 bytes of station address PROM. */
	{
		struct {unsigned char value, offset; } program_seq[] =
		{
			{E8390_NODMA+E8390_PAGE0+E8390_STOP, E8390_CMD}, /* Select page 0*/
			{0x48,	EN0_DCFG},	/* Set byte-wide (0x48) access. */
			{0x00,	EN0_RCNTLO},	/* Clear the count regs. */
			{0x00,	EN0_RCNTHI},
			{0x00,	EN0_IMR},	/* Mask completion irq. */
			{0xFF,	EN0_ISR},
			{E8390_RXOFF, EN0_RXCR},	/* 0x20  Set to monitor */
			{E8390_TXOFF, EN0_TXCR},	/* 0x02  and loopback mode. */
			{32,	EN0_RCNTLO},
			{0x00,	EN0_RCNTHI},
			{0x00,	EN0_RSARLO},	/* DMA starting at 0x0000. */
			{0x00,	EN0_RSARHI},
			{E8390_RREAD+E8390_START, E8390_CMD},
		};

		for (i = 0; i < sizeof(program_seq)/sizeof(program_seq[0]); i++)
			outb(program_seq[i].value, ((unsigned long)address) + program_seq[i].offset);
	}

	for(i = 0; i < 32; i+=2) {
		SA_prom[i] = inb((unsigned long)address + EN0_DATAPORT);
		SA_prom[i+1] = inb((unsigned long)address + EN0_DATAPORT);
		if (SA_prom[i] != SA_prom[i+1])
			wordlength = 1;
	}

	for (i = 0; i < 16; i++)
		SA_prom[i] = SA_prom[i+i];
	outb(0x49, ((unsigned long)address) + EN0_DCFG);
	start_page = NESM_START_PG;
	stop_page = NESM_STOP_PG;

	if(dev->irq == 0)
		dev->irq =IRQ_EINT11;

	/* Fill in 8390 specific dev fields. */
	if (ethdev_init(dev))
	{	  	   
     PRINTK (ERROR_MSG, "ethdev init failed\n");
     ret = -ENOMEM;
     goto err_out;
	}

	ax_local = (struct ax_device *)netdev_priv(dev);
		
	/* Support No EEPROM */
        /* 
	{
		SA_prom[0] = 0x12;
		SA_prom[1] = 0x56;
		SA_prom[2] = 0x3f;
		SA_prom[3] = 0xa9;
		SA_prom[4] = 0x78;
		SA_prom[5] = 0xb5;
	}
        */

	PRINTK(DRIVER_MSG, "%s: MAC ADDRESS ",DRV_NAME);
	for(i = 0; i < ETHER_ADDR_LEN; i++) {
		PRINTK(DRIVER_MSG, " %2.2x", SA_prom[i]);
		dev->dev_addr[i] = SA_prom[i];
	}

	PRINTK(DRIVER_MSG, "\n%s: %s found at 0x%08lX, using IRQ %d.\n",
		dev->name, DRV_NAME, dev->base_addr, dev->irq);

	ax_local->name = DRV_NAME;
	ax_local->membase = address;
	ax_local->membase16 = address16;
	ax_local->tx_start_page = NESM_START_PG;
	ax_local->rx_start_page = NESM_RX_START_PG;
	ax_local->stop_page = NESM_STOP_PG;

	ax_local->media = media;

	dev->netdev_ops = &ax_NetDevOps;
		
#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
	dev->features |= NETIF_F_HW_VLAN_TX | NETIF_F_HW_VLAN_RX;
	dev->vlan_rx_register = ax_vlan_rx_register;
	dev->vlan_rx_kill_vid = ax_vlan_rx_kill_vid;
#endif
	ax_init(dev, 0);
	PRINTK(INIT_MSG,  "AX88796B: probe end...\n");
	return 0;

err_out:
   if (address) 
      iounmap(address);
	if (address16) 
	   iounmap(address16);
			
	PRINTK(INIT_MSG, "AX88796B: probe error...\n");
	return ret;
}



/*
 * ----------------------------------------------------------------------------
 * Function Name: ax_reset
 * Purpose: 
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void ax_reset(struct net_device *dev)
{
    struct ax_device *ax_local = (struct ax_device *) netdev_priv(dev);
	void *ax_base = ax_local->membase;
	unsigned long reset_start_time = jiffies;

	/* DON'T change these to inb/outb or reset will fail on clones. */
	outb(inb((unsigned long)ax_base + EN0_RESET), ((unsigned long)ax_base) + EN0_RESET);

	ax_local->dmaing = 0;

	/* This check _should_not_ be necessary, omit eventually. */
	while ((inb((unsigned long)ax_base+EN0_ISR) & ENISR_RESET) == 0)
		if (jiffies - reset_start_time > 2*HZ/100) {
			PRINTK(ERROR_MSG, "%s: ax_reset() did not complete.\n", dev->name);
			break;
		}
	outb(ENISR_RESET, ((unsigned long)ax_base) + EN0_ISR);	/* Ack intr. */
}



/*
 * ----------------------------------------------------------------------------
 * Function Name: ax_get_hdr
 * Purpose: Grab the 796b specific header
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void ax_get_hdr(struct net_device *dev, struct ax_pkt_hdr *hdr, int ring_page)
{
    struct ax_device *ax_local = (struct ax_device *) netdev_priv(dev);
	void *ax_base = ax_local->membase;
	u16 tmp;

	/* This *shouldn't* happen. If it does, it's the last thing you'll see */
		
	if (ax_local->dmaing)
	{
		PRINTK(ERROR_MSG, "%s: DMAing conflict in ne_get_8390_hdr "
			"[DMAstat:%d][irqlock:%d].\n",
			dev->name, ax_local->dmaing, ax_local->irqlock);
		return;
	}

	ax_local->dmaing |= 0x01;
	outb(E8390_NODMA+E8390_PAGE0+E8390_START, ((unsigned long)ax_base)+ E8390_CMD);
	outb(sizeof(struct ax_pkt_hdr), ((unsigned long)ax_base) + EN0_RCNTLO);
	outb(0, ((unsigned long)ax_base) + EN0_RCNTHI);
	outb(0, ((unsigned long)ax_base) + EN0_RSARLO);		/* On page boundary */
	outb(ring_page, ((unsigned long)ax_base) + EN0_RSARHI);
	outb(E8390_RREAD+E8390_START, ((unsigned long)ax_base) + E8390_CMD);
			
	while (( inb((unsigned long)ax_base+EN0_SR) & 0x20) ==0);

	for(tmp=0; tmp < (sizeof(struct ax_pkt_hdr)>>1); tmp++) {
      *((u16 *)hdr + tmp)= inw( ((unsigned long)ax_local->membase16) + EN0_DATAPORT );
                  
	}

	outb(ENISR_RDC, ((unsigned long)ax_base) + EN0_ISR);	/* Ack intr. */
	ax_local->dmaing = 0;

	le16_to_cpus(&hdr->count);
}


/*
 * ----------------------------------------------------------------------------
 * Function Name: ax_block_input
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void ax_block_input(struct net_device *dev, int count, struct sk_buff *skb, int ring_offset)
{
    struct ax_device *ax_local = (struct ax_device *) netdev_priv(dev);
	void *ax_base = ax_local->membase;
	u8 *vlan_buf;

	/* This *shouldn't* happen. If it does, it's the last thing you'll see */
	if (ax_local->dmaing)
	{
		PRINTK(ERROR_MSG, "%s: DMAing conflict in ne_block_input "
			"[DMAstat:%d][irqlock:%d].\n",
			dev->name, ax_local->dmaing, ax_local->irqlock);
		return;
	}

	ax_local->dmaing |= 0x01;
	outb(E8390_NODMA+E8390_PAGE0+E8390_START, ((unsigned long)ax_base) + E8390_CMD);
	outb(count & 0xff, ((unsigned long)ax_base) + EN0_RCNTLO);
	outb(count >> 8, ((unsigned long)ax_base) + EN0_RCNTHI);
	outb(ring_offset & 0xff, ((unsigned long)ax_base) + EN0_RSARLO);
	outb(ring_offset >> 8, ((unsigned long)ax_base) + EN0_RSARHI);
	outb(E8390_RREAD+E8390_START, ((unsigned long)ax_base) + E8390_CMD);

	while (( inb((unsigned long)ax_base+EN0_SR) & 0x20) ==0);

	vlan_buf = ax_local->packet_buf;

#if (USE_MEMCPY == 1)
	{
		u16	pkt_len_align;
		if ( ((count % 4)) != 0 )
			pkt_len_align = count + 4 - (count % 4);
		else
			pkt_len_align = count;
	
		memcpy(vlan_buf, (ax_base+EN0_DATA_ADDR), pkt_len_align);
	
#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
		if(ax_local->vlgrp) {
			memcpy((void *)(skb->data), vlan_buf, 12);
			memcpy((void *)(skb->data+12), vlan_buf, (count-4-12));
		}
		else
#endif
			memcpy((void *)(skb->data), vlan_buf, count);
	}
#else
	{
		u8 *buf = skb->data;
		u16 i,tmp;
		tmp = count >> 1;

	#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
		if(ax_local->vlgrp) {
			for(i=0; i < tmp; i++) {
				*((u16 *)vlan_buf + i) = inw( ax_local->membase16 + EN0_DATA_ADDR);
			}
			if (count & 0x01)
				vlan_buf[count-1] = inb((unsigned long)ax_base + EN0_DATA_ADDR);
			memcpy((void *)(skb->data), vlan_buf, 12);
			memcpy((void *)(skb->data+12), vlan_buf, (count-4-12));
		}
		else
	#endif
		{
			for(i=0; i < tmp; i++) {
				*((u16 *)buf + i) = inw(((unsigned long)ax_local->membase16) + EN0_DATA_ADDR);
			}
			if (count & 0x01)
				buf[count-1] = inb(((unsigned long)ax_local->membase16) + EN0_DATA_ADDR);
		}
	}
#endif

	outb(ENISR_RDC, ((unsigned long)ax_base) + EN0_ISR);	/* Ack intr. */
	ax_local->dmaing = 0;
}


/*
 * ----------------------------------------------------------------------------
 * Function Name: ax_block_output
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void ax_block_output(struct net_device *dev, int count, const unsigned char *buf, const int start_page)
{
    struct ax_device *ax_local = (struct ax_device *) netdev_priv(dev);
	void *ax_base = ax_local->membase;
	unsigned long dma_start;

	/* This *shouldn't* happen. If it does, it's the last thing you'll see */
	if (ax_local->dmaing)
	{
		PRINTK(ERROR_MSG, "%s: DMAing conflict in ne_block_output."
			"[DMAstat:%d][irqlock:%d]\n",
			dev->name, ax_local->dmaing, ax_local->irqlock);
		return;
	}

	ax_local->dmaing |= 0x01;
	/* We should already be in page 0, but to be safe... */
	outb(E8390_PAGE0+E8390_START+E8390_NODMA, ((unsigned long)ax_base) + E8390_CMD);
	outb(ENISR_RDC, ((unsigned long)ax_base) + EN0_ISR);
	/* Now the normal output. */
	outb(count & 0xff, ((unsigned long)ax_base) + EN0_RCNTLO);
	outb(count >> 8,   ((unsigned long)ax_base) + EN0_RCNTHI);
	outb(0x00, ((unsigned long)ax_base) + EN0_RSARLO);
	outb(start_page, ((unsigned long)ax_base) + EN0_RSARHI);
	outb(E8390_RWRITE+E8390_START, ((unsigned long)ax_base) + E8390_CMD);

#if (USE_MEMCPY == 1)
	{
		memcpy((ax_base+EN0_DATA_ADDR), buf, (count+count%8));
	}
#else
	{
		u16 data, i;
		for (i=0; i<count; i+=2)
		{
			data = (u16)*(buf+i) + ( (u16)*(buf+i+1) << 8 );
			outw(data, ((unsigned long)ax_local->membase16) + EN0_DATAPORT);
		}
	}
#endif
	dma_start = jiffies;
	while ((inb((unsigned long)ax_base + EN0_ISR) & 0x40) == 0) {
		if (jiffies - dma_start > 2*HZ/100) {		/* 20ms */
			PRINTK(ERROR_MSG, "%s: timeout waiting for Tx RDC.\n", dev->name);
			ax_reset(dev);
			ax_init(dev,1);
			break;
		}
	}
	outb(ENISR_RDC, ((unsigned long)ax_base) + EN0_ISR);	/* Ack intr. */
	ax_local->dmaing = 0;
	return;
}


/*
 * ----------------------------------------------------------------------------
 * Function Name: ax_open
 * Purpose: Open/initialize 796b
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int ax_open(struct net_device *dev)
{
	unsigned long flags;
	struct ax_device *ax_local = (struct ax_device *) netdev_priv(dev);
	void *membase = ax_local->membase;
	int ret=0;

	PRINTK(DEBUG_MSG, "AX88796B: ax88796B ei_open beginning ..........\n");
	PRINTK(DEBUG_MSG, "AX88796B: membase %p\n\r", membase);

	ret = request_irq(dev->irq, (irq_handler_t)&ax_interrupt, IRQF_SHARED/*SA_INTERRUPT*/, dev->name, dev);

	if (ret) {
		PRINTK (ERROR_MSG, "%s: unable to get IRQ %d (errno=%d).\n",dev->name, dev->irq, ret);
		return -ENXIO;
	}

	PRINTK(DEBUG_MSG, "AX88796B: Request IRQ success !!\n\r");

	/* This can't happen unless somebody forgot to call ethdev_init(). */
	if (ax_local == NULL) 
	{
		PRINTK(ERROR_MSG, "%s: ei_open passed a non-existent device!\n", dev->name);
		return -ENXIO;
	}

   ax_local->packet_buf = kmalloc(1536, GFP_KERNEL);
	if(!ax_local->packet_buf)
		 return -ENOMEM;

	 memset(ax_local->packet_buf,0,1536);
	 
	dev->watchdog_timeo = 0;

	init_timer(&ax_local->watchdog);
	ax_local->watchdog.function = &ax_watchdog;
	ax_local->watchdog.expires = jiffies + AX88796_WATCHDOG_PERIOD;
	ax_local->watchdog.data = (unsigned long) dev;
	add_timer(&ax_local->watchdog);
      
    spin_lock_irqsave(&ax_local->page_lock, flags);
	ax_reset(dev);
	ax_init(dev, 1);
    spin_unlock_irqrestore(&ax_local->page_lock, flags);
        netif_start_queue(dev);
	ax_local->irqlock = 0;
	PRINTK(DEBUG_MSG, "AX88796B: ax88796B ei_open end ..........\n");
	return 0;
}


/*
 * ----------------------------------------------------------------------------
 * Function Name: ax_close
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int ax_close(struct net_device *dev)
{
   struct ax_device *ax_local = (struct ax_device *) netdev_priv(dev);
   unsigned long flags;
   PRINTK(DEBUG_MSG, "AX88796B: ax88796B ei_close beginning ..........\n");
   del_timer(&ax_local->watchdog);
   spin_lock_irqsave(&ax_local->page_lock, flags);
   ax_init(dev, 0);
   spin_unlock_irqrestore(&ax_local->page_lock, flags);
   free_irq(dev->irq, dev);
   netif_stop_queue(dev);
   PRINTK(DEBUG_MSG, "AX88796B: ax88796B ei_close end ..........\n");
   return 0;
}


/*
 * ----------------------------------------------------------------------------
 * Function Name: add_vlan
 * Purpose: Add Vlan tab for packet transmission
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
static void add_vlan(struct net_device *dev, struct sk_buff *skb, int length)
{
	struct ax_device *ax_local = (struct ax_device *) netdev_priv(dev);
	u8 *vlan_buf = ax_local->packet_buf;
	u16 vlan_tag = ax_local->vid;

	memcpy(&vlan_buf[0], skb->data, 12);
	vlan_buf[12]=0x81;
	vlan_buf[13]=0x00;
	vlan_buf[14]=(vlan_tag&0xf00)>>8;
	vlan_buf[15]=vlan_tag&0xff;
	memcpy(&vlan_buf[16],&skb->data[12],(length-12-4));
	return;
}
#endif


/*
 * ----------------------------------------------------------------------------
 * Function Name: ax_start_xmit
 * Purpose: begin packet transmission
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static int ax_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct ax_device *ax_local = (struct ax_device *) netdev_priv(dev);
	void *ax_base = ax_local->membase;
	int send_length;
	unsigned long flags;
	u8 ctepr=0, free_pages=0, need_pages;

#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
	u8 *vlan_buf = ax_local->packet_buf;
	u16 vlan_tag = 0;
#endif

	/* check for link status */
	if (!(inb((unsigned long)ax_base + EN0_SR) & 0x01)) {
		dev_kfree_skb (skb);
		PRINTK(DEBUG_MSG, "drop transmit packet\n\r");
		return 0;
	}

	send_length = ETH_ZLEN < skb->len ? skb->len : ETH_ZLEN;

#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
	if (vlan_tx_tag_present(skb)) {
		vlan_tag = vlan_tx_tag_get(skb);
		outb((u8)(vlan_tag&0xff), ((unsigned long)ax_base) + EN0_VID0);
		outb((u8)((vlan_tag&0xf00)>>8), ((unsigned long)ax_base) + EN0_VID1);
		ax_local->vid = vlan_tag;
	}
#endif
	spin_lock_irqsave(&ax_local->page_lock, flags);
		outb(0x00, ((unsigned long)ax_base) + EN0_IMR);
	spin_unlock_irqrestore(&ax_local->page_lock, flags);

	spin_lock(&ax_local->page_lock);
	ax_local->irqlock = 1;

	need_pages = (send_length -1)/256 +1;
	ctepr = inb((unsigned long)ax_base + EN0_CTEPR) & 0x7f;
		
	if(ctepr == 0) {
		if(ax_local->tx_curr_page == ax_local->tx_start_page && ax_local->tx_prev_ctepr == 0)
			free_pages = TX_PAGES;
		else
			free_pages = ax_local->tx_stop_page - ax_local->tx_curr_page;
	}
	else if(ctepr < ax_local->tx_curr_page - 1) {
		free_pages = ax_local->tx_stop_page - ax_local->tx_curr_page + 
					 ctepr - ax_local->tx_start_page + 1;
	}
	else if(ctepr > ax_local->tx_curr_page - 1) {
		free_pages = ctepr + 1 - ax_local->tx_curr_page;
	}
	else if(ctepr == ax_local->tx_curr_page - 1) {
		if(ax_local->tx_full)
			free_pages = 0;
		else
			free_pages = TX_PAGES;
	}		

	if(free_pages < need_pages) {
		PRINTK(DEBUG_MSG, "free_pages < need_pages\n\r");
		netif_stop_queue(dev);
		ax_local->tx_full = 1;
		ax_local->irqlock = 0;	
		spin_unlock(&ax_local->page_lock);
		spin_lock_irqsave(&ax_local->page_lock, flags);
		outb(ENISR_ALL, ((unsigned long)ax_base) + EN0_IMR);
		spin_unlock_irqrestore(&ax_local->page_lock, flags);
		return 1;
	}
#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
	if (vlan_tag) {
		send_length +=4;
		add_vlan(dev,skb, send_length);
		ax_block_output(dev, send_length, vlan_buf, ax_local->tx_curr_page);
	}
	else
#endif
	ax_block_output(dev, send_length, skb->data, ax_local->tx_curr_page);
	ax_trigger_send(dev, send_length, ax_local->tx_curr_page);
	if(free_pages == need_pages) {
		netif_stop_queue(dev);
		ax_local->tx_full = 1;
	}
	ax_local->tx_prev_ctepr = ctepr;
	ax_local->tx_curr_page = ax_local->tx_curr_page + need_pages < ax_local->tx_stop_page ? 
	ax_local->tx_curr_page + need_pages : 
	need_pages - (ax_local->tx_stop_page - ax_local->tx_curr_page) + ax_local->tx_start_page;

	dev_kfree_skb (skb);
	dev->trans_start = jiffies;
	ax_local->stat.tx_bytes += send_length;

	ax_local->irqlock = 0;	
	spin_unlock(&ax_local->page_lock);
	spin_lock_irqsave(&ax_local->page_lock, flags);
	outb(ENISR_ALL, ((unsigned long)ax_base) + EN0_IMR);
	spin_unlock_irqrestore(&ax_local->page_lock, flags);
	return 0;
}


/*
 * ----------------------------------------------------------------------------
 * Function Name: ax_tx_intr
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void ax_tx_intr(struct net_device *dev)
{
    struct ax_device *ax_local = (struct ax_device *) netdev_priv(dev);
	void *ax_base = ax_local->membase;
	int status = inb((unsigned long)ax_base + EN0_TSR);

	ax_local->tx_full = 0;
	if(netif_queue_stopped(dev))
		netif_wake_queue(dev);

	/* Minimize Tx latency: update the statistics after we restart TXing. */
	if (status & ENTSR_COL)
		ax_local->stat.collisions++;
	if (status & ENTSR_PTX)
		ax_local->stat.tx_packets++;
	else 
	{
		ax_local->stat.tx_errors++;
		if (status & ENTSR_ABT) 
		{
			ax_local->stat.tx_aborted_errors++;
			ax_local->stat.collisions += 16;
		}
		if (status & ENTSR_CRS) 
			ax_local->stat.tx_carrier_errors++;
		if (status & ENTSR_FU) 
			ax_local->stat.tx_fifo_errors++;
		if (status & ENTSR_CDH)
			ax_local->stat.tx_heartbeat_errors++;
		if (status & ENTSR_OWC)
			ax_local->stat.tx_window_errors++;
	}
}


/*
 * ----------------------------------------------------------------------------
 * Function Name: ax_interrupt
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static irqreturn_t ax_interrupt(int irq, void *dev_id, struct pt_regs * regs)
{
	struct net_device *dev = dev_id;
	int interrupts;
       struct ax_device *ax_local = (struct ax_device *) netdev_priv(dev);
	void *ax_base = ax_local->membase;
	unsigned long flags;

	if (dev == NULL) 
	{
		PRINTK (ERROR_MSG, "net_interrupt(): irq %d for unknown device.\n", irq);
		return 0;
	}
	outb(E8390_NODMA+E8390_PAGE0, ((unsigned long)ax_base) + E8390_CMD);

	spin_lock_irqsave(&ax_local->page_lock, flags);
		outb(0x00, ((unsigned long)ax_base) + EN0_IMR);
	spin_unlock_irqrestore(&ax_local->page_lock, flags);

	if (ax_local->irqlock) {
		return 0;
	}

	spin_lock(&ax_local->page_lock);
	
	while (1)
	{
                interrupts = inb((unsigned long)ax_base + EN0_ISR);
		if(interrupts == 0) 
			break;
                
		 
		outb(interrupts, ((unsigned long)ax_base) + EN0_ISR); /* Ack the interrupts */

		if (interrupts & ENISR_TX)
			ax_tx_intr(dev);

		if (interrupts & ENISR_OVER)
			ax_rx_overrun(dev);
		
		if (interrupts & (ENISR_RX+ENISR_RX_ERR))
			ax_receive(dev);

		if (interrupts & ENISR_TX_ERR)
			ax_tx_err(dev);

		if (interrupts & ENISR_COUNTERS) 
		{   
			ax_local->stat.rx_frame_errors += inb((unsigned long)ax_base + EN0_COUNTER0);
			ax_local->stat.rx_crc_errors   += inb((unsigned long)ax_base + EN0_COUNTER1);
			ax_local->stat.rx_missed_errors+= inb((unsigned long)ax_base + EN0_COUNTER2); 
			outb(ENISR_COUNTERS, ((unsigned long)ax_base) + EN0_ISR); /* Ack intr. */
		}

		if (interrupts & ENISR_RDC)
			outb(ENISR_RDC, ((unsigned long)ax_base) + EN0_ISR);
		
		outb(E8390_NODMA+E8390_PAGE0+E8390_START, ((unsigned long)ax_base) + E8390_CMD);
	}

	spin_unlock(&ax_local->page_lock);
	spin_lock_irqsave(&ax_local->page_lock, flags);
		outb(ENISR_ALL, ((unsigned long)ax_base) + EN0_IMR);
	spin_unlock_irqrestore(&ax_local->page_lock, flags);

	outb(E8390_NODMA+E8390_PAGE0+E8390_START, ((unsigned long)ax_base) + E8390_CMD);

	//return IRQ_RETVAL(0);
	return IRQ_HANDLED;
}


/*
 * ----------------------------------------------------------------------------
 * Function Name: ax_tx_err
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void ax_tx_err(struct net_device *dev)
{
    struct ax_device *ax_local = (struct ax_device *) netdev_priv(dev);
	void *ax_base = ax_local->membase;

	unsigned char txsr = inb((unsigned long)ax_base+EN0_TSR);
	unsigned char tx_was_aborted = txsr & (ENTSR_ABT+ENTSR_FU);

	if (tx_was_aborted)
		ax_tx_intr(dev);
	else 
	{
		ax_local->stat.tx_errors++;
		if (txsr & ENTSR_CRS) ax_local->stat.tx_carrier_errors++;
		if (txsr & ENTSR_CDH) ax_local->stat.tx_heartbeat_errors++;
		if (txsr & ENTSR_OWC) ax_local->stat.tx_window_errors++;
	}
}


/*
 * ----------------------------------------------------------------------------
 * Function Name: ax_receive
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void ax_receive(struct net_device *dev)
{
    struct ax_device *ax_local = (struct ax_device *) netdev_priv(dev);
	void *ax_base = ax_local->membase;
	unsigned char rxing_page, this_frame, next_frame;
	unsigned short current_offset;
	struct ax_pkt_hdr rx_frame;
	int num_rx_pages = ax_local->stop_page - ax_local->rx_start_page;
	while(1)
	{
		int pkt_len, pkt_stat;

		/* Get the rx page (incoming packet pointer). */
		outb(E8390_NODMA+E8390_PAGE1, ((unsigned long)ax_base) + E8390_CMD);
		rxing_page = inb((unsigned long)ax_base + EN1_CURPAG);
		outb(E8390_NODMA+E8390_PAGE0, ((unsigned long)ax_base) + E8390_CMD);

		
		/* Remove one frame from the ring.  Boundary is always a page behind. */
		this_frame = inb((unsigned long)ax_base + EN0_BOUNDARY) + 1;
		if (this_frame >= ax_local->stop_page)
			this_frame = ax_local->rx_start_page;

		if (this_frame != ax_local->current_page && (this_frame!=0x0 || rxing_page!=0xFF))
			PRINTK(RX_MSG, "%s: mismatched read page pointers %2x vs %2x.\n",
				   dev->name, this_frame, ax_local->current_page);
		
		if (this_frame == rxing_page) {	/* Read all the frames? */
			break;				/* Done for now */
		}
		current_offset = this_frame << 8;
		ax_get_hdr(dev, &rx_frame, this_frame);
		
		pkt_len = rx_frame.count - sizeof(struct ax_pkt_hdr);
		pkt_stat = rx_frame.status;
		next_frame = this_frame + 1 + ((pkt_len+4)>>8);
		
				
		/* Check for bogosity warned by 3c503 book: the status byte is never
		   written.  This happened a lot during testing! This code should be
		   cleaned up someday. */
		if (rx_frame.next != next_frame
			&& rx_frame.next != next_frame + 1
			&& rx_frame.next != next_frame - num_rx_pages
			&& rx_frame.next != next_frame + 1 - num_rx_pages) {
			ax_local->current_page = rxing_page;
			outb(ax_local->current_page-1, ((unsigned long)ax_base) + EN0_BOUNDARY);
			ax_local->stat.rx_errors++;
			PRINTK(ERROR_MSG, "error occurred! Drop this packet!!\n");
			
			PRINTK(ERROR_MSG, "%d %d %d %d, rx_frame.next=%d, next_frame=%d\n", 
			     rx_frame.next != next_frame,
			     rx_frame.next != next_frame + 1,
			     rx_frame.next != next_frame - num_rx_pages,
			     rx_frame.next != next_frame + 1 - num_rx_pages,
			     rx_frame.next, next_frame);
			
			continue;
		}
		

		if (pkt_len < 60  ||  pkt_len > 1518)
		{
			PRINTK(RX_MSG, "%s: bogus packet size: %d, status=%#2x nxpg=%#2x.\n",
					   dev->name, rx_frame.count, rx_frame.status,
					   rx_frame.next);
			ax_local->stat.rx_errors++;
			ax_local->stat.rx_length_errors++;
		}
		else if ((pkt_stat & 0x0F) == ENRSR_RXOK) 
		{
			struct sk_buff *skb;
			skb = dev_alloc_skb(pkt_len+2);
			if (skb == NULL)
			{
				printk("%s: Couldn't allocate a sk_buff of size %d.\n", dev->name, pkt_len);
				ax_local->stat.rx_dropped++;
				break;
			}
			skb_reserve(skb,2);	/* IP headers on 16 byte boundaries */
			skb->dev = dev;
			skb_put(skb, pkt_len);	/* Make room */
			ax_block_input(dev, pkt_len, skb, current_offset + sizeof(rx_frame));
			skb->protocol=eth_type_trans(skb,dev);
#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
			if(ax_local->vlgrp)
				vlan_hwaccel_rx(skb, ax_local->vlgrp, ax_local->vid);
			else
#endif
				netif_rx(skb);
			dev->last_rx = jiffies;
			ax_local->stat.rx_packets++;
			ax_local->stat.rx_bytes += pkt_len;
			if (pkt_stat & ENRSR_PHY)
				ax_local->stat.multicast++;
		}
		else 
		{
			PRINTK(ERROR_MSG, "%s: bogus packet: status=%#2x nxpg=%#2x size=%d\n",
					   dev->name, rx_frame.status, rx_frame.next, rx_frame.count);
			ax_local->stat.rx_errors++;
			/* NB: The NIC counts CRC, frame and missed errors. */
			if (pkt_stat & ENRSR_FO)
				ax_local->stat.rx_fifo_errors++;
		}
		next_frame = rx_frame.next;
		
		/* This _should_ never happen: it's here for avoiding bad clones. */
		if (next_frame >= ax_local->stop_page) {
			PRINTK(ERROR_MSG, "%s: next frame inconsistency, %#2x\n", dev->name, next_frame);
			next_frame = ax_local->rx_start_page;
		}
		ax_local->current_page = next_frame;
		outb(next_frame-1, ((unsigned long)ax_base) + EN0_BOUNDARY);
	}

	return;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax_rx_overrun
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void ax_rx_overrun(struct net_device *dev)
{
    struct ax_device *ax_local = (struct ax_device *) netdev_priv(dev);
	void *ax_base = ax_local->membase;
	unsigned char was_txing, must_resend = 0;

    
	/*
	 * Record whether a Tx was in progress and then issue the
	 * stop command.
	 */
	was_txing = inb((unsigned long)ax_base+E8390_CMD) & E8390_TRANS;
	outb(E8390_NODMA+E8390_PAGE0+E8390_STOP, ((unsigned long)ax_base) + E8390_CMD);

	PRINTK(RX_MSG, "%s: Receiver overrun.\n", dev->name);

	ax_local->stat.rx_over_errors++;

	udelay(2*1000);

	outb(0x00, ((unsigned long)ax_base) + EN0_RCNTLO);
	outb(0x00, ((unsigned long)ax_base) + EN0_RCNTHI);

	/*
	 * See if any Tx was interrupted or not. According to NS, this
	 * step is vital, and skipping it will cause no end of havoc.
	 */

	if (was_txing)
	{ 
		unsigned char tx_completed = inb((unsigned long)ax_base+EN0_ISR) & (ENISR_TX+ENISR_TX_ERR);
		if (!tx_completed)
			must_resend = 1;
	}

	/*
	 * Have to enter loopback mode and then restart the NIC before
	 * you are allowed to slurp packets up off the ring.
	 */
	outb(E8390_TXOFF, ((unsigned long)ax_base) + EN0_TXCR);
	outb(E8390_NODMA + E8390_PAGE0 + E8390_START, ((unsigned long)ax_base) + E8390_CMD);

	/*
	 * Clear the Rx ring of all the debris, and ack the interrupt.
	 */
	ax_receive(dev);

	/*
	 * Leave loopback mode, and resend any packet that got stopped.
	 */
	outb(E8390_TXCONFIG, ((unsigned long)ax_base) + EN0_TXCR); 
	if (must_resend)
    	outb(E8390_NODMA + E8390_PAGE0 + E8390_START + E8390_TRANS, ((unsigned long)ax_base) + E8390_CMD);

}


/*
 *	Collect the stats. This is called unlocked and from several contexts.
 */
static struct net_device_stats *get_stats(struct net_device *dev)
{
 	struct ax_device *ax_local = (struct ax_device *) netdev_priv(dev);
	void *ax_base = ax_local->membase; 
	unsigned long flags;

	/* If the card is stopped, just return the present stats. */
	if (!netif_running(dev))
		return &ax_local->stat;

	spin_lock_irqsave(&ax_local->page_lock,flags);
	/* Read the counter registers, assuming we are in page 0. */
	ax_local->stat.rx_frame_errors += inb((unsigned long)ax_base + EN0_COUNTER0);
	ax_local->stat.rx_crc_errors   += inb((unsigned long)ax_base + EN0_COUNTER1);
	ax_local->stat.rx_missed_errors+= inb((unsigned long)ax_base + EN0_COUNTER2);

	spin_unlock_irqrestore(&ax_local->page_lock, flags);
	return &ax_local->stat;
}


/*
 * ----------------------------------------------------------------------------
 * Function Name: make_mc_bits
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static inline void make_mc_bits(u8 *bits, struct net_device *dev)
{
	struct dev_mc_list *dmi;
	for (dmi=dev->mc_list; dmi; dmi=dmi->next) 
	{
		u32 crc;
		if (dmi->dmi_addrlen != ETH_ALEN) 
		{
			PRINTK(INIT_MSG, "%s: invalid multicast address length given.\n", dev->name);
			continue;
		}
		crc = ether_crc(ETH_ALEN, dmi->dmi_addr);
		/* 
		 * The 8390 uses the 6 most significant bits of the
		 * CRC to index the multicast table.
		 */
		bits[crc>>29] |= (1<<((crc>>26)&7));
	}
}


/*
 * ----------------------------------------------------------------------------
 * Function Name: do_set_multicast_list
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void do_set_multicast_list(struct net_device *dev)
{
 	struct ax_device *ax_local = (struct ax_device *) netdev_priv(dev);
	void *ax_base = ax_local->membase; 
	int i;
	if (!(dev->flags&(IFF_PROMISC|IFF_ALLMULTI))) 
	{
		memset(ax_local->mcfilter, 0, 8);
		if (dev->mc_list)
			make_mc_bits(ax_local->mcfilter, dev);
	}
	else
		memset(ax_local->mcfilter, 0xFF, 8);	/* mcast set to accept-all */
	 
	if (netif_running(dev))
		outb(E8390_RXCONFIG, ((unsigned long)ax_base) + EN0_RXCR);
	outb(E8390_NODMA + E8390_PAGE1, ((unsigned long)ax_base) + E8390_CMD);
	for(i = 0; i < 8; i++) 
	{
		outb(ax_local->mcfilter[i], ((unsigned long)ax_base) + EN1_MULT_SHIFT(i));
	}
	outb(E8390_NODMA + E8390_PAGE0, ((unsigned long)ax_base) + E8390_CMD);

  	if(dev->flags&IFF_PROMISC)
  		outb(E8390_RXCONFIG | 0x18, ((unsigned long)ax_base) + EN0_RXCR);
	else if(dev->flags&IFF_ALLMULTI || dev->mc_list) 
  		outb(E8390_RXCONFIG | 0x08, ((unsigned long)ax_base) + EN0_RXCR);
	
	else 
  		outb(E8390_RXCONFIG, ((unsigned long)ax_base) + EN0_RXCR);
 }


/*
 * ----------------------------------------------------------------------------
 * Function Name: set_multicast_list
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void set_multicast_list(struct net_device *dev)
{
	unsigned long flags;
	struct ax_device *ax_local = (struct ax_device*)netdev_priv(dev);
	
	spin_lock_irqsave(&ax_local->page_lock, flags);
	do_set_multicast_list(dev);
	spin_unlock_irqrestore(&ax_local->page_lock, flags);
}	


/*
 * ----------------------------------------------------------------------------
 * Function Name: ethdev_init
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
 
static int ethdev_init(struct net_device *dev)
{
   struct ax_device *ax_local = (struct ax_device *)netdev_priv(dev);
           	
	//memset(ax_local, 0, sizeof(*ax_local));	
	spin_lock_init(&ax_local->page_lock);	  
	ether_setup(dev);
        
	return 0;
}


/*
 * ----------------------------------------------------------------------------
 * Function Name: ax88796_PHY_init
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void ax88796_PHY_init(struct net_device *dev)
{
	struct ax_device *ax_local = (struct ax_device *) netdev_priv(dev);
	void *ax_base = ax_local->membase;
	u8 tmp_data;

	/* Enable AX88796B FOLW CONTROL */
	outb(ENFLOW_ENABLE, ((unsigned long)ax_base) + EN0_FLOW);

	/* Enable PHY PAUSE */
	mdio_write(dev,0x10,0x04,(mdio_read(dev,0x10,0x04) | 0x400));
	mdio_write(dev,0x10,0x00,0x1200);

	/* Enable AX88796B TQC */
	tmp_data = inb((unsigned long)ax_base+EN0_MCR);
	outb( tmp_data | ENTQC_ENABLE, ((unsigned long)ax_base) + EN0_MCR);

	/* Enable AX88796B Transmit Buffer Ring */
	outb(E8390_NODMA+E8390_PAGE3+E8390_STOP, ((unsigned long)ax_base) + E8390_CMD);
	outb(ENTBR_ENABLE, ((unsigned long)ax_base) + EN3_TBR);
	outb(E8390_NODMA+E8390_PAGE0+E8390_STOP, ((unsigned long)ax_base) + E8390_CMD);

	switch (ax_local->media) {
	default:
	case MEDIA_AUTO:
		PRINTK(DRIVER_MSG, "AX88796B: The media mode is autosense.\n");
		break;

	case MEDIA_100FULL:
		PRINTK(DRIVER_MSG, "AX88796B: The media mode is forced to 100full.\n");
		mdio_write(dev,0x10,0x00,0x2300);
		break;

	case MEDIA_100HALF:
		PRINTK(DRIVER_MSG, "AX88796B: The media mode is forced to 100half.\n");
		mdio_write(dev,0x10,0x00,0x2200);
		break;

	case MEDIA_10FULL:
		PRINTK(DRIVER_MSG, "AX88796B: The media mode is forced to 10full.\n");
		mdio_write(dev,0x10,0x00,0x0300);
		break;

	case MEDIA_10HALF:
		PRINTK(DRIVER_MSG, "AX88796B: The media mode is forced to 10half.\n");
		mdio_write(dev,0x10,0x00,0x0200);
		break;
	}

	ax_local->media_curr = inb((unsigned long)ax_base + EN0_SR);

}


/*
 * ----------------------------------------------------------------------------
 * Function Name: ax_init
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void ax_init(struct net_device *dev, int startp)
{
	struct ax_device *ax_local = (struct ax_device *) netdev_priv(dev);
	void *ax_base = ax_local->membase;
	int i;
	
	/* Follow National Semi's recommendations for initing the DP83902. */
	outb(E8390_NODMA+E8390_PAGE0+E8390_STOP, ((unsigned long)ax_base) + E8390_CMD); /* 0x21 */
	outb(0x49, ((unsigned long)ax_base) + EN0_DCFG);	/* 0x48 or 0x49 */
	/* Clear the remote byte count registers. */
	outb(0x00,  ((unsigned long)ax_base) + EN0_RCNTLO);
	outb(0x00,  ((unsigned long)ax_base) + EN0_RCNTHI);
	/* Set to monitor and loopback mode -- this is vital!. */
	outb(E8390_RXOFF, ((unsigned long)ax_base) + EN0_RXCR); /* 0x20 */
	outb(E8390_TXOFF, ((unsigned long)ax_base) + EN0_TXCR); /* 0x02 */
	/* Set the transmit page and receive ring. */
	outb(NESM_START_PG, ((unsigned long)ax_base) + EN0_TPSR);
	outb(NESM_RX_START_PG, ((unsigned long)ax_base) + EN0_STARTPG);
	outb(NESM_RX_START_PG, ((unsigned long)ax_base) + EN0_BOUNDARY);

	ax_local->current_page = NESM_RX_START_PG + 1;		/* assert boundary+1 */
	outb(NESM_STOP_PG, ((unsigned long)ax_base) + EN0_STOPPG);

	ax_local->tx_prev_ctepr = 0;
	ax_local->tx_start_page = NESM_START_PG;
	ax_local->tx_curr_page = NESM_START_PG;
	ax_local->tx_stop_page = NESM_START_PG + TX_PAGES;

	/* Clear the pending interrupts and mask. */
	outb(0xFF, ((unsigned long)ax_base) + EN0_ISR);
	outb(0x00,  ((unsigned long)ax_base) + EN0_IMR);

	/* Copy the station address into the DS8390 registers. */

	outb(E8390_NODMA + E8390_PAGE1 + E8390_STOP, ((unsigned long)ax_base) + E8390_CMD); /* 0x61 */
	outb(NESM_START_PG + TX_PAGES + 1, ((unsigned long)ax_base) + EN1_CURPAG);
	for(i = 0; i < 6; i++) 
	{
		outb(dev->dev_addr[i], ((unsigned long)ax_base) + EN1_PHYS_SHIFT(i));
		if(inb((unsigned long)ax_base + EN1_PHYS_SHIFT(i))!=dev->dev_addr[i])
			PRINTK(ERROR_MSG, "Hw. address read/write mismap %d\n",i);
	}
	outb(E8390_NODMA+E8390_PAGE0+E8390_STOP, ((unsigned long)ax_base) + E8390_CMD);

	if (startp) 
	{
		ax88796_PHY_init(dev);
		outb(0xff,  ((unsigned long)ax_base) + EN0_ISR);
		outb(ENISR_ALL,  ((unsigned long)ax_base) + EN0_IMR);
		outb(E8390_NODMA+E8390_PAGE0+E8390_START, ((unsigned long)ax_base) + E8390_CMD);
		outb(E8390_TXCONFIG, ((unsigned long)ax_base) + EN0_TXCR); /* xmit on. */ 
		/* 3c503 TechMan says rxconfig only after the NIC is started. */
		outb(E8390_RXCONFIG, ((unsigned long)ax_base) + EN0_RXCR); /* rx on,  */
		do_set_multicast_list(dev);	/* (re)load the mcast table */
	}

}


/*
 * ----------------------------------------------------------------------------
 * Function Name: ax_trigger_send
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void ax_trigger_send(struct net_device *dev, unsigned int length, int start_page)
{
 	struct ax_device *ax_local = (struct ax_device *) netdev_priv(dev);
	void *ax_base = ax_local->membase;
	
	outb(E8390_NODMA+E8390_PAGE0, ((unsigned long)ax_base) + E8390_CMD);
	outb(length & 0xff, ((unsigned long)ax_base) + EN0_TCNTLO);
	outb(length >> 8, ((unsigned long)ax_base) + EN0_TCNTHI);
	outb(start_page, ((unsigned long)ax_base) + EN0_TPSR);
	outb((E8390_NODMA|E8390_TRANS|E8390_START), ((unsigned long)ax_base) + E8390_CMD);
}


/*
 * ----------------------------------------------------------------------------
 * Function Name: ax_watchdog
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void ax_watchdog(unsigned long arg)
{

	struct net_device *dev = (struct net_device *)(arg);
 	struct ax_device *ax_local = (struct ax_device *) netdev_priv(dev);
	void *ax_base = ax_local->membase;

	int status,duplex;

	status = inb((unsigned long)ax_base + EN0_SR);
	if(ax_local->media_curr != status) {

		if ((status & 0x01))
		{
			if ((status & 0x04)) {
				PRINTK(DRIVER_MSG, "%s Link mode : 100 Mb/s  ", dev->name);
			}
			else {
				PRINTK(DRIVER_MSG, "%s Link mode : 10 Mb/s  ", dev->name);
			}

			duplex = inb((unsigned long)ax_base + EN0_MCR);
			if ((duplex & 0x80)) {
				PRINTK(DRIVER_MSG, "Duplex mode.\n");
			}
			else {
				PRINTK(DRIVER_MSG, "Half mode.\n");	
			}
		}
		else {
			PRINTK(DRIVER_MSG, "%s Link down.\n", dev->name);
		}

		ax_local->media_curr = inb((unsigned long)ax_base + EN0_SR);
	}
	mod_timer(&ax_local->watchdog, jiffies + AX88796_WATCHDOG_PERIOD);
	return ;
}


#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
/*
 * ----------------------------------------------------------------------------
 * Function Name: ax_vlan_rx_register
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void ax_vlan_rx_register(struct net_device *dev, struct vlan_group *grp)
{
 	struct ax_device *ax_local = (struct ax_device *) netdev_priv(dev);
	void *ax_base = ax_local->membase;

	outb((inb((unsigned long)ax_base+EN0_MCR) | ENVLAN_ENABLE), ((unsigned long)ax_base) + EN0_MCR);
	ax_local->vlgrp = grp;
}


/*
 * ----------------------------------------------------------------------------
 * Function Name: ax_vlan_rx_kill_vid
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void ax_vlan_rx_kill_vid(struct net_device *dev, unsigned short vid)
{
 	struct ax_device *ax_local = (struct ax_device *) netdev_priv(dev);
	void *ax_base = ax_local->membase;

	outb((inb((unsigned long)ax_base+EN0_MCR) & ~ENVLAN_ENABLE), ((unsigned long)ax_base) + EN0_MCR);
	if (ax_local->vlgrp)
		;///?ax_local->vlgrp->vlan_devices[vid] = NULL;
}
#endif


/*======================================================================
    MII interface support
======================================================================*/
#define MDIO_SHIFT_CLK		0x01
#define MDIO_DATA_WRITE0		0x00
#define MDIO_DATA_WRITE1		0x08
#define MDIO_DATA_READ		0x04
#define MDIO_MASK			0x0f
#define MDIO_ENB_IN			0x02

static void mdio_sync(struct net_device *dev)
{
	struct ax_device *ax_local = (struct ax_device *) netdev_priv(dev);
	void *ax_base = ax_local->membase;
    int bits;
    for (bits = 0; bits < 32; bits++) {
		outb(MDIO_DATA_WRITE1, ((unsigned long)ax_base) + AX88796_MII_EEPROM);
		outb(MDIO_DATA_WRITE1 | MDIO_SHIFT_CLK, ((unsigned long)ax_base) + AX88796_MII_EEPROM);
    }
}

static void mdio_clear(struct net_device *dev)
{
	struct ax_device *ax_local = (struct ax_device *) netdev_priv(dev);
	void *ax_base = ax_local->membase;

    int bits;
    for (bits = 0; bits < 16; bits++) {
		outb(MDIO_DATA_WRITE0, ((unsigned long)ax_base) + AX88796_MII_EEPROM);
		outb(MDIO_DATA_WRITE0 | MDIO_SHIFT_CLK, ((unsigned long)ax_base) + AX88796_MII_EEPROM);
    }
}

static int mdio_read(struct net_device *dev, int phy_id, int loc)
{
	struct ax_device *ax_local = (struct ax_device *) netdev_priv(dev);
	void *ax_base = ax_local->membase;

    u_int cmd = (0xf6<<10)|(phy_id<<5)|loc;
    int i, retval = 0;
	mdio_clear(dev);
    mdio_sync(dev);
    for (i = 14; i >= 0; i--) {
		int dat = (cmd&(1<<i)) ? MDIO_DATA_WRITE1 : MDIO_DATA_WRITE0;
		outb(dat, ((unsigned long)ax_base) + AX88796_MII_EEPROM);
		outb(dat | MDIO_SHIFT_CLK, ((unsigned long)ax_base) + AX88796_MII_EEPROM);
    }
    for (i = 19; i > 0; i--) {
		outb(MDIO_ENB_IN, ((unsigned long)ax_base) + AX88796_MII_EEPROM);
		retval = (retval << 1) | ((inb((unsigned long)ax_base + AX88796_MII_EEPROM) & MDIO_DATA_READ) != 0);
		outb(MDIO_ENB_IN | MDIO_SHIFT_CLK, ((unsigned long)ax_base) + AX88796_MII_EEPROM);
    }
    return (retval>>1) & 0xffff;
}

static void mdio_write(struct net_device *dev, int phy_id, int loc, int value)
{
	struct ax_device *ax_local = (struct ax_device *) netdev_priv(dev);
	void *ax_base = ax_local->membase;

    u_int cmd = (0x05<<28)|(phy_id<<23)|(loc<<18)|(1<<17)|value;
    int i;
	mdio_clear(dev);
    mdio_sync(dev);
    for (i = 31; i >= 0; i--) {
	int dat = (cmd&(1<<i)) ? MDIO_DATA_WRITE1 : MDIO_DATA_WRITE0;
		outb(dat, ((unsigned long)ax_base) + AX88796_MII_EEPROM);
		outb(dat | MDIO_SHIFT_CLK, ((unsigned long)ax_base) + AX88796_MII_EEPROM);
    }
    for (i = 1; i >= 0; i--) {
		outb(MDIO_ENB_IN, ((unsigned long)ax_base) + AX88796_MII_EEPROM);
		outb(MDIO_ENB_IN | MDIO_SHIFT_CLK, ((unsigned long)ax_base) + AX88796_MII_EEPROM);
    }
}
