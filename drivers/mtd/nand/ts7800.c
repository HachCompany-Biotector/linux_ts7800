/*
 *  drivers/mtd/nand/ts7800.c
 *
 *  Copyright (C) 2007 Technologic Systems (support@embeddedARM.com)
 *
 *  Derived from drivers/mtd/nand/edb7312.c
 *       Copyright (C) 2004 Marius Gr?er (mag@sysgo.de)
 *
 *  Derived from drivers/mtd/nand/autcpu12.c
 *       Copyright (c) 2001 Thomas Gleixner (gleixner@autronix.de)
 *
 * $Id: ts7800.c,v 1.1.1.1 2010/10/22 17:32:42 ian Exp $
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Overview:
 *   This is a device driver for the NAND flash device found on the
 *   TS-78xx boards which utilizes a 512Mbyte part
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <asm/sizes.h>
#include <asm/cacheflush.h>
#include <linux/semaphore.h>
#include <linux/fs.h>

extern int dma_op (unsigned long dev_addr, unsigned long dat, unsigned int len, short rw);
extern void __attribute__((naked)) xdma_clean_range(char *start, char *end);
extern void __attribute__((naked)) xdma_inv_range(char * start, char * end);

struct partition_info { // little-endian format
  unsigned int bootable:8; // 0x80 = active, 0x00 = don't use for booting
  unsigned int starting_head:8;
  unsigned int starting_csect:8;
  unsigned int starting_xcyl:8;
  // MSB bits 0-5 are sector
  unsigned int system_id:8; // format of partition
  unsigned int ending_head:8;
  unsigned int ending_csect:8;
  unsigned int ending_xcyl:8;
  unsigned int relative_sector:32; // sector offset from start of disk to start of volume
  unsigned int total_sectors:32;
} __attribute__((packed));

struct partition_table {
  struct partition_info part[4];
} __attribute__((__packed__));

struct MBR {
  char bootcode[446];
  struct partition_table pt;
  unsigned int sig55:8;
  unsigned int sigAA:8;
} __attribute__((__packed__));

/*
 * MTD structure for TS7800 board
 */
static struct mtd_info *ts7800_mtd = NULL;


/*
 * Module stuff
 */

static unsigned int ts7800_nandctrl = 0;
static unsigned int ts7800_nanddat = 0;
static unsigned int ts7800_dmaaddr = 0;

#ifdef CONFIG_MTD_PARTITIONS
static struct mtd_partition partition_mbr[] = {
	{ .name= "whole chip",
		  .offset= 0,
		  .size= 0x20000000 },
	{ .name= "kernel",
		  .offset= 0x20000,
		  .size= 0x400000 },
	{ .name= "initrd",
		  .offset= 0x420000,
		  .size= 0x400000 },
	{ .name= "rootfs",
		  .offset= 0x820000,
		  .size= 0x20000000 - 0x820000 },
	{ .name= "part4",
		  .offset= 0x20000000,
		  .size= 0 },
};
#endif

/*
 *	hardware specific access to control-lines
 *
 *	ctrl:
 *	NAND_NCE: bit 0 -> bit 2
 *	NAND_CLE: bit 1 -> bit 1
 *	NAND_ALE: bit 2 -> bit 0
 */
static void ts7800_hwcontrol(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct nand_chip *chip = mtd->priv;

	if (ctrl & NAND_CTRL_CHANGE) {
		unsigned long addr = ts7800_nandctrl;
		unsigned char bits;

		bits = (ctrl & NAND_NCE) << 2;
		bits |= ctrl & NAND_CLE;
		bits |= (ctrl & NAND_ALE) >> 2;

		writeb((readb(addr) & ~0x7) | bits, addr);
	}

	if (cmd != NAND_CMD_NONE)
		writeb(cmd, chip->IO_ADDR_W);
}


static void ts7800_enable_hwecc(struct mtd_info *mtd,int dir) {
	int x;
	
	/* Enable and empty FIFO */
	x = readl(ts7800_nandctrl);
	while ((x & 0x18) != 0x18) {
		writel(x | 0x8, ts7800_nandctrl);
		x = readl(ts7800_nandctrl);
	}
}

static int ts7800_calculate_ecc(struct mtd_info *mtd, const u_char *dat, u_char *ecc_code) {
  unsigned dummy,i;

  for (i = 0; i < 8; i++) {
    dummy = readl(ts7800_nandctrl); // *buf;
    dummy =  0xFFFFFF & ((dummy >> 6) | 3);
    *ecc_code++ = (dummy >> 16) & 0xFF;
    *ecc_code++ = (dummy >> 8) & 0xFF;
    *ecc_code++ = (dummy >> 0) & 0xFF;
  }
  return dummy;
}

extern int nand_correct_data(struct mtd_info *mtd, u_char *dat, u_char *read_ecc, u_char *calc_ecc);
int ts7800_correct_data(struct mtd_info *mtd, u_char *dat,
                      u_char *read_ecc, u_char *calc_ecc)
{
	int ret = 0, i;

	for (i = 0; i < 8; i++) {
		ret |= nand_correct_data(mtd, dat + (256 * i), 
		  &read_ecc[i * 3], &calc_ecc[i * 3]);
	}
	return ret;
}


static void ts7800_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	unsigned int bufadr, remain = 0;
//        unsigned long msireg;

	while (((int)buf & 0x1) || len == 1) {
		writeb(*buf++, ts7800_nanddat);
		len--;
	}

        if (((int)buf & 0x2) && len >= 2) {
		writew(*(unsigned short *)buf, ts7800_nanddat);
		buf += 2;
		len -= 2;
	}

	if (len == 0) return;

	if ((unsigned long)buf >= VMALLOC_START) {
		bufadr = (vmalloc_to_pfn(buf) << PAGE_SHIFT) |
		  ((unsigned int)buf & 0xfff);
	} else {
		/* XXX: virt_to_phys() should be properly named and called
		 * virt_to_phys_with_surprise_of_only_working_on_a
		 * _subset_of_address_space()
		 */
		bufadr = virt_to_phys((char*)buf);
	}

	if (len > (0x1000 - (bufadr & 0xfff))) {
		remain = 0x1000 - (bufadr & 0xfff);
		len = len - remain;
	}

	xdma_clean_range((char *)buf, (char *)buf + len);
	while(dma_op (0x80000804, bufadr, len, WRITE) < 0) schedule();

	if (remain) {
		buf += len;
		ts7800_write_buf(mtd, buf, remain);
	}
}

static void ts7800_read_buf(struct mtd_info *mtd, u_char *buf, int len)
{
	unsigned int bufadr, remain = 0;
//        unsigned long  msireg;

	struct nand_chip *this;
        this = ((struct nand_chip *)(&ts7800_mtd[1]));


	while (((int)buf & 0x1) || len == 1) {
		*buf++ = readb(ts7800_nanddat);
		len--;
	}

        if (((int)buf & 0x2) && len >= 2) {
		*(unsigned short *)buf = readw(ts7800_nanddat);
		buf += 2;
		len -= 2;
	}

	if (len == 0) return;

	if ((unsigned long)buf >= VMALLOC_START) {
		bufadr = (vmalloc_to_pfn(buf) << PAGE_SHIFT) | 
		  ((unsigned int)buf & 0xfff);
	} else {
		/* XXX: virt_to_phys() should be properly named and called
		 * virt_to_phys_with_surprise_of_only_working_on_a
		 * _subset_of_address_space()
		 */
		bufadr = virt_to_phys((char *)buf);
	}

	if (len > (0x1000 - (bufadr & 0xfff))) {
		remain = 0x1000 - (bufadr & 0xfff);
		len = len - remain;
	}

	xdma_inv_range(buf, buf + len);
	while(dma_op (0x80000804, bufadr, len, READ) < 0) schedule();

	if (remain) {
		buf += len;
		ts7800_read_buf(mtd, buf, remain);
	}
}

/*
 *	read device ready pin
 */
static int ts7800_device_ready(struct mtd_info *mtd)
{
	return readb(ts7800_nandctrl) & 0x20;
}

/*
 * Main initialization routine
 */
static int __init ts7800_init (void)
{
	struct nand_chip *this;
	const char *part_type = 0;
	char *nandaddr;
	char buf[2048];
	int i,got = 0,parts=0;
	unsigned ofs=0;
	struct MBR *mbr = (struct MBR *)buf;
		
	/* Allocate memory for MTD device structure and private data */
	ts7800_mtd = kmalloc(sizeof(struct mtd_info) + 
			     sizeof(struct nand_chip),
			     GFP_KERNEL);
	if (!ts7800_mtd) {
		printk("Unable to allocate TS7800 NAND MTD device structure.\n");
		return -ENOMEM;
	}

   nandaddr = ioremap(0xe8000000, 0x1000);
	
	/* Get pointer to private data */
	this = (struct nand_chip *) (&ts7800_mtd[1]);
	
	/* Initialize structures */
	memset((char *) ts7800_mtd, 0, sizeof(struct mtd_info));
	memset((char *) this, 0, sizeof(struct nand_chip));
	
	/* Link the private data with the MTD structure */
	ts7800_mtd->priv = this;
        ts7800_mtd->owner = THIS_MODULE;
	
	/* insert callbacks */
	this->IO_ADDR_R = nandaddr + 0x804;
	this->IO_ADDR_W = nandaddr + 0x804;
	ts7800_nanddat = (unsigned int)nandaddr + 0x804;
	ts7800_nandctrl = (unsigned int)nandaddr + 0x800;
	ts7800_dmaaddr = (unsigned int)nandaddr + 0x400;

	this->cmd_ctrl = ts7800_hwcontrol;
	this->dev_ready = ts7800_device_ready;
	this->chip_delay = 15;
	this->options = NAND_USE_FLASH_BBT;
#ifdef DISABLE_DMA_FOR_TEST
        this->ecc.mode = NAND_ECC_SOFT;
#else
        this->ecc.mode = NAND_ECC_HW;
#endif
        this->ecc.size = 2048;
        this->ecc.bytes = 24;
#ifndef DISABLE_DMA_FOR_TEST
        this->ecc.hwctl = ts7800_enable_hwecc;
        this->ecc.calculate = ts7800_calculate_ecc;
        this->ecc.correct = ts7800_correct_data;
#endif
        // this->autooob = &ts7800_oobinfo;
#ifndef DISABLE_DMA_FOR_TEST
	this->read_buf = ts7800_read_buf;
	this->write_buf = ts7800_write_buf;
#endif
	printk("Searching for NAND flash...\n");
	/* Scan to find existence of the device */
	if (nand_scan (ts7800_mtd, 1)) {
		kfree (ts7800_mtd);
		return -ENXIO;
	}
	for (i=0;i<512;i++) {
	  buf[i] = 0;
	}
	ts7800_mtd->read(ts7800_mtd,0,512,&got,buf);

	if ( mbr->sigAA == 0xAA && mbr->sig55 == 0x55) {
          part_type = "MBR";
	  for (i=0;i<4;i++) {
	    if (mbr->pt.part[i].total_sectors) {
	      parts++;
	      partition_mbr[i+1].offset = 
	        512 * mbr->pt.part[i].relative_sector;
	      if (i < 2) {
	        partition_mbr[i+1].size = 
	          512 * mbr->pt.part[i+1].relative_sector -
	          512 * mbr->pt.part[i].relative_sector;
	      } else {
	        partition_mbr[i+1].size = 
	          512 * mbr->pt.part[i].total_sectors;
	      }
	      ofs = partition_mbr[i+1].offset 
	        + partition_mbr[i+1].size;
	    }
	  }
	} else {
          part_type = "default";
	  parts = 3;
	}
	
	/* Register the partitions */
	printk(KERN_NOTICE "Using %s partition definition\n", part_type);
	add_mtd_partitions(ts7800_mtd, partition_mbr, parts + 1);
	
	/* Return happy */
	return 0;
}
module_init(ts7800_init);

/*
 * Clean up routine
 */
static void __exit ts7800_cleanup (void)
{
	/* Unregister the device */
	del_mtd_device (ts7800_mtd);

	iounmap((void *)ts7800_nandctrl);
	
	/* Free the MTD device structure */
	kfree (ts7800_mtd);
}
module_exit(ts7800_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jesse Off <joff@embeddedARM.com>");
MODULE_DESCRIPTION("MTD driver for Technologic Systems TS-7800 board");
