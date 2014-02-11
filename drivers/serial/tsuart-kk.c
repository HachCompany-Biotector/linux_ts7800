/*
 *  linux/drivers/serial/tsuart-kk.c
 *
 *  Linux 2.6 Driver for Technologic Systems UART
 *
 *  (c) 2007 Technologic Systems
 *
 * This source code is hereby licensed to the Linux 2.4 kernel under the
 * GPL, and to Technologic Systems customers under the BSD license.
 *
 * 2008-08-26:mos:initial version, based on tsuart7370.c
 */
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/serial.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <linux/serial_core.h>
#include <asm/uaccess.h>
#include "tsuart1.h"

#define BOARD_ID_KK 0x77
#define FPGA_BASE 0x600ff080
volatile unsigned short *tmp;

static int tryInitPort(int offset) {
  struct tsuart_port *ts;

  printk("Port %d\n",offset);
  ts = kmalloc(sizeof(struct tsuart_port),GFP_KERNEL);
  if (!ts) {
    printk("Unable to allocate memory for tsuart port\n");
    return 0;
  }
  memset(ts,0,sizeof(struct tsuart_port));

  init_tsuart_port(ts);
  ts->u.iotype = SERIAL_IO_MEM;
  ts->get = tsuartGetMEM16;
  ts->put = tsuartPutMEM16;
  ts->mdmctrl = 0;
  ts->debugFlags = portDebug;
  ts->u.mapbase = 0x600ff0a0 + 4 * offset;
  ts->u.irq = 32;
  ts->localStatus |= PORT_STATUS_SHIRQ;
  ts->boardId = BOARD_ID_KK;
  if (tsuart_register_port(0,ts) != 0) {
    printk("Failed to register port!\n");
    return 0;
  }
  return 1;
}

//---------------------------------------------------------------------------
// KERNEL MODULE
//---------------------------------------------------------------------------
static int __init tsuart_init(void)
{
  tryInitPort(0);
  tryInitPort(1);
  return 0;
}

static void __exit tsuart_exit(void)
{
}

module_init(tsuart_init);
module_exit(tsuart_exit);

MODULE_DESCRIPTION("Technologic Systems TS-UART driver");


