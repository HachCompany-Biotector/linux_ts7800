/*
 *  linux/drivers/serial/tsuart7350.c
 *
 *  Linux 2.6 Driver for Technologic Systems UART
 *
 *  (c) 2007 Technologic Systems
 *
 * This source code is hereby licensed to the Linux 2.4 kernel under the
 * GPL, and to Technologic Systems customers under the BSD license.
 *
 * 2008-03-19:mos:initial version, adapted from tsuart7800.c
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

//---------------------------------------------------------------------------
// TS-7350 (TEST)
//---------------------------------------------------------------------------
#define BOARD_ID_7350 0x7350

struct uart_driver *dr9=0;

static int tryInitPort(int offset) {
  struct tsuart_port *ts;

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
  ts->boardId = BOARD_ID_7350;

  if (tsuart_register_port(0,ts) != 0) {
    printk("Failed to register port!\n");
    return 0;
  }

  ts = kmalloc(sizeof(struct tsuart_port),GFP_KERNEL);
  if (!ts) {
    printk("Unable to allocate memory for tsuart nine-bit port\n");
    return 0;
  }
  memset(ts,0,sizeof(struct tsuart_port));

  init_tsuart_port(ts);
  ts->u.iotype = SERIAL_IO_MEM;
  ts->get = tsuartGetMEM16_9;
  ts->put = tsuartPutMEM16_9;
  ts->mdmctrl = 0;
  ts->debugFlags = portDebug;
  ts->pstatus |= PORT_STATUS_TXMSB;
  ts->rxsize = 2;
  ts->u.mapbase = 0x600ff0a0 + 4 * offset;
  ts->u.irq = 32;
  ts->localStatus |= PORT_STATUS_SHIRQ;
  ts->boardId = BOARD_ID_7350;

  if (tsuart_register_port(dr9,ts) != 0) {
    printk("Failed to register 9-bit port!\n");
    return 0;
  }

  return 1;
}

//---------------------------------------------------------------------------
// KERNEL MODULE
//---------------------------------------------------------------------------
static int __init tsuart_init(void)
{
  int i,maxport;
  unsigned short *tmp;

  tmp = ((unsigned short *) __ioremap(0x600FF000, 4096,0));
  if (tmp == (unsigned short *)0x600FF000) {
    printk(KERN_ERR "tsuart: cannot map 0x600FF000\n");
    return -ENOMEM;
  }
  if ((tmp[0x40] & 0xFFF0) != 0xCDE0) {
    printk("TS-UART: TS-7390 not detected\n");
    return -ENOMEM;
  }

  dr9 = kmalloc(sizeof(struct uart_driver),GFP_KERNEL);
  if (!dr9) {
    printk("Unable to allocate memory for tsuart port\n");
    return -1;
  }

  tsuart_configure_driver(dr9,234,65,"tt8s",10);
  for (i=0;i<6;i++) {
    tryInitPort(i);
  }
  return 0;
}

static void __exit tsuart_exit(void)
{
}

module_init(tsuart_init);
module_exit(tsuart_exit);

MODULE_DESCRIPTION("Technologic Systems TS-UART driver");


