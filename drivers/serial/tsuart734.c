/*
 *  linux/drivers/serial/tsuart-bat3.c
 *
 *  TS-UART loader for Technologic Systems TS-734
 *
 *  (c) 2008 Technologic Systems
 *
 * This source code is hereby licensed to the Linux 2.6 kernel under the
 * GPL, and to Technologic Systems customers under the BSD license.
 *
 * 2008-07-18:mos:initial version, ported from 2.4 version
 *
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

struct uart_driver *dr8=0,*dr9=0;

#define BOARD_ID_734 0x734

static int tryInitPort(int i) {
  struct tsuart_port *ts1,*ts2;

  ts1 = kmalloc(sizeof(struct tsuart_port),GFP_KERNEL);
  if (!ts1) {
    printk("Unable to allocate memory for tsuart port\n");
    return 0;
  }
  memset(ts1,0,sizeof(struct tsuart_port));
  init_tsuart_port(ts1);
  ts1->u.iotype = SERIAL_IO_MEM;
  ts1->get = tsuartGetMEM16;
  ts1->put = tsuartPutMEM16;
  ts1->debugFlags = portDebug;
  ts1->u.mapbase = 0x72000400+i*4;
  ts1->portStatus = &ts1->pstatus;
  ts1->pstatus = 0;
  ts1->boardId = BOARD_ID_734;
  ts1->u.irq = 40;
  ts1->mdmctrl = 0;
  ts1->localStatus |= PORT_STATUS_SHIRQ;

  ts2 = kmalloc(sizeof(struct tsuart_port),GFP_KERNEL);
  if (!ts2) {
    printk("Unable to allocate memory for tsuart port\n");
    return 0;
  }
  memset(ts2,0,sizeof(struct tsuart_port));
  init_tsuart_port(ts2);
  ts2->rxsize = 2;
  ts2->dataMask = 0x1FF;
  ts2->u.iotype = SERIAL_IO_MEM;
  ts2->get = tsuartGetMEM16_9;
  ts2->put = tsuartPutMEM16_9;
  ts2->debugFlags = portDebug;
  ts2->u.mapbase = 0x72000400+i*4;
  ts2->portStatus = &ts1->pstatus;
  ts2->pstatus |= PORT_STATUS_TXMSB;
  ts2->boardId = BOARD_ID_734;
  ts2->u.irq = 40;
  ts2->mdmctrl = 0;
  ts2->localStatus |= PORT_STATUS_SHIRQ;

  if (tsuart_register_port(dr8,ts1) == 0) {
    //return 1;
  }
  if (tsuart_register_port(dr9,ts2) == 0) {
    //return 1;
  }
  return 0;
}

static int __init tsuart_init(void)
{
  int i,ports=0;
  volatile unsigned short int *cr;

  cr = (volatile unsigned short *)((int) __ioremap(0x72000000, 4096,0) + 0x422);
  if (*cr != 0x79E1) {
    printk("TS-UART/734 did not find TS-734 board\n");
    __iounmap(cr);
    return -1;
  } else {
    printk("TS-UART/734 found TS-734 board\n");
    __iounmap(cr);
  }

  dr8 = kmalloc(sizeof(struct uart_driver),GFP_KERNEL);
  if (!dr8) {
    printk("Unable to allocate memory for tsuart port\n");
    return 0;
  }
  dr9 = kmalloc(sizeof(struct uart_driver),GFP_KERNEL);
  if (!dr9) {
    kfree(dr8);
    printk("Unable to allocate memory for tsuart port\n");
    return 0;
  }
 if (!tsuart_configure_driver(dr8,234,128,"ttyT8Sv",8)) {
    kfree(dr8);
    kfree(dr9);
    return 0;
  }
  if (!tsuart_configure_driver(dr9,234,136,"ttyT9Sv",8)) {
    kfree(dr8);
    kfree(dr9);
    return 0;
  }
  for (i=0;i<8;i++) {
    if (!tryInitPort(i)) {
      ports++;
    }
  }
  return (ports > 0) ? 0 : -1;
}

static void __exit tsuart_exit(void)
{
  // This module can be safely unloaded after initialization is complete.
  // We allow module to load only so error conditions can be detected.
}

module_init(tsuart_init);
module_exit(tsuart_exit);

MODULE_DESCRIPTION("Technologic Systems TS-UART driver");
