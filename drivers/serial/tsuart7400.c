/*
 *  linux/drivers/serial/tsuart7400.c
 *
 *  TS-UART loader for Technologic Systems TS-7400
 *
 *  (c) 2006 Technologic Systems
 *
 * This source code is hereby licensed to the Linux 2.6 kernel under the
 * GPL, and to Technologic Systems customers under the BSD license.
 *
 * 2006-07-18:mos:initial version, ported from 2.4 version minus support for TS-7412
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

#define BOARD_ID_7400 0x7400
static int tryInitPort(int base) {
  struct tsuart_port *ts;

  ts = kmalloc(sizeof(struct tsuart_port),GFP_KERNEL);
  if (!ts) {
    printk("Unable to allocate memory for tsuart port\n");
    return 0;
  }
  memset(ts,0,sizeof(struct tsuart_port));
  init_tsuart_port(ts);
  ts->u.iotype = SERIAL_IO_MEM;
  ts->get = tsuartGetMEM8;
  ts->put = tsuartPutMEM8;
  ts->mdmctrl = 0;
  ts->debugFlags = portDebug;
  ts->u.mapbase = /*0 x72000000 + */ base;
  ts->u.irq = 33;
  ts->localStatus |= PORT_STATUS_SHIRQ;
  ts->boardId = BOARD_ID_7400;

  if (tsuart_register_port(0,ts) != 0) {
    return 0;
  }
  return 1;
}

static int __init tsuart_init(void)
{
  int ports=0;
	
  if (tryInitPort(0x12400000)) {
    ports++;
  }
  return (ports > 0) ? 0 : -1;
}

static void __exit tsuart_exit(void)
{
}

module_init(tsuart_init);
module_exit(tsuart_exit);


