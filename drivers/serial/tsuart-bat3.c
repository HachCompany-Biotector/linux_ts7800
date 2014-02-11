/*
 *  linux/drivers/serial/tsuart-bat3.c
 *
 *  TS-UART loader for Technologic Systems TS-BAT3
 *
 *  (c) 2007 Technologic Systems
 *
 * This source code is hereby licensed to the Linux 2.6 kernel under the
 * GPL, and to Technologic Systems customers under the BSD license.
 *
 * 2007-11-28:mos:initial 2.6 version ported from 2.4 version
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

#define BOARD_ID_BAT3 0xBA13

static struct uart_driver *dr3=0;

static unsigned long saved_A=0, saved_B=0;

static void on104(void) {
  volatile unsigned long *tmp;
  tmp = ((unsigned long *) __ioremap(0xE8000000, 4096,0));
  if (tmp == (unsigned long *)0xE8000000) {
    printk(KERN_ERR "tsuart1: cannot map 0xE8000000\n");
    return;
  }
  saved_A = tmp[0x30/4];
  saved_B = tmp[0x34/4];
  tmp[0x30/4] = 0x55555555;
  tmp[0x34/4] = 0x55555555;
  __iounmap(tmp);
}

static void off104(void) {
  volatile unsigned long *tmp;
  tmp = ((unsigned long *) __ioremap(0xE8000000, 4096,0));
  if (tmp == (unsigned long *)0xE8000000) {
    printk(KERN_ERR "tsuart1: cannot map 0xE8000000\n");
    return;
  }
  tmp[0x30/4] = saved_A;
  tmp[0x34/4] = saved_B;
  __iounmap(tmp);
}

static int tryInitPort(int iobase) {
  struct tsuart_port *ts;
  unsigned val;
  volatile unsigned char *sh;

  ts = kmalloc(sizeof(struct tsuart_port),GFP_KERNEL);
  if (!ts) {
    printk("Unable to allocate memory for tsuart port\n");
    return 0;
  }
  memset(ts,0,sizeof(struct tsuart_port));
  init_tsuart_port(ts);
  //ts->u.iotype = SERIAL_IO_PORT;
  ts->u.iotype = SERIAL_IO_MEM;
  ts->get = tsuartGetMEM8;
  ts->put = tsuartPutMEM8;
  //ts->get = tsuartGetIO8;
  //ts->put = tsuartPutIO8;
  ts->debugFlags = portDebug;
  ts->u.mapbase = 0xEE000000 + iobase + 2;
  // tsu->u.iobase = ??? + iobase + 2;
  ts->boardId = BOARD_ID_BAT3;
  ts->mdmctrl = 0;

#if 0
  ts->u.irq = 64 + 5; // IRQ5
  printk("TS-UART/BAT3 found at port 0x%04X\n",iobase);
  if (tsuart_register_port(dr3,ts) == 0) {
    __iounmap(sh);
    return 1;
  }
#else
  sh = ((volatile unsigned char *) __ioremap(0xEE000000, 4096,0));
  if (sh == (volatile unsigned char *)0xEE000000) {
    printk(KERN_ERR "tsuart-bat3: cannot map 0xEE000000\n");
    return -ENOMEM;
  }
  sh += iobase;
  val = sh[0];
  if (val == 0x28) {
    val = sh[1];
    if ((val & 0x80) == 0x80) {
      ts->u.irq = 64 + 6; // IRQ 6
      if ((val & 0x40) == 0x40) {
	ts->localStatus |= PORT_STATUS_SHIRQ; // IRQ6 with sharing...
      }
    } else {
      ts->u.irq = 64 + 5; // IRQ5
    }
    printk("TS-UART/BAT3 found at port 0x%04X\n",iobase);
    if (tsuart_register_port(dr3,ts) == 0) {
      __iounmap(sh);
      return 1;
    }
  }
  __iounmap(sh);
#endif
  return 0;
}

static int __init tsuart_init(void)
{
  int ports=0;

  dr3 = 0;
/*
  dr3 = kmalloc(sizeof(struct uart_driver),GFP_KERNEL);
  if (!dr3) {
    printk("Unable to allocate memory for tsuart port\n");
    return 0;
  }
 if (!tsuart_configure_driver(dr3,234,95,"ttyT3S",4)) {
    kfree(dr3);
    return 0;
  }
*/
  on104();

  if (tryInitPort(0x100)) {
    ports++;
  }
  if (tryInitPort(0x110)) {
    ports++;
  }
  if (tryInitPort(0x200)) {
    ports++;
  }
  if (tryInitPort(0x210)) {
    ports++;
  }

  if (ports == 0) {
    printk("TS-UART/BAT3 did not detect a TS-BAT3 board\n");
    off104();
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

