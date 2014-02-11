/*
 *  linux/drivers/serial/tsuart7800.c
 *
 *  Linux 2.6 Driver for Technologic Systems UART
 *
 *  (c) 2007 Technologic Systems
 *
 * This source code is hereby licensed to the Linux 2.4 kernel under the
 * GPL, and to Technologic Systems customers under the BSD license.
 *
 * 2007-12-21:mos:removed debug code that wasn't initializing all port
 * 2008-01-17:mos:added support for 9-bit mode
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
// TS-7800 (TEST)
//---------------------------------------------------------------------------
#define BOARD_ID_7800 0x7800

struct uart_driver *dr9=0;

// mask off bits 0xFC000 in register 0xE8000038
// this sets C14-C19 on the PC104 bus to GPIO, which is needed for
// UART #8 and UART #9 signals to appear on the header.
int semioff104(void) {
  volatile unsigned long *tmp;
  tmp = ((unsigned long *) __ioremap(0xE8000000, 4096,0));
  if (tmp == (unsigned long *)0xE8000000) {
    printk(KERN_ERR "tsuart1: cannot map 0xE8000000\n");
    return 0;
  }
  if (tmp[0x38/4] & 0xFC000) {
    // already set to not GPIO; reject the change
    __iounmap(tmp);
    return 0; 
  } else {
    tmp[0x38/4] &= 0x03FFF;
    __iounmap(tmp);
    return 1;
  }
}

void (*old_shutdown)(struct uart_port *);
static struct uart_ops tsuart_ops_7800;
void shutdown_7800(struct uart_port *p) {

  // printk("Waiting for transmitter to be empty to shut down...");
  // assume we won't be able to send any more bytes while waiting
  while (!bit_TRE(TSPORT(p)->get(TSPORT(p),0))) {
    printk(".");
    set_current_state(TASK_INTERRUPTIBLE);
    schedule_timeout(1);
  }
  printk("done!\n");
  set_current_state(TASK_RUNNING);
  old_shutdown(p);
}

static int tryInitPort(int offset) {
  struct tsuart_port *ts;
  volatile unsigned long *tmp;
  volatile unsigned short *sh;
 
  tmp = ((unsigned long *) __ioremap(0xE8000000, 4096,0));
  if (tmp == (unsigned long *)0xE8000000) {
    printk(KERN_ERR "tsuart1: cannot map 0xE8000000\n");
    return -ENOMEM;
  }
  if ((*tmp & 0xFFFFFF00) != 0xB48000) {
    printk(KERN_ERR "TSUART1 magic: %lX\n",*tmp);
    return -1;
  }
  sh = (volatile unsigned short *)tmp;
  sh[122] = 5208; // 2400
  sh[121] = 2604; // 4800
  sh[120] = 1302; // 9600
  sh[119] = 651;  // 19200
  sh[118] = 326;  // 38400
  sh[117] = 217;  // 57600
  sh[116] = 108;  // 115200
  __iounmap(tmp);
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
  ts->u.mapbase = 0xE80000C0 + 4 * offset;
   
  
  ts->u.irq = 64 + 16 + offset; // to 64+25
  //ts->localStatus |= PORT_STATUS_SHIRQ;
  ts->boardId = BOARD_ID_7800;

  if (tsuart_register_port(0,ts) != 0) {
    printk("Failed to register port!\n");
    return 0;
  }

  old_shutdown = ts->u.ops->shutdown;
  tsuart_ops_7800 = *(ts->u.ops);
  tsuart_ops_7800.shutdown = shutdown_7800;
  ts->u.ops = &tsuart_ops_7800;

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
  ts->u.mapbase = 0xE80000C0 + 4 * offset;
  ts->u.irq = 64 + 16 + offset; // to 64+25
  //ts->localStatus |= PORT_STATUS_SHIRQ;
  ts->boardId = BOARD_ID_7800;

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

  dr9 = kmalloc(sizeof(struct uart_driver),GFP_KERNEL);
  if (!dr9) {
    printk("Unable to allocate memory for tsuart port\n");
    return -1;
  }

  tsuart_configure_driver(dr9,234,65,"tt8s",10);
  maxport = semioff104() ? 9 : 7;
  if (maxport == 7) {
    printk("Not initializing PC104 pin serial ports, these pins already marked as in use.\n");
  }
  for (i=0;i<=maxport;i++) {
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


