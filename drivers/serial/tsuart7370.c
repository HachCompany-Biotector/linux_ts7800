/*
 *  linux/drivers/serial/tsuart7370.c
 *
 *  Linux 2.6 Driver for Technologic Systems UART
 *
 *  (c) 2007 Technologic Systems
 *
 * This source code is hereby licensed to the Linux 2.4 kernel under the
 * GPL, and to Technologic Systems customers under the BSD license.
 *
 * 2008-04-14:mos:initial version, based on tsuart7350.c
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
// TS-7370 magic 0x6C92
//---------------------------------------------------------------------------
#define BOARD_ID_7370 0x7370
#define FPGA_BASE 0x600ff080
volatile unsigned short *tmp;

static int mdmctrl(struct tsuart_port *ts,int mctrl) {
  int dtr,dcd,port;

  // DTR is an output
  // DCD is an input
  dtr = (mctrl & TIOCM_DTR) ? 1 : 0;
  dcd = (mctrl & TIOCM_CAR) ? 1 : 0;

  port = (((int)ts->u.mapbase) - 0x600ff0a0)/4;
  //                  DATA OE   DTR    DCD
  // DTR=B8,  DCD=B11 0x0C 0x14 0x0080 0x0400
  // DTR=B15, DCD=B15 0x0C 0x14 0x8000 0x4000
  if (port == 9) {
    if (mctrl & 0x80000000) {
      if (dtr) {
	tmp[0x0C/2] |= 0x0080;
      } else {
	tmp[0x0C/2] &= ~0x0080;
      }
    }
    dtr = !!(tmp[0x0C/2] & 0x0080);
    dcd = !!(tmp[0x0C/2] & 0x0400);
  } else if (port == 10) {
    if (mctrl & 0x80000000) {
      if (dtr) {
	tmp[0x0C/2] |= 0x8000;
      } else {
	tmp[0x0C/2] &= ~0x8000;
      }
    }
    dtr = !!(tmp[0x0C/2] & 0x8000);
    dcd = !!(tmp[0x0C/2] & 0x4000);
  }
  return ((dcd ? TIOCM_CAR : 0) | TIOCM_DSR | (dtr ? TIOCM_DTR : 0));
}

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
  ts->mdmctrl = (offset > 8) ? mdmctrl : 0;
  ts->debugFlags = portDebug;
  ts->u.mapbase = 0x600ff0a0 + 4 * offset;
  ts->u.irq = 32;
//  ts->localStatus |= PORT_STATUS_SHIRQ;
  ts->boardId = BOARD_ID_7370;

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
  ts->mdmctrl = (offset > 8) ? mdmctrl : 0;
  ts->debugFlags = portDebug;
  ts->pstatus |= PORT_STATUS_TXMSB;
  ts->rxsize = 2;
  ts->u.mapbase = 0x600ff0a0 + 4 * offset;
  ts->u.irq = 32;
//  ts->localStatus |= PORT_STATUS_SHIRQ;
  ts->boardId = BOARD_ID_7370;

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
  int i;

  dr9 = kmalloc(sizeof(struct uart_driver),GFP_KERNEL);
  if (!dr9) {
    printk("Unable to allocate memory for tsuart port\n");
    return -1;
  }

  tsuart_configure_driver(dr9,234,65,"tt8s",12);
  for (i=0;i<5;i++) {
    tryInitPort(i);
  }

  // UART 5  A29-31 [31:28] [15:12] 0x1A 0xF000
  // UART 6  A25-28 [27:24] [11:08] 0x1A 0x0F00
  // UART 7  A21-24 [23:20] [07:04] 0x1A 0x00F0
  // UART 8  A17-20 [19:16] [03:00] 0x1A 0x000F
  // UART 9  A11-16 [15:10] [15:10] 0x18 0xFC00 
  // UART 10 B15-20 [19:14] [3:0]   0x1E 0x000F
  //                        [15:14] 0x1C 0xC000
  //                  DATA OE   DTR    DCD
  // DTR=B8,  DCD=B11 0x0C 0x14 0x0080 0x0400 UART9
  // DTR=B15, DCD=B15 0x0C 0x14 0x8000 0x4000 UART10
  tmp = ((unsigned short *) __ioremap(0x600ff000, 4096,0));
  if (tmp == (unsigned short *)0x600ff000) {
    printk(KERN_ERR "tsuart1: cannot map 0xE8000000\n");
    return 0;
  }
  tmp += 0x80/2;
  if ((tmp[0x1A/2] & 0xF000) == 0) {
    tryInitPort(5);
  } else {
    printk("Not initializing TS-7370 TS-UART port 5 up (PC104 pins in use)\n");
    goto initend;
  }
  if ((tmp[0x1A/2] & 0x0F00) == 0) {
    tryInitPort(6);
  } else {
    printk("Not initializing TS-7370 TS-UART port 6 up (PC104 pins in use)\n");
    goto initend;
  }
  if ((tmp[0x1A/2] & 0x00F0) == 0) {
    tryInitPort(7);
  } else {
    printk("Not initializing TS-7370 TS-UART port 7 up (PC104 pins in use)\n");
    goto initend;
  }
  if ((tmp[0x1A/2] & 0x000F) == 0) {
    tryInitPort(8);
  } else {
    printk("Not initializing TS-7370 TS-UART port 8 up (PC104 pins in use)\n");
    goto initend;
  }
  if ((tmp[0x18/2] & 0xFC00) == 0) {
    tmp[0x14/2] |= 0x0080;
    tmp[0x14/2] &= ~0x0400;
    tryInitPort(9);
  } else {
    printk("Not initializing TS-7370 TS-UART port 9 up (PC104 pins in use)\n");
    goto initend;
  }
  if ((tmp[0x1E/2] & 0x000F) == 0
      && (tmp[0x1C/2] & 0xC000) == 0) {
    tmp[0x14/2] |= 0x8000;
    tmp[0x14/2] &= ~0x4000;
    tryInitPort(10);
  } else {
    printk("Not initializing TS-7370 TS-UART port 10 (PC104 pins in use)\n");
  }
  tryInitPort(11); // where did this port come from?
  // Note that if some ports in the middle are not initialized, this will cause
  // the strange condition that device names will be different for the same port!
  // this is unacceptable because we require the ability to detect the correct port for DTR/DCD purposes!
 initend:
  return 0;
}

static void __exit tsuart_exit(void)
{
}

module_init(tsuart_init);
module_exit(tsuart_exit);

MODULE_DESCRIPTION("Technologic Systems TS-UART driver");


