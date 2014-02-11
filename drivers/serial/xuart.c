/*
 *  linux/drivers/serial/xuart.c
 *
 *  Linux 2.6 Driver for TS-XUART on Prosoft
 *
 *  (c) 2008 Technologic Systems
 *
 * This source code is hereby licensed to the Linux 2.4 kernel under the
 * GPL, and to Technologic Systems customers under the BSD license.
 *
 * 2008-08-19:mos:created skeleton from xuart.c
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
#include "xuartcore.c"

#define DELAY_TIME      HZ / 100        // 100 Hz
#define PORT_XUART 99
#define XPORT(port) ((struct xuart_port *)port)

#define XU_ODD 1
#define XU_EVEN 2
#define XU_NONE 0

struct xuart_port {
  struct uart_port u;
  struct xuart_port *next;
  struct xuartcore *xu;
  volatile unsigned short *rtscts;
  int debugFlags;
  int readahead;
};

#define DEBUG_ENDS 1
#define DEBUG_CONFIG 2
#define DEBUG_PROBE 4
#define DEBUG_INTERFACE 8
#define DEBUG_CONTROL 16
#define DEBUG_INTERRUPT 32
#define DEBUG_DATA 64
#define DEBUG_TXRX 128
#define DEBUG_PARITY 256

//---------------------------------------------------------------------------
static void xuart_stop_tx(struct uart_port *port);
static unsigned int xuart_get_mctrl(struct uart_port *port);
//---------------------------------------------------------------------------

static struct timer_list timer;
static struct timer_list timer2 = { .function = NULL };
static int timer_added = 0;
static int started = 0;
//static struct xuart_port *TS = 0;
static struct xuart_port *ts = 0;

//---------------------------------------------------------------------------
// OS TX/RX INTERFACE
//---------------------------------------------------------------------------
// process the byte in the lsb of data by sending it to the
// appropriate OS buffers for the given TS UART UART
static void xuart_handleRx(struct xuart_port *port,unsigned int ch) {
  struct tty_struct *tty;
  int flag = 0;

  if (port->debugFlags & DEBUG_DATA) {
    unsigned int ttyinf = port->u.state ? (port->u.state->port.tty ? 3:1) : 0;
    printk("xuart Rx(%d):%02X:%d\n",port->u.line,ch,ttyinf);
  }
  if (!port->u.state || !port->u.state->port.tty) {
    return; 
  }
  tty = port->u.state->port.tty;
  /*
  if (tty->flip.count >= TTY_FLIPBUF_SIZE) {
    tty->flip.tqueue.routine((void *)tty);
    //printk("->%X ",tty->flip.char_buf_ptr);
    if (tty->flip.count >= TTY_FLIPBUF_SIZE) {
      printk("XUART0 - TTY FLIP BUF FAILED!\n");
      return;
    }
  }
  */
  /*
  if (PARITYERROR()) {
    flag |= TTY_PARITY;
  }
  */
  tty_insert_flip_char(tty, (ch & 0xFF), flag);
  return;
}

static void xuart_try_handleRx(struct xuart_port *port) {
  char buf[256];
  int i,len;

  while ((len = xu_readc(port->xu,port->u.line,buf,256)) > 0) {
    for (i=0;i<len;i++) {
      xuart_handleRx(port,buf[i]);
    }
    tty_flip_buffer_push(port->u.state->port.tty);
  }
  xu_readc(port->xu,port->u.line,NULL,port->readahead);
}


// If there is data ready to be transmitted for the given UART,
// then feed to the byte to the UART
static void xuart_try_handleTx(struct xuart_port *port) {
  struct circ_buf *xmit;
  unsigned int ch=0,i=0;

  if (!port->u.state || !port->u.state->port.tty) {
    if (port->debugFlags & DEBUG_DATA) {
      unsigned int ttyinf = port->u.state ? (port->u.state->port.tty ? 3:1) : 0;
      printk("xuart Tx(%d) <%d>\n",port->u.line,ttyinf);
    }
    return;
  }
  if (port->u.state->port.flags & ASYNC_CTS_FLOW) {
    port->xu->hwcts |= (1<<port->u.line);
    port->xu->txcfg |= (0x3 << (port->u.line * 2));
  } else {
    port->xu->hwcts &= ~(1<<port->u.line);
    port->xu->txcfg |= (0x2 << (port->u.line * 2));
  }

  /* This is handled at a hardware level
  if (port->u.state->flags & ASYNC_CTS_FLOW) {
    if (*(port->rtscts) & (1 << (8+port->u.line))) { // looking at CTS
      port->u.state->port.tty->hw_stopped = 0;
    } else {
      port->u.state->port.tty->hw_stopped = 1;
      if (!timer_added) {
	timer_added = 1;
        if (port->debugFlags & DEBUG_INTERRUPT) {
	  printk("Adding timer (%d)\n",port->u.line);
	}
	timer.expires = jiffies + DELAY_TIME;
	init_timer(&timer);
	//add_timer(&timer);
      }
    }
  } else {
    // set HW CTS?
  }
  */
  xmit = &port->u.state->xmit;
  if (uart_circ_empty(xmit)) { // || uart_tx_stopped(&(port->u))) {
    return;
  }

  i = 0;
  //printk("%d:%d %p\n",xmit->tail,xmit->head,&i);
  while (i < 512) { // handle maximum of 512 characters at once
    // handle one character
    ch = xmit->buf[xmit->tail];
    if (xu_writec(port->xu,port->u.line,(unsigned char *)&ch,1) < 1) {
      break;
    }
    if (port->debugFlags & DEBUG_DATA) {
      unsigned int ttyinf = port->u.state ? (port->u.state->port.tty ? 3:1) : 0;
      printk("xuart Tx(%d):%02X:%d\n",port->u.line,ch,ttyinf);
    }
    xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
    port->u.icount.tx++;
    // done handling one character
    i++;
    if (uart_circ_empty(xmit)) { // || uart_tx_stopped(&(port->u))) {
      break;
    }
  }

  if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS) {
    uart_write_wakeup(&port->u);
  }
  if (uart_circ_empty(xmit)) {
    if (port->debugFlags & DEBUG_TXRX) {
      printk("xuart stopping xmit at end\n");
    }
    xuart_stop_tx(&port->u);
  }
}

//---------------------------------------------------------------------------
// TIMERS
//---------------------------------------------------------------------------

static void xuart_cts_timer(unsigned long data) {
  struct uart_port *port = (struct uart_port *)data;
  timer_added = 0;
  if (XPORT(port)->debugFlags & DEBUG_INTERRUPT) {  
    printk("xuart_cts_timer()\n");
  }
}

//---------------------------------------------------------------------------
// LINUX INTERFACE
//---------------------------------------------------------------------------

static void xuart_stop_tx(struct uart_port *port) {
  struct xuart_port *p = XPORT(port);
  int i=0;
  
  while (xu_draintx(p->xu,p->u.line) && i < 1000000) i++;

  //printk("xuart_stop_tx waited %d\n",i);

  if (XPORT(port)->debugFlags & DEBUG_INTERFACE) {  
    printk("xuart_stop_tx(%d)\n",port->line);
  }
}

static void xuart_start_tx(struct uart_port *port) {
  unsigned long flags;

  if (XPORT(port)->debugFlags & DEBUG_INTERFACE) {  
    printk("xuart_start_tx(%d)\n",port->line);
  }
  local_irq_save(flags);
  xuart_try_handleTx(XPORT(port));
  local_irq_restore(flags);
}

static void xuart_stop_rx(struct uart_port *port)
{
  if (XPORT(port)->debugFlags & DEBUG_INTERFACE) {
    printk("xuart_stop_rx(%d)\n",port->line);
  }
}

static void xuart_enable_ms(struct uart_port *port)
{
  if (XPORT(port)->debugFlags & DEBUG_INTERFACE) {
    printk("xuart_enable_ms(%d)\n",port->line);
  }
}

static unsigned int xuart_tx_empty(struct uart_port *port)
{
  int bit = 1; // no way to tell, so pretend it's so
  if (XPORT(port)->debugFlags & DEBUG_INTERFACE) {
    printk("xuart_tx_empty called(%d) = %d\n",port->line,bit);
  }
  return bit;
}

static unsigned int xuart_get_mctrl(struct uart_port *port) {
  struct xuart_port *p = XPORT(port);
  unsigned int result = 0;

  result |= TIOCM_CAR;  // pretend DCD is always set
  result |= TIOCM_DSR;  // pretend DSR is always set
  if (*(p->rtscts) & (1 << (8+p->u.line))) {
    result |= TIOCM_CTS;
  } else {
    result &= ~TIOCM_CTS;
  }
  if (p->debugFlags & DEBUG_INTERFACE) {
    printk("xuart_get_mctrl(%d) CTS=%d\n",port->line,
	   (result & TIOCM_CTS) ? 1 : 0);
  }
  return result;
}


static void xuart_set_mctrl(struct uart_port *port, unsigned int mctrl) {
  struct xuart_port *p = XPORT(port);

  if (p->debugFlags & DEBUG_INTERFACE) {
    printk("xuart_set_mctrl(%d,RTS=%d)\n",port->line,((mctrl&TIOCM_RTS) != 0) ? 1:0);
  }
  if (mctrl & TIOCM_RTS) {
    *(p->rtscts) |= (1 << (4+p->u.line));
  } else {
    *(p->rtscts) &= ~(1 << (4+p->u.line));
  }

}

static void xuart_break_ctl(struct uart_port *port, int break_state)
{
  struct xuart_port *p = XPORT(port);

  // We can't translate Linux's break interface exactly, as we need to
  // know ahead of time how long the break will be, so we will just always
  // send a 100ms break. Hopefully this is okay.
  if (break_state == -1) {
    xu_txbreak(p->xu,p->u.line,break_state);
  }
  if (p->debugFlags & DEBUG_INTERFACE) {
    printk("--- xuart_break_ctl(%d,%d)\n",port->line,break_state);
  }
}

static int xuart_startup(struct uart_port *port)
{
  if (XPORT(port)->debugFlags & DEBUG_INTERFACE) {
    printk("xuart_startup(%d), started=%d\n",port->line,started+1);
  }

  /* create our timer but don't submit it just yet */
  timer.data = (unsigned long )port;
  timer.function = xuart_cts_timer;

  if (timer2.function == NULL) {
    if (XPORT(port)->debugFlags & DEBUG_INTERRUPT) {
      printk("(NOT) Starting MDM timer\n");
    }
    timer2.expires = jiffies + HZ;
    //timer2.function = ???
    init_timer(&timer2);
    //add_timer(&timer2);
  } else {
    printk("MDM timer can't start!\n");
  }
  return 0;
}

static void xuart_shutdown(struct uart_port *port)
{
  struct xuart_port *p = XPORT(port);
  int i = 0; //,j=0,k=0;

  if (p->debugFlags & DEBUG_INTERFACE) {
    printk("xuart_shutdown(%d), started=%d,overflows=%d\n",port->line,started-1,port->icount.overrun);
  }

  while (xu_draintx(p->xu,p->u.line) && i < 1000000) i++;
  xu_close(p->xu,p->u.line);

  //printk("xu_draintx waited %d,%d,%d\n",i,j,k);
  if (timer_added) {
    del_timer (&timer);
    timer_added = 0;
  }
  if (timer2.function) {
    del_timer(&timer2);
    timer2.function = NULL;
  }
  port->state->xmit.tail = port->state->xmit.head; // avoid stuckness
}

static void xuart_change_speed(struct uart_port *port, 
				struct ktermios *new_termios, 
				struct ktermios *old_termios)
{
  int encoded_baud,baud,par=0,parodd=0,dbits=8,stop=1,xpar;
  int cflag = new_termios->c_cflag;
  int iflag = new_termios->c_iflag;
  int oflag = new_termios->c_oflag;
  char mode[4];
  struct xuart_port *p = XPORT(port);
  //int maxflush;

  mode[3] = 0;
  if (XPORT(port)->debugFlags & DEBUG_INTERFACE) {
    printk("xuart_change_speed called,cflag=0x%X,iflag=0x%X,oflag=0x%X\n",cflag,iflag,oflag);
  }

  encoded_baud = cflag & CBAUD;
  switch (encoded_baud) {
  //case B150:    baud = 150; break;
  case B300:    baud = 300; break;
  case B600:    baud = 600; break;
  case B1200:   baud = 1200; break;
  case B2400:   baud = 2400; break;
  case B4800:   baud = 4800; break;
  case B9600:   baud = 9600; break;
  case B19200:  baud = 19200; break;
  case B38400:  baud = 38400; break;
  case B57600:  baud = 57600; break;
  case B115200: baud = 115200; break;
  case B230400: baud = 230400; break;
  default: baud=0;
  }
  uart_update_timeout(port,cflag,baud);

  if (baud) {
    if (XPORT(port)->debugFlags & DEBUG_CONTROL) {
      printk("xuart:change_speed, port %d: %d baud\n",port->line,baud);
    }
  }
  if (cflag & PARENB) {
    par = 1;
    if (cflag & PARODD) {
      mode[1] = 'o';
    } else {
      mode[1] = 'e';
    }
    parodd = (cflag & PARODD) != 0;
  } else {
    mode[1] = 'n';
  }
  xpar = par ? (parodd ? XU_ODD : XU_EVEN) : XU_NONE;

  // byte size and parity
  switch (cflag & CSIZE) {
  case CS7:
    dbits = 7;
    mode[0] = '7';
    break;
  case CS8:
    dbits = 8;
    mode[0] = '8';
    break;
  default: // not CS8
    printk("xuart:unsupported byte size, using CS8\n");
    mode[0] = '8';
    break;
  }
  if (cflag & CSTOPB) {
    mode[2] = '2';
    stop = 2;
  } else {
    mode[2] = '1';
  }
  p->readahead = 1+baud/1000;
  xu_close(p->xu,p->u.line);

  if (xu_open(p->xu,p->u.line,mode,baud) < 0) {
    printk("ERROR setting XUART line parameters!\n");
  }
}

static void xuart_set_termios(struct uart_port *port,
		struct ktermios *termios, struct ktermios *old_termios)
{
  unsigned long port_flags;

  if (XPORT(port)->debugFlags & DEBUG_INTERFACE) {
    printk("set_termios called\n");
  }

  spin_lock_irqsave(&port->lock, port_flags);
  xuart_change_speed(port, termios, old_termios);
  spin_unlock_irqrestore(&port->lock, port_flags);

}

static const char *xuart_type(struct uart_port *port)
{
  return port->type == PORT_XUART ? "XUART" : NULL;
}

/*
 * Release the memory region(s) being used by 'port'
 */
static void xuart_release_port(struct uart_port *port)
{
  if (XPORT(port)->debugFlags & DEBUG_INTERFACE) {
    printk("xuart_release_port(%d)\n",port->line);
  }
  if (port->iotype == SERIAL_IO_MEM) {
    __iounmap(port->membase);
    port->membase = 0;
  }
}

/*
 * Request the memory region(s) being used by 'port'
 */
static int xuart_request_port(struct uart_port *port) {
  //int max;
  if (XPORT(port)->debugFlags & DEBUG_INTERFACE) {
    printk("xuart_request_port(%d)\n",port->line);
  }
  if (port->iotype == SERIAL_IO_MEM) {
    if (port->membase == 0) {
      port->membase = (void *) __arm_ioremap(port->mapbase & 0xFFFFF000, 4096,0)
	+ (port->mapbase & 0xFFF);
      //printk("port %p:mapbase=%X, membase=%X\n",port,port->mapbase,port->membase);
      if (port->membase == (void *)(port->mapbase & 0xFFF)) {
	port->membase = 0;
	printk(KERN_ERR "xuart: cannot map io memory\n");
	return -ENOMEM;
      }
    }
  }
  return 0;
}

/*
 * Configure/autoconfigure the port.
 */
static void xuart_config_port(struct uart_port *port, int flags)
{
  if (XPORT(port)->debugFlags & DEBUG_INTERFACE) {
    printk("xuart_config_port(%d,%X)\n",port->line,flags);
  }
  if (flags & UART_CONFIG_TYPE) {
    port->type = PORT_XUART;
    xuart_request_port(port);
  }
}

/*
 * verify the new serial_struct (for TIOCSSERIAL).
 */
static int xuart_verify_port(struct uart_port *port, struct serial_struct *ser)
{
  int ret = 0;

  if (port == NULL) return -EINVAL; // avoid warning
  if (ser->type != PORT_UNKNOWN && ser->type != PORT_XUART)
    ret = -EINVAL;
  if (ser->irq < 0 || ser->irq >= NR_IRQS)
    ret = -EINVAL;
  if (XPORT(port)->debugFlags & DEBUG_INTERFACE) {
    printk("xuart_verify_port(%d,%d)=%d\n",port->line,ser->irq,ret);
  }
  return ret;
}

static int xuart_ioctls (struct uart_port *port, unsigned int cmd, unsigned long arg)
{
  return -ENOIOCTLCMD;
}


//---------------------------------------------------------------------------
// Interrupt Routine
//---------------------------------------------------------------------------
static irqreturn_t xuart_interrupt(int irq, void *dev_id) {
  struct xuart_port *port = (struct xuart_port *)dev_id;

  if (port->debugFlags & DEBUG_INTERRUPT) {  
    printk("xuart_interrupt()\n");
  }

  // port assumed to be first in list. this should be true, as we should
  // allocate the irq to the first port in the list.
  xu_irq(port->xu);
  while (port) {
    xuart_try_handleTx(port);
    xuart_try_handleRx(port);
    port = port->next;
  }

  return IRQ_HANDLED;
}

//---------------------------------------------------------------------------
// CORE LOGIC
//---------------------------------------------------------------------------
static struct uart_ops xuart_ops = {
	.stop_tx	= xuart_stop_tx,
	.start_tx	= xuart_start_tx,
	.stop_rx	= xuart_stop_rx,
	.enable_ms	= xuart_enable_ms,
 	.tx_empty	= xuart_tx_empty,
	.get_mctrl	= xuart_get_mctrl,
	.set_mctrl	= xuart_set_mctrl,
	.break_ctl	= xuart_break_ctl,
	.startup	= xuart_startup,
	.shutdown	= xuart_shutdown,
	.set_termios	= xuart_set_termios,
	.type		= xuart_type,
	.release_port	= xuart_release_port,
	.request_port	= xuart_request_port,
	.config_port	= xuart_config_port,
	.verify_port	= xuart_verify_port,
	.ioctl		= xuart_ioctls,
};

static struct uart_driver xuart_reg;

int xuart_configure_driver(struct uart_driver *dr,int major,int minor,
			    char *tty,int maxports) {
  memset(dr,0,sizeof(struct uart_driver));
  dr->owner = THIS_MODULE;
  dr->major = major;
  dr->dev_name = tty;
  dr->minor = minor;
  dr->nr = maxports;
  dr->cons = NULL;
  //  printk("Registering UART driver %p with minor %d\n",dr,dr->minor);
  if (uart_register_driver(dr)) {
    printk("Unable to register driver\n");
    return 0;
  }
  return 1;
}

int xuart_register_port(struct uart_driver *driver,
				struct xuart_port *port) {
  struct uart_port *p = &port->u;
  int ret=0;

  if (!driver) {
    driver = &xuart_reg;
  }
  if ((ret = uart_add_one_port(driver,p)) == 0) {
    if (port->debugFlags & DEBUG_ENDS) {
      printk("registered port %p\n",port);
    }    
    request_irq(port->u.irq, xuart_interrupt, 
		IRQF_SHARED, 
		"xuart_uarts", p);
    if (ret != 0) {
      printk("Unable to allocate irq %d\n",port->u.irq);
      ret = 0; // for testing
    } else {
      if (port->debugFlags & DEBUG_INTERRUPT) {
	printk("Allocated IRQ %d\n",port->u.irq);
      }
    }
  } else {
    if (port->debugFlags & DEBUG_ENDS) {
      printk("Failed to register port %p with driver %p, ret=%d\n",port,driver,ret);
    }
    return -1;
  }
  return ret;
}

//---------------------------------------------------------------------------
// KERNEL MODULE
//---------------------------------------------------------------------------

#define SERIAL_XUART_MAJOR	235	// in unassigned range (231-239)
#define SERIAL_XUART_MINOR	1
#define TTY_NAME "ttz"

static int portDebug=0;
module_param(portDebug,int,0644);

/* USE THIS:
int maskirq(void *a, int m) {
       static int state = 0;
       int r;

       if (m) disable_irq(XUART_IRQ);
       else enable_irq(XUART_IRQ);
       r = state;

       state = m;
       return r;
} 
*/
int linux_maskirq(void *dummy,int mask) {
  static unsigned long flags;
  static int masked = 0;

  if (mask) {
    if (!masked) {
      local_irq_save(flags);
    }
    masked++;
  } else if (!mask) {
    masked--;
    if (!masked) {
      local_irq_restore(flags);
    }
  }
  return !mask;
}

static int __init xuart_init(void)
{
  struct xuart_port *ts0 = 0;
  struct xuartcore *xu;

  xuart_configure_driver(&xuart_reg,SERIAL_XUART_MAJOR,SERIAL_XUART_MINOR,
                          TTY_NAME,64);

  xu = kmalloc(sizeof(struct xuartcore),GFP_KERNEL);
  if (!xu) {
    printk("Unable to allocate memory for xuartcore\n");
  }
  memset(xu,0,sizeof(struct xuartcore));
  xu->xu_regstart = ((unsigned int)__arm_ioremap(0x600FF100, 4096,0));
  xu->xu_memstart = __arm_ioremap(0x60000000, 8192,0);
  xu->xu_maskirq = linux_maskirq;
  if (xu_reset(xu) != 2) {
    printk("Failed to initialize xuartcore\n");
    return -1;
  }

  ts0 = ts = kmalloc(sizeof(struct xuart_port),GFP_KERNEL);
  if (!ts) {
    printk("Unable to allocate memory for xuart port\n");
    return -1;
  }
  memset(ts,0,sizeof(struct xuart_port));
  ts->u.irq = 0;
  ts->u.uartclk = 0; // unused
  ts->u.fifosize = 0; // unused
  ts->u.ops = 0;
  ts->u.flags = ASYNC_BOOT_AUTOCONF;
  ts->u.line = 0;
  ts->u.iobase = 0;
  ts->u.membase = 0;
  ts->u.mapbase = 0;


  ts->u.iotype = SERIAL_IO_MEM;
  ts->u.mapbase = 0x600FF100;
  ts->u.irq = 33;
  ts->u.ops = &xuart_ops;
  ts->debugFlags = portDebug;
  ts->next = 0;
  ts->xu = xu;
  ts->rtscts = ((void *) __arm_ioremap(0x6000f000, 4096,0) + 0x88);

  if (xuart_register_port(0,ts) != 0) {
    printk("Failed to register port!\n");
    return 0;
  }

  ts0->next = ts = kmalloc(sizeof(struct xuart_port),GFP_KERNEL);
  if (!ts) {
    printk("Unable to allocate memory for xuart port\n");
    return -1;
  }
  memset(ts,0,sizeof(struct xuart_port));
  ts->u.irq = 0;
  ts->u.uartclk = 0; // unused
  ts->u.fifosize = 0; // unused
  ts->u.ops = 0;
  ts->u.flags = ASYNC_BOOT_AUTOCONF;
  ts->u.line = 1;
  ts->u.iobase = 0;
  ts->u.membase = 0;
  ts->u.mapbase = 0;


  ts->u.iotype = SERIAL_IO_MEM;
  ts->u.mapbase = 0x6000F200;
  ts->u.irq = 33;
  ts->u.ops = &xuart_ops;
  ts->debugFlags = portDebug;
  ts->next = 0;
  ts->xu = xu;
  ts->rtscts = ts0->rtscts;

  if (xuart_register_port(0,ts) != 0) {
    printk("Failed to register port!\n");
    return -1;
  }
  return 0;
}

static void __exit xuart_exit(void)
{
}

module_init(xuart_init);
module_exit(xuart_exit);

MODULE_AUTHOR("Technologic Systems");
MODULE_DESCRIPTION("Technologic Systems TS-UART driver");
MODULE_LICENSE("GPL");  // or BSD.  Take your pick!

