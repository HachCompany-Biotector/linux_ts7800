#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/serial_8250.h>
#include <linux/reboot.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <mach/hardware.h>
#include <asm/bitops.h>
#include <asm/io.h>

static struct uart_port req;
static unsigned int io = 0x3e8;
static unsigned int irq = 6;

static int __init
ts7800_isa16550_init (void)
{
	int line;

	req.type = PORT_16550A;
	req.iotype = UPIO_MEM;
	req.iobase = 0;
	req.fifosize = 16;
	req.flags = UPF_IOREMAP | UPF_SHARE_IRQ;
	req.regshift = 0;
	req.irq = 64 + irq;
	req.membase = (char *)(0xee000000UL + io);
	req.mapbase = 0xee000000 + io;
	req.uartclk = 1843200;
	line = serial8250_register_port(&req);
	return 0;
}

static void __exit ts7800_isa16550_exit(void)
{
   
}

module_init(ts7800_isa16550_init);
module_exit(ts7800_isa16550_exit);
module_param(io, uint, 0644);
module_param(irq, uint, 0644);
MODULE_LICENSE("GPL");
