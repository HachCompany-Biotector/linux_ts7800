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
#include <asm/hardware.h>
#include <asm/bitops.h>
#include <asm/io.h>

static struct uart_port req;
static unsigned int baseio = 0x11e00000;
static unsigned int io = 0x3e8;
static unsigned int irq = 33;

static int __init
ts72xx_isa16550_init (void)
{
	int line;

	req.type = PORT_16550A;
	req.iotype = UPIO_MEM;
	req.iobase = 0;
	req.fifosize = 16;
	req.flags = UPF_IOREMAP | UPF_SHARE_IRQ;
	req.regshift = 0;
	req.irq = irq;
	req.membase = (char *)(baseio + io);
	req.mapbase = baseio + io;
	req.uartclk = 1843200;
	line = serial8250_register_port(&req);
	return 0;
}

static void __exit ts72xx_isa16550_exit(void)
{
}

module_init(ts72xx_isa16550_init);
module_exit(ts72xx_isa16550_exit);
module_param(baseio, uint, 0644);
module_param(io, uint, 0644);
module_param(irq, uint, 0644);
MODULE_LICENSE("GPL");

