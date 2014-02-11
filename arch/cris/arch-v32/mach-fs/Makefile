# $Id: Makefile,v 1.1.1.1 2010/10/22 17:05:50 ian Exp $
#
# Makefile for the linux kernel.
#

obj-y   := dma.o pinmux.o io.o arbiter.o
bj-$(CONFIG_ETRAX_VCS_SIM) += vcs_hook.o
obj-$(CONFIG_CPU_FREQ)   += cpufreq.o

clean:

