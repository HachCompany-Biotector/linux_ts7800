#*******************************************************************************
# Copyright (C) Marvell International Ltd. and its affiliates
#
# This software file (the "File") is owned and distributed by Marvell 
# International Ltd. and/or its affiliates ("Marvell") under the following
# alternative licensing terms.  Once you have made an election to distribute the
# File under one of the following license alternatives, please (i) delete this
# introductory statement regarding license alternatives, (ii) delete the two
# license alternatives that you have not elected to use and (iii) preserve the
# Marvell copyright notice above.
#
#
#*******************************************************************************
# Marvell GPL License Option
#
# If you received this File from Marvell, you may opt to use, redistribute and/or 
# modify this File in accordance with the terms and conditions of the General 
# Public License Version 2, June 1991 (the "GPL License"), a copy of which is 
# available along with the File in the license.txt file or by writing to the Free 
# Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 or 
# on the worldwide web at http://www.gnu.org/licenses/gpl.txt. 
#
# THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED 
# WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY 
# DISCLAIMED.  The GPL License provides additional details about this warranty 
# disclaimer.
#*******************************************************************************/
include $(TOPDIR)/arch/arm/mach-mv88fxx81/config/mvRules.mk

# Objects list

COMMON_OBJS        = $(COMMON_DIR)/mvDebug.o 		\
					 $(COMMON_DIR)/mvCommon.o

OSSERVICES_OBJS    = $(OSSERV_LINUX)/mvOsLinux.o 	\
					 $(OSSERV_ARCH_DIR)/mvLinuxArm.o

BOARD_OBJS      = $(BOARD_ENV_DIR)/mvBoardEnvLib.o      \
                  $(BOARD_ENV_PLAT_DIR)/mvBoardEnvSpec.o \
                  $(BOARD_ETHPHY_DIR)/mvEthPhy.o

CONTROLLER_OBJS = $(SOC_CPUIF_DIR)/mvCpuIf.o 			\
				  $(SOC_ENV_DIR)/mvCtrlEnvLib.o        	\
                  $(SOC_ENV_DIR)/mvCtrlEnvAddrDec.o 	\
				  $(SOC_TWSI_DIR)/mvTwsi.o           	\
                  $(SOC_AHB_TO_MBUS_DIR)/mvAhbToMbus.o 	\
				  $(SOC_DRAM_DIR)/mvDramIf.o     		\
                  $(SOC_PCI_DIR)/mvPci.o 				\
				  $(SOC_PEX_DIR)/mvPex.o 				\
				  $(SOC_PCIIF_DIR)/mvPciIf.o 			\
                  $(SOC_IDMA_DIR)/mvIdma.o 				\
				  $(SOC_ETH_DIR)/mvEth.o 				\
				  $(SOC_ETH_DIR)/mvEthAddrDec.o			

BOARD_OBJS += $(BOARD_DIR)/buffalo/BuffaloUart.o

LSP_OBJS        = $(LSP_DIR)/core.o 	\
				  $(LSP_DIR)/irq.o 		\
				  $(LSP_DIR)/mm.o

ifeq ($(CONFIG_MV_DMA_COPYUSER),y)
LSP_OBJS +=  $(LSP_DIR)/dma.o 
endif

obj-y           := mv88f5181.o
mv88f5181-objs  := $(LSP_OBJS) $(COMMON_OBJS) $(OSSERVICES_OBJS) $(BOARD_OBJS) $(CONTROLLER_OBJS)

mv88f5181-$(CONFIG_MV_CESA) += $(SOC_CESA_DIR)/mvCesa.o 		\
							   $(SOC_CESA_DIR)/mvCesaDebug.o 	\
							   $(SOC_CESA_DIR)/mvMD5.o 			\
							   $(SOC_CESA_DIR)/mvSHA1.o 		\
							   $(SOC_CESA_DIR)/mvLru.o 			\
							   $(SOC_CESA_AES_DIR)/mvAesAlg.o	\
							   $(SOC_CESA_AES_DIR)/mvAesApi.o
mv88f5181-$(CONFIG_MV_CESA_TEST) += $(SOC_CESA_DIR)/mvCesaTest.o
mv88f5181-$(CONFIG_PCI)  += $(LSP_DIR)/pci.o



