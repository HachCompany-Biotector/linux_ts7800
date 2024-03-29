/*******************************************************************************
Copyright (C) Marvell International Ltd. and its affiliates

This software file (the "File") is owned and distributed by Marvell 
International Ltd. and/or its affiliates ("Marvell") under the following
alternative licensing terms.  Once you have made an election to distribute the
File under one of the following license alternatives, please (i) delete this
introductory statement regarding license alternatives, (ii) delete the two
license alternatives that you have not elected to use and (iii) preserve the
Marvell copyright notice above.


********************************************************************************
Marvell GPL License Option

If you received this File from Marvell, you may opt to use, redistribute and/or 
modify this File in accordance with the terms and conditions of the General 
Public License Version 2, June 1991 (the "GPL License"), a copy of which is 
available along with the File in the license.txt file or by writing to the Free 
Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 or 
on the worldwide web at http://www.gnu.org/licenses/gpl.txt. 

THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED 
WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY 
DISCLAIMED.  The GPL License provides additional details about this warranty 
disclaimer.
*******************************************************************************/
/*******************************************************************************
* mvSysHwCfg.h - Marvell system HW configuration file
*
* DESCRIPTION:
*       None.
*
* DEPENDENCIES:
*       None.
*
*******************************************************************************/

#ifndef __INCmvSysHwConfigh
#define __INCmvSysHwConfigh

#include <linux/autoconf.h>

#ifndef MV_88F5181
#       define MV_88F5181
#endif

// GPP registers offsets */
   
#define GPP_DATA_OUT_REG		0x10100
#define GPP_DATA_OUT_EN_REG		0x10104
#define GPP_BLINK_EN_REG		0x10108
#define GPP_DATA_IN_POL_REG		0x1010C
#define GPP_DATA_IN_REG			0x10110
#define GPP_INT_CAUSE_REG		0x10114
#define GPP_INT_MASK_REG		0x10118
#define GPP_INT_LVL_REG			0x1011c

/* includes */
#define _1K         0x00000400
#define _4K         0x00001000
#define _8K         0x00002000
#define _16K        0x00004000
#define _32K        0x00008000
#define _64K        0x00010000
#define _128K       0x00020000
#define _256K       0x00040000
#define _512K       0x00080000

#define _1M         0x00100000
#define _2M         0x00200000
#define _4M         0x00400000
#define _8M         0x00800000
#define _16M        0x01000000
#define _32M        0x02000000
#define _64M        0x04000000
#define _128M       0x08000000
#define _256M       0x10000000
#define _512M       0x20000000

#define _1G         0x40000000
#define _2G         0x80000000

/* 
 *  System memory mapping 
 */


/* SDRAM: actual mapping is auto detected */
#define SDRAM_CS0_BASE  0x00000000
#define SDRAM_CS0_SIZE  _256M

#define SDRAM_CS1_BASE  0x10000000
#define SDRAM_CS1_SIZE  _256M

#define SDRAM_CS2_BASE  0x20000000
#define SDRAM_CS2_SIZE  _256M

#define SDRAM_CS3_BASE  0x30000000
#define SDRAM_CS3_SIZE  _256M

/* PEX */
#define PEX0_MEM_BASE 0xe0000000
#define PEX0_MEM_SIZE _128M

#define PEX0_IO_BASE 0xf2000000
#define PEX0_IO_SIZE _1M

/* used for WA */
#define PEX_CONFIG_RW_WA_TARGET DEVICE_CS1
#define PEX_CONFIG_RW_WA_USE_ORIGINAL_WIN_VALUES 0
#define PEX_CONFIG_RW_WA_SIZE _16M
#define PEX_CONFIG_RW_WA_BASE 0xf0000000

#if defined(MV_88F5182) || defined(MV_88F5181L)
#define CRYPT_ENG_BASE  0xf0000000
#define CRYPT_ENG_SIZE	_64K
#endif

/* PCI0: IO and memory space */
#define PCI0_MEM_BASE  0xe8000000
#define PCI0_MEM_SIZE  _128M

#define PCI0_IO_BASE    0xf2100000
#define PCI0_IO_SIZE    _1M

/* Device: CS0 - SRAM, CS1 - RTC, CS2 - UART/VOIP, CS3 - large flash */
#define DEVICE_CS0_BASE 0xfa000000
#define DEVICE_CS0_SIZE _2M

#define DEVICE_CS1_BASE 0xf4000000
#define DEVICE_CS1_SIZE _32M

#define DEVICE_CS2_BASE 0xfa800000
#define DEVICE_CS2_SIZE _1M

#define LARGE_FLASH_BASE 	DEVICE_CS1_BASE

/* Internal registers: size is defined in Controllerenvironment */
#define INTER_REGS_BASE	0xF1000000

#define BOOTDEV_CS_BASE	0xff800000
#define BOOTDEV_CS_SIZE _8M

/* DRAM detection stuff */
#define MV_DRAM_AUTO_SIZE

#define DRAM_DETECT_FLAG_ADDR   0x03000000
#define DRAM_CONFIG_ROM_ADDR    0x03000004
 
/* We use the following registers to store DRAM interface pre configuration   */
/* auto-detection results
          */
/* IMPORTANT: We are using mask register for that purpose. Before writing     */
/* to units mask register, make sure main maks register is set to disable     */
/* all interrupts.                                                            */
#define DRAM_BUF_REG0   0x1011c /* sdram bank 0 size            */
#define DRAM_BUF_REG1   0x20318 /* sdram config                         */
#define DRAM_BUF_REG2   0x20114 /* sdram mode                           */
#define DRAM_BUF_REG3   0x20320 /* dunit control low            */
#define DRAM_BUF_REG4   0x20404 /* sdram address control        */
#define DRAM_BUF_REG5   0x2040c /* sdram timing control low     */
#define DRAM_BUF_REG6   0x40108 /* sdram timing control high    */
#define DRAM_BUF_REG7   0x40114 /* sdram ODT control low        */
#define DRAM_BUF_REG8   0x41910 /* sdram ODT control high       */
#define DRAM_BUF_REG9   0x41a08 /* sdram Dunit ODT control      */
#define DRAM_BUF_REG10  0x41a30 /* sdram Extended Mode              */
 
/* Following the pre-configuration registers default values restored after    */
/* auto-detection is done                                                     */
#define DRAM_BUF_REG0_DV    0           /* GPIO Interrupt Level Mask Reg      */
#define DRAM_BUF_REG1_DV        0           /* ARM Timer 1 reload register        */
#define DRAM_BUF_REG2_DV    0           /* AHB to MBUS Bridge int Mask Reg    */
#define DRAM_BUF_REG3_DV        0           /* ARM Watchdog Timer Register        */
#define DRAM_BUF_REG4_DV        0           /* Host to ARM Doorbel Mask Register  */
#define DRAM_BUF_REG5_DV        0           /* ARM To Host Doorbel Mask Register  */
#define DRAM_BUF_REG6_DV        0           /* PCI Exp Uncorrectable Err Mask Reg */
#define DRAM_BUF_REG7_DV        0           /* PCI Exp Correctable Err Mask Reg   */
#define DRAM_BUF_REG8_DV        0           /* PCI Express interrupt Mask Reg     */
#define DRAM_BUF_REG9_DV        0           /* PCI Express Spare Register         */
#define DRAM_BUF_REG10_DV       0x012C0004  /* PCI Exp Acknowledge Timers (x4) Reg*/

/* PCI stuff */
/* Local bus and device number of PCI0/1*/
#define PCI_HOST_BUS_NUM		0
#define PCI_HOST_DEV_NUM		0

#define PCI_ARBITER_CTRL    /* Use/unuse the Marvell integrated PCI arbiter	*/
#undef	PCI_ARBITER_BOARD	/* Use/unuse the PCI arbiter on board			*/

/* Check macro validity */
#if defined(PCI_ARBITER_CTRL) && defined (PCI_ARBITER_BOARD)
	#error "Please select either integrated PCI arbiter or board arbiter"
#endif

/* Pex\PCI stuff */
#define PEX0_HOST_BUS_NUM               0
#define PEX0_HOST_DEV_NUM               0
#define PCI0_HOST_BUS_NUM               1
#define PCI0_HOST_DEV_NUM               0
                                                                                                                             
/* no pci1 in MV_88F5181 */
#define PCI1_HOST_BUS_NUM               0
#define PCI1_HOST_DEV_NUM               0
/* no pex1 in MV_88F5181 */
#define PEX1_HOST_BUS_NUM               0
#define PEX1_HOST_DEV_NUM               0

/* Board clock detection */
#define TCLK_AUTO_DETECT    /* Use Tclk auto detection 		*/
#define SYSCLK_AUTO_DETECT	/* Use SysClk auto detection 	*/
#define PCLCK_AUTO_DETECT  /* Use PClk auto detection */


/* Memory uncached, HW or SW cache coherency is not needed */
#define MV_UNCACHED             0   
/* Memory cached, HW cache coherency supported in WriteThrough mode */
#define MV_CACHE_COHER_HW_WT    1
/* Memory cached, HW cache coherency supported in WriteBack mode */
#define MV_CACHE_COHER_HW_WB    2
/* Memory cached, No HW cache coherency, Cache coherency must be in SW */
#define MV_CACHE_COHER_SW       3


/****************************************************************/
/************* Ethernet driver configuration ********************/
/****************************************************************/

/* IRQ numbers */
#ifndef ETH_PORT0_IRQ_NUM
# define ETH_PORT0_IRQ_NUM	21
#endif
#ifndef ETH_PORT1_IRQ_NUM
# define ETH_PORT1_IRQ_NUM	-1
#endif
#ifndef ETH_PORT2_IRQ_NUM
# define ETH_PORT2_IRQ_NUM	-1
#endif

/* MTU */
#ifndef CONFIG_ETH_0_MTU
# define CONFIG_ETH_0_MTU	1500
#endif
#ifndef CONFIG_ETH_1_MTU
# define CONFIG_ETH_1_MTU	1500
#endif
#ifndef CONFIG_ETH_2_MTU
# define CONFIG_ETH_2_MTU	1500
#endif

/* MAC */
#ifndef CONFIG_ETH_0_MACADDR
# define CONFIG_ETH_0_MACADDR	"000000000050"
#endif
#ifndef CONFIG_ETH_1_MACADDR
# define CONFIG_ETH_1_MACADDR	"000000000051"
#endif
#ifndef CONFIG_ETH_2_MACADDR
# define CONFIG_ETH_2_MACADDR	"000000000052"
#endif

/* port's default queueus */
#define EGIGA_DEF_TXQ 0
#define EGIGA_DEF_RXQ 0

/* interrupt coalescing setting */
#define EGIGA_TX_COAL    200
#define EGIGA_RX_COAL    200

/* Multi-queue support */
#ifdef CONFIG_EGIGA_MULTI_Q
#define INCLUDE_MULTI_QUEUE 1
#else
#undef INCLUDE_MULTI_QUEUE
#endif

#define ETH_DESCR_UNCACHED
/* Checksum offloading */
#ifndef CONFIG_MV_ETH_HEADER
#define RX_CSUM_MIN_BYTE_COUNT 72
#ifndef CONFIG_QUARTER_DECK
#define TX_CSUM_OFFLOAD
#define RX_CSUM_OFFLOAD
#endif
#endif

/* Descriptors location: DRAM/internal-SRAM */
#define ETH_DESCR_IN_SDRAM
#undef  ETH_DESCR_IN_SRAM    /* No integrated SRAM in 88Fxx81 devices */
#if defined(ETH_DESCR_IN_SRAM)
 #define ETH_DESCR_CONFIG_STR    "Ethernet descriptors in integrated SRAM"
#elif defined(ETH_DESCR_IN_SDRAM)
 #define ETH_DESCR_CONFIG_STR    "Ethernet descriptors in DRAM"
#else 
 #error "Ethernet descriptors location undefined"
#endif /* ETH_DESCR_IN_SRAM or ETH_DESCR_IN_SDRAM*/

/* SW Sync-Barrier: not relevant for 88fxx81*/
/* Reasnable to define this macro when descriptors in SRAM and buffers in DRAM */
/* In RX the CPU theoretically might see himself as the descriptor owner,      */
/* although the buffer hadn't been written to DRAM yet. Performance cost.      */
/* #define INCLUDE_SYNC_BARR */

/* Buffers cache coherency method (buffers in DRAM) */
#define ETHER_DRAM_COHER    MV_CACHE_COHER_SW   /* No HW coherency in 88Fxx81 devices */
#if (ETHER_DRAM_COHER == MV_CACHE_COHER_HW_WB)
 #define ETH_SDRAM_CONFIG_STR    "DRAM HW cache coherency (write-back)"
#elif (ETHER_DRAM_COHER == MV_CACHE_COHER_HW_WT)
 #define ETH_SDRAM_CONFIG_STR    "DRAM HW cache coherency (write-through)"
#elif (ETHER_DRAM_COHER == MV_CACHE_COHER_SW)
 #define ETH_SDRAM_CONFIG_STR    "DRAM SW cache-coherency"
#elif (ETHER_DRAM_COHER == MV_UNCACHED)
#   define ETH_SDRAM_CONFIG_STR  "DRAM uncached"
#else
 #error "Ethernet-DRAM undefined"
#endif /* ETHER_DRAM_COHER */

#define MV_ETH_TX_Q_NUM		1

/* port's default queueus */
#define ETH_DEF_TXQ         0
#define ETH_DEF_RXQ         0 

//#endif /* CONFIG_MV_INCLUDE_GIG_ETH || CONFIG_MV_INCLUDE_UNM_ETH */

#ifdef CONFIG_MV_INCLUDE_UNM_ETH

/* interrupt coalescing setting */
#define ETH_TX_COAL    0
#define ETH_RX_COAL    0

#endif /* CONFIG_MV_INCLUDE_UNM_ETH */

//#ifdef CONFIG_MV_INCLUDE_GIG_ETH
/****************************************************************/
/************* Ethernet driver configuration ********************/
/****************************************************************/

/* interrupt coalescing setting */
#define ETH_TX_COAL    200
#if defined CONFIG_MV88F6082
#define ETH_RX_COAL    1000
#else
#define ETH_RX_COAL    200
#endif

#define ETH_INCLUDE_TSO


/*#define ETH_TX_DONE_ISR*/

/* Delay [usec] between 2 writes to ETH_TX_QUEUE_COMMAND_REG */
/* 0 means NO delay */
#define ETH_TX_UDELAY           1

/* Perion [msec] for periodic TX timer. 
 * This timer will enable TX queue to prevent stack of last packet, and
 * will care tx_done functionality.
 */
#define ETH_TX_TIMER_PERIOD     10

/* Checksum offloading */
#define TX_CSUM_OFFLOAD
#define RX_CSUM_OFFLOAD

//#endif /* CONFIG_MV_INCLUDE_GIG_ETH */


#ifdef CONFIG_MV_GATEWAY

#ifdef INCLUDE_MULTI_QUEUE
#define MV_ETH_RX_Q_NUM		    4
#define ETH_NUM_OF_RX_DESCR     64
#define ETH_NUM_OF_TX_DESCR     2000
#define ETH_RX_QUEUE_QUOTA	    32   /* quota per Rx Q */
#else
#define MV_ETH_RX_Q_NUM		    1
#define ETH_NUM_OF_RX_DESCR     128
#define ETH_NUM_OF_TX_DESCR     ETH_NUM_OF_RX_DESCR*2
#endif /* INCLUDE_MULTI_QUEUE */

#elif defined(CONFIG_MV88F6082)
#ifdef INCLUDE_MULTI_QUEUE
#define MV_ETH_RX_Q_NUM             8
#define ETH_NUM_OF_RX_DESCR     64
#define ETH_NUM_OF_TX_DESCR     ETH_NUM_OF_RX_DESCR*MV_ETH_RX_Q_NUM
#define ETH_RX_QUEUE_QUOTA          32   /* quota per Rx Q */
#else
#define MV_ETH_RX_Q_NUM             1
#define ETH_NUM_OF_RX_DESCR     64
#define ETH_NUM_OF_TX_DESCR     ETH_NUM_OF_RX_DESCR*2
#endif /* INCLUDE_MULTI_QUEUE */

#else /* CONFIG_MV_ETHERNET */
#ifdef INCLUDE_MULTI_QUEUE
 #if defined(CONFIG_MV_INCLUDE_UNM_ETH) /* UNIMAC */
  #define MV_ETH_RX_Q_NUM		4
 #else
  #define MV_ETH_RX_Q_NUM		8
 #endif
#define ETH_NUM_OF_RX_DESCR     64
#define ETH_NUM_OF_TX_DESCR     ETH_NUM_OF_RX_DESCR*MV_ETH_RX_Q_NUM
#define ETH_RX_QUEUE_QUOTA	    32   /* quota per Rx Q */
#else

#define MV_ETH_RX_Q_NUM		    1

#if defined CONFIG_MV88F6082
#   define ETH_NUM_OF_RX_DESCR     64
#else 
#   define ETH_NUM_OF_RX_DESCR     128
#endif /* CONFIG_MV88F6082 */

#define ETH_NUM_OF_TX_DESCR     ETH_NUM_OF_RX_DESCR*2
#endif /* INCLUDE_MULTI_QUEUE */

#endif /* CONFIG_MV_GATEWAY */
/****************************************************************/
/*************** Sata driver configuration **********************/
/****************************************************************/

/* IRQ numbers */
#ifndef SATA_IRQ_NUM
# define SATA_IRQ_NUM	29
#endif


			   
#endif /* __INCmvSysHwConfigh */
