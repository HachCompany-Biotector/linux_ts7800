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
Marvell Commercial License Option

If you received this File from Marvell and you have entered into a commercial
license agreement (a "Commercial License") with Marvell, the File is licensed
to you under the terms of the applicable Commercial License.

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
********************************************************************************
Marvell BSD License Option

If you received this File from Marvell, you may opt to use, redistribute and/or 
modify this File under the following licensing terms. 
Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    *   Redistributions of source code must retain the above copyright notice,
	    this list of conditions and the following disclaimer. 

    *   Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution. 

    *   Neither the name of Marvell nor the names of its contributors may be 
        used to endorse or promote products derived from this software without 
        specific prior written permission. 
    
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR 
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

/*******************************************************************************
* mvDramIfBasicAsm.s
*
* DESCRIPTION:
*       Memory full detection and best timing configuration is done in 
*       C code. C runtime environment requires a stack. This module API 
*       initialize DRAM interface chip select 0 for basic functionality for 
*       the use of stack.
*       The module API assumes DRAM information is stored in I2C EEPROM reside
*       in a given I2C address MV_BOARD_DIMM0_I2C_ADDR. The I2C EEPROM 
*       internal data structure is assumed to be orgenzied in common DRAM 
*       vendor SPD structure.
*       NOTE: DFCDL values are assumed to be already initialized prior to 
*       this module API activity.
*       
*
* DEPENDENCIES:
*       None.
*
*******************************************************************************/

/* includes */
#define MV_ASMLANGUAGE
#include "mvOsAsm.h"
#include "mvSysHwConfig.h"
#include "mvDramIfRegs.h"
#include "mvDramIfConfig.h"
#include "mvCpuIfRegs.h"
#include "mvPexRegs.h"
#include "mvCtrlEnvSpec.h"

/* defines  */

/* locals   */
.data
.globl _mvDramIfConfig

.text

/*******************************************************************************
* _mvDramIfConfig - Basic DRAM interface initialization.
*
* DESCRIPTION:
*       The function will initialize the following DRAM parameters using the
*       values prepared by mvDramIfDetect routine. Values are located
*       in predefined registers.
*
* INPUT:
*       None.
*
* OUTPUT:
*       None.
*
* RETURN:
*       None.
*
*******************************************************************************/

_mvDramIfConfig:      
        
        /* Save register on stack */
	cmp	sp, #0
	beq	no_stack_s
save_on_stack:
        stmdb	sp!, {r1, r2, r3, r4}
no_stack_s:

	/* 1) Write to SDRAM coniguration register */         
        ldr     r1, =(INTER_REGS_BASE + DRAM_BUF_REG1)
        ldr     r4, [r1]
        ldr     r1, =(INTER_REGS_BASE + SDRAM_CONFIG_REG)
        str     r4, [r1]
        
	/* 2) Write Dunit control low register */ 
        ldr     r1, =(INTER_REGS_BASE + DRAM_BUF_REG3)
        ldr     r4, [r1]
        ldr     r1, =(INTER_REGS_BASE + SDRAM_DUNIT_CTRL_REG)
        str     r4, [r1]
        
        /* 3) Write SDRAM address control register */ 
        ldr     r1, =(INTER_REGS_BASE + DRAM_BUF_REG4)
        ldr     r4, [r1]
        ldr     r1, =(INTER_REGS_BASE + SDRAM_ADDR_CTRL_REG)
        str     r4, [r1]
        
        /* 4) Write SDRAM bank 0 size register */ 
        ldr     r1, =(INTER_REGS_BASE + DRAM_BUF_REG0)
        ldr     r4, [r1]
        ldr     r1, =(INTER_REGS_BASE + SDRAM_SIZE_REG(0))
        str     r4, [r1]
                          
        /* 5) Write SDRAM open pages control register */
        ldr     r1, =(INTER_REGS_BASE + SDRAM_OPEN_PAGE_CTRL_REG)
        ldr     r4, =SDRAM_OPEN_PAGES_CTRL_REG_DV
        str     r4, [r1]
                          
        /* 6) Write SDRAM timing Low register */
        ldr     r1, =(INTER_REGS_BASE + DRAM_BUF_REG5)
        ldr     r4, [r1]
        ldr     r1, =(INTER_REGS_BASE + SDRAM_TIMING_CTRL_LOW_REG)
        str     r4, [r1]
        
        /* 7) Write SDRAM timing High register */
        ldr     r1, =(INTER_REGS_BASE + DRAM_BUF_REG6)
        ldr     r4, [r1]
        ldr     r1, =(INTER_REGS_BASE + SDRAM_TIMING_CTRL_HIGH_REG)
        str     r4, [r1]
                
        /* 8) Write SDRAM mode register */ 
        /* The CPU must not attempt to change the SDRAM Mode register setting */
        /* prior to DRAM controller completion of the DRAM initialization     */
        /* sequence. To guarantee this restriction, it is recommended that    */
        /* the CPU sets the SDRAM Operation register to NOP command, performs */
        /* read polling until the register is back in Normal operation value, */
        /* and then sets SDRAM Mode register to it�s new value.               */
        
	/* 8.1 write 'nop' to SDRAM operation */
        mov     r4, #0x5                 /* 'NOP' command              */
        MV_REG_WRITE_ASM(r4, r1, SDRAM_OPERATION_REG)
       
        /* 8.2 poll SDRAM operation. Make sure its back to normal operation   */
_sdramOpPoll1:        
        ldr     r4, [r1]
        cmp     r4, #0                          /* '0' = Normal SDRAM Mode    */
        bne     _sdramOpPoll1
        
        /* 8.3 Now its safe to write new value to SDRAM Mode register         */
        ldr     r1, =(INTER_REGS_BASE + DRAM_BUF_REG2)
        ldr     r4, [r1]
        ldr     r1, =(INTER_REGS_BASE + SDRAM_MODE_REG)
        str     r4, [r1]
        
        /* 8.4 Make the Dunit write the DRAM its new mode                     */       
        mov     r4, #0x3                 /* Mode Register Set command  */
        MV_REG_WRITE_ASM (r4, r1, SDRAM_OPERATION_REG)
               
        /* 8.5 poll SDRAM operation. Make sure its back to normal operation   */
_sdramOpPoll2:        
        ldr     r4, [r1]                     
        cmp     r4, #0                          /* '0' = Normal SDRAM Mode    */
        bne     _sdramOpPoll2
	
        /* Config DDR2 registers (Extended mode, ODTs and pad calibration)    */
        MV_REG_READ_ASM (r4, r1, SDRAM_CONFIG_REG)
        tst     r4, #SDRAM_DTYPE_DDR2
        beq     _extModeODTEnd
        
        /* 9) Write SDRAM Extended mode register This operation should be     */
        /*    done for each memory bank                                       */ 
        /* write 'nop' to SDRAM operation */
        mov     r4, #0x5                 /* 'NOP' command              */
        MV_REG_WRITE_ASM (r4, r1, SDRAM_OPERATION_REG)
       
        /* poll SDRAM operation. Make sure its back to normal operation   */
_sdramOpPoll3:        
        ldr     r4, [r1]
        cmp     r4, #0                          /* '0' = Normal SDRAM Mode    */
        bne     _sdramOpPoll3
        
        /* Now its safe to write new value to SDRAM Extended Mode register    */
        ldr     r1, =(INTER_REGS_BASE + DRAM_BUF_REG10)
        ldr     r4, [r1]
        ldr     r1, =(INTER_REGS_BASE + SDRAM_EXTENDED_MODE_REG)
        str     r4, [r1]
        
        /* Go over each of the Banks */
        ldr     r3, =0          /* r3 = DRAM bank Num */

extModeLoop:        
        /* Set the SDRAM Operation Control to each of the DRAM banks          */
        mov     r2, r3   /* Do not swap the bank counter value */
        MV_REG_WRITE_ASM (r2, r1, SDRAM_OPERATION_CTRL_REG)
        
        /* Make the Dunit write the DRAM its new mode                     */       
        mov     r4, #0x4        /* Extended Mode Register Set command  */
        MV_REG_WRITE_ASM (r4, r1, SDRAM_OPERATION_REG)
               
        /* poll SDRAM operation. Make sure its back to normal operation   */
_sdramOpPoll4:        
        ldr     r4, [r1]                     
        cmp     r4, #0                          /* '0' = Normal SDRAM Mode    */
        bne     _sdramOpPoll4
	
        add     r3, r3, #1
        cmp     r3, #4         /* 4 = Number of banks */
        bne     extModeLoop
        
extModeEnd:	
        /* Config DDR2 On Die Termination (ODT) registers */
        /* Write SDRAM DDR2 ODT control low register */
        ldr     r1, =(INTER_REGS_BASE + DRAM_BUF_REG7)
        ldr     r4, [r1]
        ldr     r1, =(INTER_REGS_BASE + DDR2_SDRAM_ODT_CTRL_LOW_REG)
        str     r4, [r1]
        
        /* Write SDRAM DDR2 ODT control high register */
        ldr     r1, =(INTER_REGS_BASE + DRAM_BUF_REG8)
        ldr     r4, [r1]
        ldr     r1, =(INTER_REGS_BASE + DDR2_SDRAM_ODT_CTRL_HIGH_REG)
        str     r4, [r1]
        
        /* Write SDRAM DDR2 Dunit ODT control register */
        ldr     r1, =(INTER_REGS_BASE + DRAM_BUF_REG9)
        ldr     r4, [r1]
        ldr     r1, =(INTER_REGS_BASE + DDR2_DUNIT_ODT_CONTROL_REG)
        str     r4, [r1]
        

_extModeODTEnd:

        /* Implement Guideline (GL# MEM-2) P_CAL Automatic Calibration  */
        /* Does Not Work for Address/Control and Data Pads.             */
        /* Relevant for: 88F5181-A1/B0 and 88F5281-A0                   */
        
        /* Read device ID  */
        MV_REG_READ_ASM (r3, r1, PEX_CFG_DIRECT_ACCESS(0,PEX_DEVICE_AND_VENDOR_ID))
        mov     r3, r3, LSR #16         /* r3 = r3 >> 16 */
        
        /* Read device revision */
        MV_REG_READ_ASM(r2, r1, PEX_CFG_DIRECT_ACCESS(0,PEX_CLASS_CODE_AND_REVISION_ID));
        ldr     r1, =0x000000FF                  /* Mask for calss ID */
        and     r2, r2, r1
                
        /* Continue if Orion1 and device revision B1 */
        ldr     r1, =MV_5181_DEV_ID
        cmp     r3, r1
        bne     1f

        cmp     r2, #MV_5181_B1_REV
        bge     glMem2End
        b       glMem2Start 
1:

        /* Continue if Orion2 and device revision >= B0 */
        ldr     r1, =MV_5281_DEV_ID
        cmp     r3, r1
        bne     1f

        cmp     r2, #MV_5281_B0_REV
        bge     glMem2End
        b       glMem2Start 
1:
        /* Orion NAS */
        ldr     r5, =MV_5182_DEV_ID
        cmp     r6, r5
        beq     glMem2Start

        /* Orion Shark */
        ldr     r5, =MV_8660_DEV_ID
        cmp     r6, r5
        beq     glMem2Start

	b	glMem2End

glMem2Start:
        
        /* DDR SDRAM Address/Control Pads Calibration                         */
        MV_REG_READ_ASM (r3, r1, SDRAM_ADDR_CTRL_PADS_CAL_REG)
                
        /* Set Bit [31] to make the register writable                   */
        orr   r2, r3, #SDRAM_WR_EN
        
        MV_REG_WRITE_ASM (r2, r1, SDRAM_ADDR_CTRL_PADS_CAL_REG)
        
        bic   r3, r3, #SDRAM_WR_EN      /* Make register read-only      */
        bic   r3, r3, #SDRAM_TUNE_EN    /* Disable auto calibration     */
        bic   r3, r3, #SDRAM_DRVN_MASK  /* Clear r1[5:0]<DrvN>          */
        bic   r3, r3, #SDRAM_DRVP_MASK  /* Clear r1[11:6]<DrvP>         */        
        
        /* Get the final N locked value of driving strength [22:17]     */
        mov   r1, r3
        mov   r1, r1, LSL #9
        mov   r1, r1, LSR #26    /* r1[5:0]<DrvN>  = r3[22:17]<LockN>   */
        orr   r1, r1, r1, LSL #6 /* r1[11:6]<DrvP> = r1[5:0]<DrvN>      */
        
        /* Write to both <DrvN> bits [5:0] and <DrvP> bits [11:6]       */
        orr   r3, r3, r1
        
        MV_REG_WRITE_ASM (r3, r1, SDRAM_ADDR_CTRL_PADS_CAL_REG)

        
        /* DDR SDRAM Data Pads Calibration                         	*/
        MV_REG_READ_ASM (r3, r1, SDRAM_DATA_PADS_CAL_REG)
                
        /* Set Bit [31] to make the register writable                   */
        orr   r2, r3, #SDRAM_WR_EN
        
        MV_REG_WRITE_ASM (r2, r1, SDRAM_DATA_PADS_CAL_REG)
        
        bic   r3, r3, #SDRAM_WR_EN      /* Make register read-only      */
        bic   r3, r3, #SDRAM_TUNE_EN    /* Disable auto calibration     */
        bic   r3, r3, #SDRAM_DRVN_MASK  /* Clear r1[5:0]<DrvN>          */
        bic   r3, r3, #SDRAM_DRVP_MASK  /* Clear r1[11:6]<DrvP>         */        
        
        /* Get the final N locked value of driving strength [22:17]     */
        mov   r1, r3
        mov   r1, r1, LSL #9
        mov   r1, r1, LSR #26
        orr   r1, r1, r1, LSL #6 /* r1[5:0] = r3[22:17]<LockN>  */
        
        /* Write to both <DrvN> bits [5:0] and <DrvP> bits [11:6]       */
        orr   r3, r3, r1
        
        MV_REG_WRITE_ASM (r3, r1, SDRAM_DATA_PADS_CAL_REG)

glMem2End:


        /* Implement Guideline (GL# MEM-3) Drive Strength Value         */
        /* Relevant for: 88F5181-A1/B0/B1 and 88F5281-A0/B0             */

        /* Get SDRAM Config value */
        MV_REG_READ_ASM (r2, r1, SDRAM_CONFIG_REG)
        
        /* Get DIMM type */
        tst     r2, #SDRAM_DTYPE_DDR2
        beq     ddr1StrengthVal

ddr2StrengthVal:        
        ldr     r4, =DDR2_ADDR_CTRL_PAD_STRENGTH_TYPICAL_DV
        ldr     r2, =DDR2_DATA_PAD_STRENGTH_TYPICAL_DV
        b       setDrvStrength 
ddr1StrengthVal:        
        ldr     r4, =DDR1_ADDR_CTRL_PAD_STRENGTH_TYPICAL_DV
        ldr     r2, =DDR1_DATA_PAD_STRENGTH_TYPICAL_DV

setDrvStrength:
        /* DDR SDRAM Address/Control Pads Calibration                   */
        MV_REG_READ_ASM (r3, r1, SDRAM_ADDR_CTRL_PADS_CAL_REG)
        
        orr   r3, r3, #SDRAM_WR_EN      /* Make register writeable      */
        
        MV_REG_WRITE_ASM (r3, r1, SDRAM_ADDR_CTRL_PADS_CAL_REG)
        HTOLL(r3,r1)
        
        bic   r3, r3, #SDRAM_WR_EN      /* Make register read-only      */
        bic   r3, r3, #SDRAM_PRE_DRIVER_STRENGTH_MASK
        orr   r3, r4, r3                /* Set default value for DDR    */
        
        MV_REG_WRITE_ASM (r3, r1, SDRAM_ADDR_CTRL_PADS_CAL_REG)

        
        /* DDR SDRAM Data Pads Calibration                         	      */        
        MV_REG_READ_ASM (r3, r1, SDRAM_DATA_PADS_CAL_REG)
        
        orr   r3, r3, #SDRAM_WR_EN      /* Make register writeable      */
        
        MV_REG_WRITE_ASM (r3, r1, SDRAM_DATA_PADS_CAL_REG)
        HTOLL(r3,r1)
        
        bic   r3, r3, #SDRAM_WR_EN      /* Make register read-only      */
        bic   r3, r3, #SDRAM_PRE_DRIVER_STRENGTH_MASK
        orr   r3, r2, r3                /* Set default value for DDR    */        
        
        MV_REG_WRITE_ASM (r3, r1, SDRAM_DATA_PADS_CAL_REG)


        /* Implement Guideline (GL# MEM-4) DQS Reference Delay Tuning   */
        /* Relevant for: 88F5181-A1/B0/B1 and 88F5281-A0/B0             */
                
        /* Get SDRAM Config value */
        MV_REG_READ_ASM (r2, r1, SDRAM_CONFIG_REG)
        
        /* Get DIMM type */
        tst     r2, #SDRAM_DTYPE_DDR2
        beq     ddr1FtdllVal

ddr2FtdllVal:        
        ldr     r4, =FTDLL_DDR2_200MHZ
        b       setFtdllReg 
ddr1FtdllVal:        
        /* Get the "sample on reset" register for the DDR frequancy     */
        MV_REG_READ_ASM (r4, r1, MPP_SAMPLE_AT_RESET)
        ldr     r1, =MSAR_ARMDDRCLCK_MASK
        and     r1, r4, r1

        /* Read device ID  */
        MV_REG_READ_ASM(r3, r2, PEX_CFG_DIRECT_ACCESS(0,PEX_DEVICE_AND_VENDOR_ID))
        mov     r3, r3, LSR #16         /* r3 = r3 >> 16 */
        
        /* Continue if Orion2 */
        ldr     r2, =MV_5281_DEV_ID
        cmp     r3, r2
        bne     1f
        
        tst    r4, #MSAR_ARMDDRCLCK_H_MASK
        beq    1f
        orr     r1, r1, #BIT8        
1:
        ldr    r4, =FTDLL_DDR1_166MHZ
        cmp     r1, #MSAR_ARMDDRCLCK_333_167
        beq    setFtdllReg 
        cmp     r1, #MSAR_ARMDDRCLCK_500_167
        beq    setFtdllReg 
        cmp     r1, #MSAR_ARMDDRCLCK_667_167
        beq    setFtdllReg 
        cmp     r1, #MSAR_ARMDDRCLCK_400_133
        beq     setFtdllReg 
        cmp     r1, #MSAR_ARMDDRCLCK_533_133
        beq     setFtdllReg 
        cmp     r1, #MSAR_ARMDDRCLCK_500_125
        beq     setFtdllReg 
        
        
        ldr    r4, =FTDLL_DDR1_200MHZ
        cmp     r1, #MSAR_ARMDDRCLCK_400_200
        beq    setFtdllReg 
        cmp     r1, #MSAR_ARMDDRCLCK_600_200
        beq    setFtdllReg 
        cmp     r1, #MSAR_ARMDDRCLCK_800_200
        beq    setFtdllReg 
                       
        ldr    r4, =0

setFtdllReg:

        MV_REG_READ_ASM (r2, r1, SDRAM_FTDLL_CONFIG_REG)
        orr    r2, r2, r4
        MV_REG_WRITE_ASM (r2, r1, SDRAM_FTDLL_CONFIG_REG)     
        bic   r2, r2, #1
        MV_REG_WRITE_ASM (r2, r1, SDRAM_FTDLL_CONFIG_REG)

restoreTmpRegs:
        /* Restore the registers we used to save the DDR detect values */
        
        ldr     r4, =DRAM_BUF_REG0_DV
        MV_REG_WRITE_ASM (r4, r1, DRAM_BUF_REG0)

        ldr     r4, =DRAM_BUF_REG1_DV
        MV_REG_WRITE_ASM (r4, r1, DRAM_BUF_REG1)
        
        ldr     r4, =DRAM_BUF_REG2_DV
        MV_REG_WRITE_ASM (r4, r1, DRAM_BUF_REG2)

        ldr     r4, =DRAM_BUF_REG3_DV
        MV_REG_WRITE_ASM (r4, r1, DRAM_BUF_REG3)

        ldr     r4, =DRAM_BUF_REG4_DV
        MV_REG_WRITE_ASM (r4, r1, DRAM_BUF_REG4)
        
        ldr     r4, =DRAM_BUF_REG5_DV
        MV_REG_WRITE_ASM (r4, r1, DRAM_BUF_REG5)
        
        ldr     r4, =DRAM_BUF_REG6_DV
        MV_REG_WRITE_ASM (r4, r1, DRAM_BUF_REG6)
        
        ldr     r4, =DRAM_BUF_REG7_DV
        MV_REG_WRITE_ASM (r4, r1, DRAM_BUF_REG7)
        
        ldr     r4, =DRAM_BUF_REG8_DV
        MV_REG_WRITE_ASM (r4, r1, DRAM_BUF_REG8)
        
        ldr     r4, =DRAM_BUF_REG9_DV
        MV_REG_WRITE_ASM (r4, r1, DRAM_BUF_REG9)
        
        ldr     r4, =DRAM_BUF_REG10_DV
        MV_REG_WRITE_ASM (r4, r1, DRAM_BUF_REG10)
       
 
        /* Restore registers */
        /* Save register on stack */
	cmp	sp, #0
	beq	no_stack_l
load_from_stack:
        ldmia	sp!, {r1, r2, r3, r4}
no_stack_l:
           
        mov     pc, lr
