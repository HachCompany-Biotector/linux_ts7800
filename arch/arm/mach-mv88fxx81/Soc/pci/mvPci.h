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


#ifndef __INCPCIH
#define __INCPCIH

#include "mvPciRegs.h"
#include "mvCtrlEnvLib.h"
#include "mvCtrlEnvAddrDec.h"



/* NOTE not supported in this driver:

 Built In Self Test (BIST)
 Vital Product Data (VPD)
 Message Signaled Interrupt (MSI)
 Power Management
 Compact PCI Hot Swap
 Header retarget
 
Registers not supported: 
1) PCI DLL Status and Control (PCI0 0x1D20, PCI1 0x1DA0)
2) PCI/MPP Pads Calibration (CI0/MPP[31:16] 0x1D1C, PCI1/MPP[15:0] 0X1D9C)
*/  

/* defines */
/* The number of supported PCI interfaces depend on Marvell controller 		*/
/* device number. This device number ID is located on the PCI unit 			*/
/* configuration header. This creates a loop where calling PCI 				*/
/* configuration read/write	routine results a call to get PCI configuration */
/* information etc. This macro defines a default PCI interface. This PCI	*/
/* interface is sure to exist.												*/
#define PCI_DEFAULT_IF	0


/* typedefs */
/* The Marvell controller supports both conventional PCI and PCI-X.         */
/* This enumeration describes the PCI type.                                 */
typedef enum _mvPciType
{
    MV_PCI_CONV,    /* Conventional PCI */
    MV_PCIX         /* PCI-X            */
}MV_PCI_TYPE;

/* The Marvell controller supports both PCI width of 32 and 64 bit.         */
/* This enumerator describes PCI width                                      */
typedef enum _mvPciWidth
{
    MV_PCI_32,  /* PCI width 32bit  */
    MV_PCI_64   /* PCI width 64bit  */
}MV_PCI_WIDTH;

/* This structure describes the PCI unit configured type, speed and width.  */
typedef struct _mvPciMode
{
    MV_PCI_TYPE  pciType;    /* PCI type                                    */
    MV_U32       pciSpeed;   /* Assuming PCI base clock on board is 33MHz   */
    MV_PCI_WIDTH pciWidth;   /* PCI bus width                               */
}MV_PCI_MODE;

/* This structure describes a PCI BAR. It is also refered as PCI target     */
/* window to keep consistency with other address decode units in the system */
typedef struct _mvPciBarWin 
{
    MV_ADDR_WIN     addrWin;    /* Address window       */
    MV_BOOL     	enable;     /* BAR enable/disable   */
}MV_PCI_BAR_WIN;

/* This structure describes PCI region attributes                           */
typedef struct _mvPciRegionAttr
{
    MV_PROT_RIGHT   access;         /* Access protection                    */
    MV_PROT_RIGHT   write;          /* Write protection                     */
    MV_SWAP_TYPE    swapType;       /* Data swap mode for that region       */
    MV_U32		    readMaxBurst;   /* Read max burst                       */
    MV_U32		    readBurst;      /* Read burst. Conventional PCI only    */
    MV_U32		    writeMaxBurst;  /* Write max burst                      */
    MV_BOOL         pciOrder;       /* Hardware support for PCI ordering    */
}MV_PCI_REGION_ATTR;

/* The PCI slave interface supports configurable access control.            */
/* It is possible to define up to six address ranges to different           */
/* configurations. This structure describes the PCI access region           */
typedef struct _mvPciProtWin
{
    MV_ADDR_WIN         addrWin;    /* An address window                    */
    MV_PCI_REGION_ATTR  attributes; /* Window attributes                    */
    MV_BOOL             enable;     /* Window enabled/disabled              */
}MV_PCI_PROT_WIN;

/* Global Functions prototypes */

/* mvPciInit - Initialize PCI interfaces*/
MV_STATUS mvPciInit(MV_U32 pciIf);
/* mvPciWaFix - WA for the PCI parity errata */
MV_VOID mvPciWaFix(MV_U32 pciIf);

/* mvPciCommandSet - Set PCI comman register value.*/
MV_STATUS mvPciCommandSet(MV_U32 pciIf, MV_U32 command);

/* mvPciModeGet - Get PCI interface mode.*/
MV_STATUS mvPciModeGet(MV_U32 pciIf, MV_PCI_MODE *pPciMode);

/* mvPciRetrySet - Set PCI retry counters*/
MV_STATUS mvPciRetrySet(MV_U32 pciIf, MV_U32 counter);

/* mvPciDiscardTimerSet - Set PCI discard timer*/
MV_STATUS mvPciDiscardTimerSet(MV_U32 pciIf, MV_U32 pClkCycles);

/* mvPciArbEnable - PCI arbiter enable/disable*/
MV_STATUS mvPciArbEnable(MV_U32 pciIf, MV_BOOL enable);

/* mvPciArbParkDis - Disable arbiter parking on agent */
MV_STATUS mvPciArbParkDis(MV_U32 pciIf, MV_U32 pciAgentMask);

/* mvPciArbBrokDetectSet - Set PCI arbiter broken detection */
MV_STATUS mvPciArbBrokDetectSet(MV_U32 pciIf, MV_U32 pClkCycles);

/* mvPciConfigRead - Read from configuration space */
MV_U32 mvPciConfigRead (MV_U32 pciIf, MV_U32 bus, MV_U32 dev,
						MV_U32 func,MV_U32 regOff);

/* mvPciConfigWrite - Write to configuration space */
MV_STATUS mvPciConfigWrite(MV_U32 pciIf, MV_U32 bus, MV_U32 dev,
                           MV_U32 func, MV_U32 regOff, MV_U32 data);

/* mvPciMasterEnable - Enable/disale PCI interface master transactions.*/
MV_STATUS mvPciMasterEnable(MV_U32 pciIf, MV_BOOL enable);

/* mvPciSlaveEnable - Enable/disale PCI interface slave transactions.*/
MV_STATUS mvPciSlaveEnable(MV_U32 pciIf, MV_U32 bus, MV_U32 dev,MV_BOOL enable);

/* mvPciLocalBusNumSet - Set PCI interface local bus number.*/
MV_STATUS mvPciLocalBusNumSet(MV_U32 pciIf, MV_U32 busNum);

/* mvPciLocalBusNumGet - Get PCI interface local bus number.*/
MV_U32 mvPciLocalBusNumGet(MV_U32 pciIf);

/* mvPciLocalDevNumSet - Set PCI interface local device number.*/
MV_STATUS mvPciLocalDevNumSet(MV_U32 pciIf, MV_U32 devNum);

/* mvPciLocalDevNumGet - Get PCI interface local device number.*/
MV_U32 mvPciLocalDevNumGet(MV_U32 pciIf);

/* mvPciTargetWinSet - Set PCI to peripheral target address window BAR*/
MV_STATUS mvPciTargetWinSet(MV_U32 pciIf, MV_PCI_BAR slaveTarget, 
                            MV_PCI_BAR_WIN *pAddrBarWin);

/* mvPciTargetWinGet - Get PCI to peripheral target address window*/
MV_STATUS mvPciTargetWinGet(MV_U32 pciIf, MV_PCI_BAR slaveTarget, 
                            MV_PCI_BAR_WIN *pAddrBarWin);

/* mvPciTargetWinEnable - Enable/disable a PCI BAR window*/
MV_STATUS mvPciTargetWinEnable(MV_U32 pciIf,MV_PCI_BAR slaveTarget, 
							   MV_BOOL enable);

/* mvPciProtWinSet - Set PCI protection access window*/
MV_STATUS mvPciProtWinSet(MV_U32 pciIf, MV_U32 winNum, 
                          MV_PCI_PROT_WIN *pProtWin);

/* mvPciProtWinGet - Get PCI protection access window*/
MV_STATUS mvPciProtWinGet(MV_U32 pciIf, 
						  MV_U32 winNum, 
                          MV_PCI_PROT_WIN *pProtWin);

/* mvPciProtWinEnable - Get PCI protection access window*/
MV_STATUS mvPciProtWinEnable(MV_U32 pciIf, MV_U32 winNum, MV_BOOL enable);

/* mvPciTargetRemap - Set PCI to target address window remap.*/
MV_STATUS mvPciTargetRemap(MV_U32 pciIf, MV_PCI_BAR slaveTarget, 
                           MV_ADDR_WIN *pAddrWin);

/* mvPciAddrDecShow - Display address decode windows attributes */
MV_VOID mvPciAddrDecShow(MV_VOID);

#endif /* #ifndef __INCPCIH */



