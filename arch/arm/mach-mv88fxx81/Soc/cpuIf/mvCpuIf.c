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


/* includes */
#include "mvCpuIf.h"
#include "mvCpuIfConfig.h"

/*#define MV_DEBUG*/
/* defines  */
#ifdef MV_DEBUG
	#define DB(x)	x
#else
	#define DB(x)
#endif	



/* locals   */
/* static functions */
static MV_BOOL cpuTargetWinOverlap(MV_TARGET target, MV_ADDR_WIN *pAddrWin);


/* CPU address decode table. Note that table entry number must match its    */
/* winNum enumerator. For example, table entry '4' must describe Deivce CS0 */
/* winNum which is represent by DEVICE_CS0 enumerator (4).                  */
MV_CPU_DEC_WIN cpuAddrWinMap[] = 
{
/*     base low      base high     size       	   WinNum     enable */
	{{SDRAM_CS0_BASE ,    0,      SDRAM_CS0_SIZE} ,0xFFFFFFFF,DIS},
	{{SDRAM_CS1_BASE ,    0,      SDRAM_CS1_SIZE} ,0xFFFFFFFF,DIS}, 
	{{SDRAM_CS2_BASE ,    0,      SDRAM_CS2_SIZE} ,0xFFFFFFFF,DIS},
	{{SDRAM_CS3_BASE ,    0,      SDRAM_CS3_SIZE} ,0xFFFFFFFF,DIS}, 
	{{PEX0_MEM_BASE  ,    0,      PEX0_MEM_SIZE } ,0x0       ,EN},
	{{PEX0_IO_BASE   ,    0,      PEX0_IO_SIZE   },0x2       ,EN},
	{{PCI0_MEM_BASE  ,    0,      PCI0_MEM_SIZE } ,0x1       ,EN},
	{{PCI0_IO_BASE   ,    0,      PCI0_IO_SIZE  } ,0x3       ,EN},
	{{INTER_REGS_BASE,    0,      INTER_REGS_SIZE},0x8       ,EN},
	{{DEVICE_CS0_BASE,    0,      DEVICE_CS0_SIZE},0x5		 ,EN}, 
	{{DEVICE_CS1_BASE,    0,      DEVICE_CS1_SIZE},0x6		 ,EN}, 
	{{DEVICE_CS2_BASE,    0,      DEVICE_CS2_SIZE},0x7       ,EN},
	{{BOOTDEV_CS_BASE,    0,      BOOTDEV_CS_SIZE},0x4       ,EN},
	{{CRYPT_ENG_BASE,     0,      CRYPT_ENG_SIZE} ,0x7       ,DIS},
    /* Table terminator */
    {{TBL_TERM, TBL_TERM, TBL_TERM}, TBL_TERM,TBL_TERM} 
};


/*******************************************************************************
* mvCpuIfInit - Initialize Controller CPU interface
*
* DESCRIPTION:
*       This function initialize Controller CPU interface:
*       1. Set CPU interface configuration registers.
*       2. Set CPU master Pizza arbiter control according to static 
*          configuration described in configuration file.
*       3. Opens CPU address decode windows. DRAM windows are assumed to be 
*		   already set (auto detection).
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
MV_STATUS mvCpuIfInit(void)
{    
	MV_U32 regVal;
	MV_TARGET target;
	MV_ADDR_WIN addrWin;	

	/* Set ARM Configuration register */
	regVal  = MV_REG_READ(CPU_CONFIG_REG);
	regVal &= ~CPU_CONFIG_DEFAULT_MASK;
	regVal |= CPU_CONFIG_DEFAULT;


	MV_REG_WRITE(CPU_CONFIG_REG,regVal);

	/* Set ARM Control and Status register */
	regVal  = MV_REG_READ(CPU_CTRL_STAT_REG);
	regVal &= ~CPU_CTRL_STAT_DEFAULT_MASK;
	regVal |= CPU_CTRL_STAT_DEFAULT;
	MV_REG_WRITE(CPU_CTRL_STAT_REG,regVal);

	/* First disable all CPU target windows  */
	for (target = 0; cpuAddrWinMap[target].enable != TBL_TERM; target++)
    	{
		if ((MV_TARGET_IS_DRAM(target))||(target == INTER_REGS))
		{
			continue;
		}

		mvCpuIfTargetWinEnable(target,MV_FALSE);
	}

	/* Go through all targets in user table until table terminator			*/
	for (target = 0; cpuAddrWinMap[target].enable != TBL_TERM; target++)
    	{
	/* if DRAM auto sizing is used do not initialized DRAM target windows, 	*/
	/* assuming this already has been done earlier.							*/
#ifdef	MV_DRAM_AUTO_SIZE
		if (MV_TARGET_IS_DRAM(target))
		{
			continue;
		}
#endif
		
		if((0 == cpuAddrWinMap[target].addrWin.size) ||
		   (MV_FALSE == cpuAddrWinMap[target].enable))

		{
			
			if (MV_OK != mvCpuIfTargetWinEnable(target, MV_FALSE))
			{
				DB(mvOsPrintf("mvCpuIfInit:ERR. mvCpuIfTargetWinEnable fail\n"));
				return MV_ERROR;
			}

		}
		else
		{
			if (MV_OK != mvCpuIfTargetWinSet(target, &cpuAddrWinMap[target]))
			{
				DB(mvOsPrintf("mvCpuIfInit:ERR. mvCpuIfTargetWinSet fail\n"));


				return MV_ERROR;
			}

			addrWin.baseLow = cpuAddrWinMap[target].addrWin.baseLow;
			addrWin.baseHigh = cpuAddrWinMap[target].addrWin.baseHigh;
			if (0xffffffff == mvAhbToMbusWinRemap(cpuAddrWinMap[target].winNum ,&addrWin))
			{
				DB(mvOsPrintf("mvCpuIfInit:WARN. mvAhbToMbusWinRemap can't remap winNum=%d\n",
							  cpuAddrWinMap[target].winNum));
			}


		}
    }

	return MV_OK;
}

/*******************************************************************************
* mvCpuIfTargetWinSet - Set CPU-to-peripheral target address window
*
* DESCRIPTION:
*       This function sets a peripheral target (e.g. SDRAM bank0, PCI0_MEM0) 
*       address window, also known as address decode window.  
*       A new address decode window is set for specified target address window.
*       If address decode window parameter structure enables the window, 
*       the routine will also enable the target window, allowing CPU to access
*       the target window.
*
* INPUT:
*       target      - Peripheral target enumerator.
*       pAddrDecWin - CPU target window data structure.
*
* OUTPUT:
*       N/A
*
* RETURN:
*       MV_OK if CPU target window was set correctly, MV_ERROR in case of  
*       address window overlapps with other active CPU target window or
*		trying to assign 36bit base address while CPU does not support that.
*       The function returns MV_NOT_SUPPORTED, if the target is unsupported.
*
*******************************************************************************/
MV_STATUS mvCpuIfTargetWinSet(MV_TARGET target, MV_CPU_DEC_WIN *pAddrDecWin)
{
	MV_AHB_TO_MBUS_DEC_WIN decWin;
	MV_U32 existingWinNum;
	MV_DRAM_DEC_WIN addrDecWin;

	/* Check parameters */
	if (target >= MAX_TARGETS)
	{
		mvOsPrintf("mvCpuIfTargetWinSet: target %d is Illegal\n", target);
		return MV_ERROR;
	}

    /* 2) Check if the requested window overlaps with current windows		*/
    if (MV_TRUE == cpuTargetWinOverlap(target, &pAddrDecWin->addrWin))
	{
        	mvOsPrintf("mvCpuIfTargetWinSet: ERR. Target %d overlap\n", target);
		return MV_BAD_PARAM;
	}

	if (MV_TARGET_IS_DRAM(target))
	{
		/* copy relevant data to MV_DRAM_DEC_WIN structure */
		addrDecWin.addrWin.baseHigh = pAddrDecWin->addrWin.baseHigh;
		addrDecWin.addrWin.baseLow = pAddrDecWin->addrWin.baseLow;
		addrDecWin.addrWin.size = pAddrDecWin->addrWin.size;
		addrDecWin.enable = pAddrDecWin->enable;
	}
	else
	{
		/* copy relevant data to MV_AHB_TO_MBUS_DEC_WIN structure */
		decWin.addrWin.baseLow = pAddrDecWin->addrWin.baseLow;
		decWin.addrWin.baseHigh = pAddrDecWin->addrWin.baseHigh;
		decWin.addrWin.size = pAddrDecWin->addrWin.size;
		decWin.enable = pAddrDecWin->enable;
		decWin.target = target;

		existingWinNum = mvAhbToMbusWinTargetGet(target);

		/* check if there is already another Window configured
		for this target */
		if ((existingWinNum < MAX_AHB_TO_MBUS_WINS )&&
			(existingWinNum != pAddrDecWin->winNum))
		{
			/* if we want to enable the new winow number 
			passed by the user , then the old one should
			be disabled */
			if (MV_TRUE == pAddrDecWin->enable)
			{
				/* be sure it is disabled */
				mvAhbToMbusWinEnable(existingWinNum , MV_FALSE);
			}
		}

        if (mvAhbToMbusWinSet(pAddrDecWin->winNum,&decWin) != MV_OK)
		{
			mvOsPrintf("mvCpuIfTargetWinSet: mvAhbToMbusWinSet Failed\n");
			return MV_ERROR;
		}

	}

    return MV_OK;
}

/*******************************************************************************
* mvCpuIfTargetWinGet - Get CPU-to-peripheral target address window
*
* DESCRIPTION:
*		Get the CPU peripheral target address window.
*
* INPUT:
*       target - Peripheral target enumerator
*
* OUTPUT:
*       pAddrDecWin - CPU target window information data structure.
*
* RETURN:
*       MV_OK if target exist, MV_ERROR otherwise.
*
*******************************************************************************/
MV_STATUS mvCpuIfTargetWinGet(MV_TARGET target, MV_CPU_DEC_WIN *pAddrDecWin)
{

	MV_U32 winNum=0xffffffff;
	MV_AHB_TO_MBUS_DEC_WIN decWin;
	MV_DRAM_DEC_WIN addrDecWin;

	/* Check parameters */
	if (target >= MAX_TARGETS)
	{
		mvOsPrintf("mvCpuIfTargetWinGet: target %d is Illigal\n", target);
		return MV_ERROR;
	}

	if (MV_TARGET_IS_DRAM(target))
	{
		if (mvDramIfWinGet(target,&addrDecWin) != MV_OK)
		{
			mvOsPrintf("mvCpuIfTargetWinGet: Failed to get window target %d\n",
					   target);
			return MV_ERROR;
		}

		/* copy relevant data to MV_CPU_DEC_WIN structure */
		pAddrDecWin->addrWin.baseLow = addrDecWin.addrWin.baseLow;
		pAddrDecWin->addrWin.baseHigh = addrDecWin.addrWin.baseHigh;
		pAddrDecWin->addrWin.size = addrDecWin.addrWin.size;
		pAddrDecWin->enable = addrDecWin.enable;
		pAddrDecWin->winNum = 0xffffffff;

	}
	else
	{
		/* get the Window number associated with this target */

		winNum = mvAhbToMbusWinTargetGet(target);

		if (winNum >= MAX_AHB_TO_MBUS_WINS)
		{
			return MV_NO_SUCH;

		}

		if (mvAhbToMbusWinGet(winNum , &decWin) != MV_OK)
		{
			mvOsPrintf("%s: mvAhbToMbusWinGet Failed at winNum = %d\n",
					   __FUNCTION__, winNum);
			return MV_ERROR;

		}

		/* copy relevant data to MV_CPU_DEC_WIN structure */
		pAddrDecWin->addrWin.baseLow = decWin.addrWin.baseLow;
		pAddrDecWin->addrWin.baseHigh = decWin.addrWin.baseHigh;
		pAddrDecWin->addrWin.size = decWin.addrWin.size;
		pAddrDecWin->enable = decWin.enable;
		pAddrDecWin->winNum = winNum;

	}

	return MV_OK;
}

/*******************************************************************************
* mvCpuIfTargetWinEnable - Enable/disable a CPU address decode window
*
* DESCRIPTION:
*       This function enable/disable a CPU address decode window.
*       if parameter 'enable' == MV_TRUE the routine will enable the 
*       window, thus enabling CPU accesses (before enabling the window it is 
*       tested for overlapping). Otherwise, the window will be disabled.
*
* INPUT:
*       target - Peripheral target enumerator.
*       enable - Enable/disable parameter.
*
* OUTPUT:
*       N/A
*
* RETURN:
*       MV_ERROR if protection window number was wrong, or the window 
*       overlapps other target window.
*
*******************************************************************************/
MV_STATUS mvCpuIfTargetWinEnable(MV_TARGET target,MV_BOOL enable)
{
	MV_U32 winNum, temp;
	MV_CPU_DEC_WIN addrDecWin;

	/* Check parameters */
	if (target >= MAX_TARGETS)
	{
		mvOsPrintf("mvCpuIfTargetWinEnable: target %d is Illigal\n", target);
		return MV_ERROR;
	}

	/* get the window and check if it exist */
	temp = mvCpuIfTargetWinGet(target, &addrDecWin);
	if (MV_NO_SUCH == temp)
	{
		return (enable? MV_ERROR: MV_OK);
	}
	else if( MV_OK != temp)
	{
		mvOsPrintf("%s: ERR. Getting target %d failed.\n",__FUNCTION__, target);
		return MV_ERROR;
	}

	/* check overlap */

	if (MV_TRUE == enable)
	{
		if (MV_TRUE == cpuTargetWinOverlap(target, &addrDecWin.addrWin))
		{
			DB(mvOsPrintf("%s: ERR. Target %d overlap\n",__FUNCTION__, target));
			return MV_ERROR;
		}
		
	}

	if (!MV_TARGET_IS_DRAM(target))
	{
		/* get the Window number associated with this target */

		winNum = mvAhbToMbusWinTargetGet(target);

		if (winNum >= MAX_AHB_TO_MBUS_WINS)
		{
			return (enable? MV_ERROR: MV_OK);
		}

		if (mvAhbToMbusWinEnable(winNum , enable) != MV_OK)
		{
			mvOsPrintf("mvCpuIfTargetWinGet: Failed to enable window = %d\n",
					   winNum);
			return MV_ERROR;

		}
	}

	return MV_OK;
}

/*******************************************************************************
* cpuTargetWinOverlap - Detect CPU address decode windows overlapping
*
* DESCRIPTION:
*       An unpredicted behaviur is expected in case CPU address decode 
*       windows overlapps.
*       This function detects CPU address decode windows overlapping of a 
*       specified target. The function does not check the target itself for 
*       overlapping. The function also skipps disabled address decode windows.
*
* INPUT:
*       target      - Peripheral target enumerator.
*       pAddrDecWin - An address decode window struct.
*
* OUTPUT:
*       None.
*
* RETURN:
*       MV_TRUE if the given address window overlaps current address
*       decode map, MV_FALSE otherwise.
*
*******************************************************************************/
static MV_BOOL cpuTargetWinOverlap(MV_TARGET target, MV_ADDR_WIN *pAddrWin)
{
    MV_U32 			targetNum;
    MV_CPU_DEC_WIN 	addrDecWin;
	MV_STATUS		status;

	for(targetNum = 0; targetNum < MAX_TARGETS; targetNum++)
    {
		/* don't check our target or illegal targets */
        if (targetNum == target)
        {
            continue;
        }       

		/* Get window parameters 	*/
		status = mvCpuIfTargetWinGet(targetNum, &addrDecWin);
        if(MV_NO_SUCH == status)
        {
            continue;
        }
		if(MV_OK != status)
		{
			DB(mvOsPrintf("cpuTargetWinOverlap: ERR. TargetWinGet failed\n"));
            return MV_TRUE;
		}

		/* Do not check disabled windows	*/
		if (MV_FALSE == addrDecWin.enable)
		{
			continue;
		}
        
        if(MV_TRUE == ctrlWinOverlapTest(pAddrWin, &addrDecWin.addrWin))
		{                    
			DB(mvOsPrintf(
			"cpuTargetWinOverlap: Required target %d overlap current %d\n", 
															target, targetNum));
			return MV_TRUE;           
		}
    }
    
	return MV_FALSE;
}
