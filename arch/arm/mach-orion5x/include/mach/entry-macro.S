/*
 * arch/arm/mach-orion5x/include/mach/entry-macro.S
 *
 * Low-level IRQ helper macros for Orion platforms
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <mach/bridge-regs.h>
#include <mach/irqs.h>

#if (0)

	.macro  disable_fiq
	.endm

	.macro  arch_ret_to_user, tmp1, tmp2
	.endm

	.macro  get_irqnr_preamble, base, tmp
	ldr	\base, =MAIN_IRQ_CAUSE
	.endm

	.macro  get_irqnr_and_base, irqnr, irqstat, base, tmp
	ldr	\irqstat, [\base, #0]		@ main cause
	ldr	\tmp, [\base, #(MAIN_IRQ_MASK - MAIN_IRQ_CAUSE)] @ main mask
	mov	\irqnr, #0			@ default irqnr
	@ find cause bits that are unmasked
	ands	\irqstat, \irqstat, \tmp	@ clear Z flag if any
	clzne	\irqnr,	\irqstat		@ calc irqnr
	rsbne	\irqnr, \irqnr, #31
	.endm
	
#else

  .macro  disable_fiq
        .endm
        
	.macro  get_irqnr_preamble, base, tmp
	ldr	\base, =MAIN_IRQ_CAUSE
	.endm

	.macro  arch_ret_to_user, tmp1, tmp2
	.endm

        
#define IRQ_START 	0
#define IRQ_GPP_0_7	IRQ_ORION5X_GPIO_0_7
#define IRQ_GPP_8_15 IRQ_ORION5X_GPIO_8_15
#define IRQ_GPP_16_23 IRQ_ORION5X_GPIO_16_23
#define IRQ_GPP_24_31 IRQ_ORION5X_GPIO_24_31
#define IRQ_DOORBELL IRQ_ORION5X_DOORBELL_H2C
	
#if defined(CONFIG_CPU_BIG_ENDIAN)
        	.macro	HTOLL sr, tt		@ sr   = A  ,B  ,C  ,D    
        	eor \tt, \sr, \sr, ror #16 ;    @ temp = A^C,B^	  
        	bic \tt, \tt, #0xff0000 ;       @ temp = A^C,0  ,C^A,D^B
        	mov \sr, \sr, ror #8 ;          @ sr   = D  ,A  ,B  ,C
        	eor \sr, \sr, \tt, lsr #8       @ sr   = D  ,C  ,B  ,A
        	.endm
#else		
        	.macro	HTOLL sr, tt	
        	.endm
#endif				
                /*
                 * This macro should retunr the follows:
                 *      irqnr - the number of the received irq
                 *      eq flag - not set if succeed and set if fail.
                 */
                .macro  get_irqnr_and_base, irqnr, irqstat, base, tmp
                ldr     \base, =MV_ASM_IRQ_BASE_REG
                ldr     \tmp, [\base, #MV_ASM_IRQ_MASK_REG]                     @ get mask
                HTOLL	\tmp, \base
                mov     \irqnr, #IRQ_START
                ldr	\base, =MV_ASM_IRQ_BASE_REG				
                ldr     \irqstat, [\base, #MV_ASM_IRQ_CAUSE_REG]                @ get cause
		HTOLL	\irqstat, \base
                ldr	\base, =MV_ASM_IRQ_BASE_REG
                ands    \irqstat, \irqstat, \tmp                        @ and mask and cause
                beq     1004f                                           @ if eq to 0 (flag set) error return, else go to 1001

1001:           tst     \irqstat, #0xff                                 @ find the first bit which is '1'
                moveq   \irqstat, \irqstat, lsr #8
                addeq   \irqnr, \irqnr, #8
                tsteq   \irqstat, #0xff
                moveq   \irqstat, \irqstat, lsr #8
                addeq   \irqnr, \irqnr, #8
                tsteq   \irqstat, #0xff
                moveq   \irqstat, \irqstat, lsr #8
                addeq   \irqnr, \irqnr, #8
                tst     \irqstat, #0x0f
                moveq   \irqstat, \irqstat, lsr #4
                addeq   \irqnr, \irqnr, #4
                tst     \irqstat, #0x03
                moveq   \irqstat, \irqstat, lsr #2
                addeq   \irqnr, \irqnr, #2
                tst     \irqstat, #0x01
                addeq   \irqnr, \irqnr, #1

1002:           teq     \irqnr, #IRQ_GPP_0_7                            @ check if it a Gpp int 0 - 7
                beq     1003f
                teq     \irqnr, #IRQ_GPP_8_15                           @ check if its a Gpp int 8 - 15
                beq     1003f
#if defined(CONFIG_MACH_TS78XX)
                teq     \irqnr, #IRQ_GPP_16_23                          @ check if it a Gpp int 16 - 23
                beq     1003f
                teq     \irqnr, #IRQ_GPP_24_31                          @ check if its a Gpp int 24 - 31
                beq     1003f
#endif
		          teq	\irqnr, #IRQ_DOORBELL
		          beq	1005f
                teq     \irqnr, #96                                     @ if irq isn't 96 unset the flag
                b       1004f


1005:
		ldr	\base, =(ORION5X_REGS_VIRT_BASE+0x20400)    @@@@ 0xf1020400	@ Host to CPU Doorbell base 
		ldr	\tmp, [\base, #0x4]	@ Host to CPU Doorbell mask reg
		ldr	\irqstat, [\base]	@ Host to CPU Doorbell reg
		mov	\irqnr, #64		@ Doorbell IRQ nums start at 64
		ands	\irqstat, \irqstat, \tmp
		bne	1001b
		b	1004f


1003:           ldr     \base, =MV_ASM_GPP_IRQ_BASE_REG
		ldr     \tmp, [\base, #MV_ASM_GPP_IRQ_MASK_REG]
		HTOLL	\tmp, \base
		ldr	\base, =MV_ASM_GPP_IRQ_BASE_REG				
                ldr     \irqstat, [\base, #MV_ASM_GPP_IRQ_CAUSE_REG]    @ get cause
		HTOLL	\irqstat, \base
                ldr	\base, =MV_ASM_GPP_IRQ_BASE_REG				
                mov     \irqnr, #IRQ_ASM_GPP_START
                ands    \irqstat, \irqstat, \tmp                        @ and mask and cause
                bne     1001b                                           @ if eq to 0 (flag set) error return, else go to 1001

1004:
                .endm


#endif
