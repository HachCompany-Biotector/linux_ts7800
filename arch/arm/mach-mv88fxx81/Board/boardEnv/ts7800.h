/*
 * arch/arm/mach-mv88fxx81/Board/boardEnv/ts7800.h
 */

/*
 * TS7800 memory map:
 *
 * virt         phys            size
 * xxxxxxxx	E8000000        4K      TS-7800 RTC regs
 */

#define TS7800_RTC_BASE                 0xE8000000
#define TS7800_RTC_SIZE                 0x00001000
#define TS7800_RTC_INDEX                0x00000808	//0xE800_0808
#define TS7800_RTC_DATA                 0x0000080C	//0xE800_080C
