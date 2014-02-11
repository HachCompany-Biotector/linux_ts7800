/*
 * linux/drivers/video/ts7kvfb.h
 *
 * Framebuffer device for the TS-7KV board.
 *
 * Based on the TS-7300 FB driver By Fotis Loukos <fotisl@csd.auth.gr>
 *
 * Based on skeletonfb.c for new api by James Simmons and the original driver
 * for the 2.4 series by Eddie Dawydiuk.
 */

#if !defined (_TS7KVFB_H)
#define _TS7KVFB_H

#define VIDEORAM_BASE       0xED000000
#define VIDEORAM_SIZE       1024 * 1024
#define VIDEOREGS_BASE      0xEF000000
#define VIDEOREGS_SIZE      10

#define BLTCTRL             0
#define BLTSZ               2
#define SRCBLT              4
#define DSTBLT              6
#define VIDCTRL             8

#define PALETTE_COLORS      16
#define CMAP_LEN            256
#define TS7KV_BPP          16

#define TS7KV_MAXWIDTH     800

#define REG_ID0 0
#define REG_ID1 1
#define REG_CTRL0 4
#define REG_DEVSEL 0x1E
#define DONE_MASK 0x80
#define TS7KV_ID0 0x41
#define TS7KV_ID1 0x20
#define IO8_BASE 0xEE200000
#define IO16_BASE 0xEF200000
#define MEM16_BASE 0xED000000
#define REGS_CTRL_BASE 0xE0
#define REGS_VID_BASE 0x00
#define MEM_VID_BASE 0x00
#define REGS_CTRL_SIZE 8
#define REGS_VID_SIZE 0x20
#define MEM_VID_SIZE 0x100000

struct ts7kvfb_par {
  void __iomem *video_mem;
  void __iomem *fpga_regs;
  u16 width;
  u16 height;
  unsigned short SWIN_7KV;  
};

#endif
