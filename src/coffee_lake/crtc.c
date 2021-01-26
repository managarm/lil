#include "crtc.h"
#include "../imports.h"

#include "dp.h"

#define TRANS_CONF(pipe) (0x70008 + ((pipe) * 0x1000))


#define TRANS_CONF_ENABLE (1 << 31)
#define TRANS_CONF_STATUS (1 << 30)

#define DDI_BUF_CTL(c) (0x64000 + ((c) * 0x100))

#define DDI_BUF_CTL_ENABLE (1 << 31)
#define DDI_BUF_CTL_PORT_MASK (0x7 << 27)
#define DDI_BUF_CTL_MODE_SELECT_MASK (0x7 << 24)
#define DDI_BUF_CTL_SYNC_SELECT_MASK (0x3 << 18)
#define DDI_BUF_CTL_SYNC_ENABLE (1 << 15)

#define PF_CTL(pipe, i) (0x68180 + ((pipe) * 0x800) + ((i) * 0x100))
#define PF_WIN_SZ(pipe, i) (0x68174 + ((pipe) * 0x800) + ((i) * 0x100))
#define PF_WIN_POS(pipe, i) (0x68170 + ((pipe) * 0x800) + ((i) * 0x100))

#define DAC_CTL 0xE1100

#define TRANS_CHICKEN2(pipe) (0xF0064 + ((pipe) * 0x1000))
#define TRANS_CHICKEN2_TIMING_OVERRIDE (1 << 31)

#define PIXCLK_GATE 0xC6020

#define SBI_ADDR 0xC6000
#define SBI_DATA 0xC6004
#define SBI_CTL_STAT 0xC6008

#define SBI_BUSY 1
#define SBI_DST_ICLK (0 << 16)
#define SBI_DST_MPHY (1 << 16)

#define SBI_OP_IO_READ (2 << 8)
#define SBI_OP_IO_WRITE (3 << 8)
#define SBI_OP_CR_READ (6 << 8)
#define SBI_OP_CR_WRITE (7 << 8)

#define SBI_SSCCTL6 0x60C
#define SBI_SSCCTL_DISABLE (1 << 0)

#define FDI_RX_CTL(pipe) (0xF000C + ((pipe) * 0x1000))
#define FDI_RX_ENABLE (1 << 31)



uint32_t sbi_reg_rw(struct LilGpu* gpu, uint16_t reg, uint32_t dst, uint32_t v, bool write) {
    volatile uint32_t* addr = (uint32_t*)(gpu->mmio_start + SBI_ADDR);
    volatile uint32_t* data = (uint32_t*)(gpu->mmio_start + SBI_DATA);
    volatile uint32_t* status = (uint32_t*)(gpu->mmio_start + SBI_CTL_STAT);

    while(*status & SBI_BUSY)
        ;

    *addr = reg << 16;
    *data = write ? v : 0;

    uint32_t cmd = 0;
    if(dst == SBI_DST_ICLK)
        cmd |= SBI_DST_ICLK | (write ? SBI_OP_CR_WRITE : SBI_OP_CR_READ);
    else
        cmd |= SBI_DST_MPHY | (write ? SBI_OP_IO_WRITE : SBI_OP_IO_WRITE);

    cmd |= SBI_BUSY;
    *status = cmd;

    while(*status & SBI_BUSY)
        ;

    if(!write)
        return *data;
    
    return 0;
}


static void set_mask (volatile uint32_t* reg, bool set, uint32_t mask) {
    if (set) {
        *reg = *reg | mask;
    } else {
        *reg = *reg & ~mask;
    }
}

static void wait_mask (volatile uint32_t* reg, bool set, uint32_t mask) {
    if (set) {
        while(!(*reg & mask)) {}
    } else {
        while((*reg & mask)) {}
    }
}

void lil_cfl_shutdown (struct LilGpu* gpu, struct LilCrtc* crtc) {
    if(crtc->connector->type == DISPLAYPORT || crtc->connector->type == EDP) {
        lil_cfl_dp_disable(gpu, crtc->connector);
    } else {
        lil_panic("Unknown connector type");
    }

    volatile uint32_t* trans_conf = (uint32_t*)(gpu->mmio_start + TRANS_CONF(crtc->pipe_id));
    set_mask(trans_conf, false, TRANS_CONF_ENABLE);
    wait_mask(trans_conf, false, TRANS_CONF_STATUS);

    volatile uint32_t* ddi_buf_ctl = (uint32_t*)(gpu->mmio_start + DDI_BUF_CTL(crtc->pipe_id));
    set_mask(ddi_buf_ctl, false, DDI_BUF_CTL_ENABLE | DDI_BUF_CTL_SYNC_ENABLE | DDI_BUF_CTL_SYNC_SELECT_MASK | DDI_BUF_CTL_PORT_MASK | DDI_BUF_CTL_MODE_SELECT_MASK);

    // TODO: PIPE_A and B have 2 scalers, C only has 1
    for(size_t i = 0; i < 2; i++) {
        volatile uint32_t* pf_ctl = (uint32_t*)(gpu->mmio_start + PF_CTL(crtc->pipe_id, i));
        volatile uint32_t* pf_win_sz = (uint32_t*)(gpu->mmio_start + PF_WIN_SZ(crtc->pipe_id, i));
        volatile uint32_t* pf_win_pos = (uint32_t*)(gpu->mmio_start + PF_WIN_POS(crtc->pipe_id, i));

        *pf_ctl = 0;
        *pf_win_pos = 0;
        *pf_win_sz = 0;
    }

    if(crtc->connector->type == DISPLAYPORT || crtc->connector->type == EDP) {
        lil_cfl_dp_post_disable(gpu, crtc->connector);
    } else {
        lil_panic("Unknown connector type");
    }
}

void lil_cfl_commit_modeset (struct LilGpu* gpu, struct LilCrtc* crtc) {

}