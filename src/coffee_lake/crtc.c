#include "crtc.h"

#include <lil/imports.h>

#include "dp.h"

#define TRANS_HTOTAL(pipe) (0x60000 + ((pipe) * 0x1000))
#define TRANS_HBLANK(pipe) (0x60004 + ((pipe) * 0x1000))
#define TRANS_HSYNC(pipe) (0x60008 + ((pipe) * 0x1000))

#define TRANS_VTOTAL(pipe) (0x6000C + ((pipe) * 0x1000))
#define TRANS_VBLANK(pipe) (0x60010 + ((pipe) * 0x1000))
#define TRANS_VSYNC(pipe) (0x60014 + ((pipe) * 0x1000))

#define TRANS_DATAM(pipe) (0x60030 + ((pipe) * 0x1000))
#define TRANS_DATAN(pipe) (0x60034 + ((pipe) * 0x1000))

#define TRANS_LINKM(pipe) (0x60040 + ((pipe) * 0x1000))
#define TRANS_LINKN(pipe) (0x60044 + ((pipe) * 0x1000))

#define TRANS_CONF(pipe) (0x70008 + ((pipe) * 0x1000))
#define TRANS_CONF_EDP 0x7F008
#define TRANS_CONF_ENABLE (1u << 31)
#define TRANS_CONF_STATUS (1 << 30)

#define TRANS_CLK_SEL(pipe) (0x46140 + ((pipe) * 0x4))
#define TRANS_CLK_SEL_MASK (0x7 << 29)
#define TRANS_CLK_SEL_DDI_B (2 << 29)
#define TRANS_CLK_SEL_DDI_C (3 << 29)
#define TRANS_CLK_SEL_DDI_D (4 << 29)
#define TRANS_CLK_SEL_DDI_E (5 << 29)

#define DDI_BUF_CTL(c) (0x64000 + ((c) * 0x100))

#define DDI_BUF_CTL_ENABLE (1u << 31)
#define DDI_BUF_CTL_PORT_MASK (0x7 << 27)
#define DDI_BUF_CTL_MODE_SELECT_MASK (0x7 << 24)
#define DDI_BUF_CTL_SYNC_SELECT_MASK (0x3 << 18)
#define DDI_BUF_CTL_SYNC_ENABLE (1 << 15)

#define TRANS_DDI_FUNC_CTL(c) (0x60400 + ((c) * 0x1000))
#define TRANS_DDI_FUNC_CTL_EDP 0x6F400

#define TRANS_DDI_FUNC_CTL_ENABLE (1u << 31)
#define TRANS_DDI_FUNC_SELECT_MASK (7 << 28)

#define TRANS_DDI_MODE_DP_SST 0x2
#define TRANS_DDI_MODE_SHIFT 24

#define TRANS_DDI_BPC_SHIFT 20

#define PF_CTL(pipe, i) (0x68180 + ((pipe) * 0x800) + ((i) * 0x100))
#define PF_WIN_SZ(pipe, i) (0x68174 + ((pipe) * 0x800) + ((i) * 0x100))
#define PF_WIN_POS(pipe, i) (0x68170 + ((pipe) * 0x800) + ((i) * 0x100))

#define DAC_CTL 0xE1100

#define TRANS_CHICKEN2(pipe) (0xF0064 + ((pipe) * 0x1000))
#define TRANS_CHICKEN2_TIMING_OVERRIDE (1u << 31)

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
#define FDI_RX_ENABLE (1u << 31)

#define IPS_CTL 0x43408
#define IPS_STATUS 0x43401

#define CLKGAKE_DIS_PSL(pipe) (0x46520 + ((pipe) * 0x4))
#define DUPS1_GATING_DIS (1 << 15)
#define DUPS2_GATING_DIS (1 << 19)



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

    volatile uint32_t* trans_ddi_func_ctl;
    if(crtc->connector->type == EDP)
        trans_ddi_func_ctl = (uint32_t*)(gpu->mmio_start + TRANS_DDI_FUNC_CTL_EDP);
    else
        trans_ddi_func_ctl = (uint32_t*)(gpu->mmio_start + TRANS_DDI_FUNC_CTL(crtc->pipe_id));

    set_mask(trans_ddi_func_ctl, false, TRANS_DDI_FUNC_CTL_ENABLE | TRANS_DDI_FUNC_SELECT_MASK);

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
    if(crtc->connector->type == DISPLAYPORT || crtc->connector->type == EDP) {
        lil_cfl_dp_pre_enable(gpu, crtc->connector);
    } else {
        lil_panic("Unknown connector type");
    }

    // eDP always uses DDI A's clk
    if(crtc->connector->type != EDP) {
        lil_panic("TODO: Non eDP modeset");

        //volatile uint32_t* trans_clk_sel = (uint32_t*)(gpu->mmio_start + TRANS_CLK_SEL(crtc->pipe_id));
    }

    LilModeInfo mode = crtc->current_mode;
    LilDpMnValues mn = lil_cfl_dp_calculate_mn(gpu, &mode);

    volatile uint32_t* htotal = (uint32_t*)(gpu->mmio_start + TRANS_HTOTAL(crtc->pipe_id));
    volatile uint32_t* hblank = (uint32_t*)(gpu->mmio_start + TRANS_HBLANK(crtc->pipe_id));
    volatile uint32_t* hsync = (uint32_t*)(gpu->mmio_start  + TRANS_HSYNC(crtc->pipe_id));
    volatile uint32_t* vtotal = (uint32_t*)(gpu->mmio_start + TRANS_VTOTAL(crtc->pipe_id));
    volatile uint32_t* vblank = (uint32_t*)(gpu->mmio_start + TRANS_VBLANK(crtc->pipe_id));
    volatile uint32_t* vsync = (uint32_t*)(gpu->mmio_start  + TRANS_VSYNC(crtc->pipe_id));
    *htotal = ((mode.htotal - 1) << 16) | (mode.hactive - 1);
    *hblank = ((mode.htotal - 1) << 16) | (mode.hactive - 1);
    *hsync = ((mode.hsyncEnd - 1) << 16) | (mode.hsyncStart - 1);
    *vtotal = ((mode.vtotal - 1) << 16) | (mode.vactive - 1);
    *vblank = ((mode.vtotal - 1) << 16) | (mode.vactive - 1);
    *vsync = ((mode.vsyncEnd - 1) << 16) | (mode.vsyncStart - 1);

    volatile uint32_t* pipe_m_1 = (uint32_t*)(gpu->mmio_start + TRANS_DATAM(crtc->pipe_id));
    volatile uint32_t* pipe_n_1 = (uint32_t*)(gpu->mmio_start + TRANS_DATAN(crtc->pipe_id));
    volatile uint32_t* pipe_link_m_1 = (uint32_t*)(gpu->mmio_start + TRANS_LINKM(crtc->pipe_id));
    volatile uint32_t* pipe_link_n_1 = (uint32_t*)(gpu->mmio_start + TRANS_LINKN(crtc->pipe_id));
    *pipe_m_1 = (0b111111 << 25) | mn.data_m;
    *pipe_n_1 = mn.data_n;

    *pipe_link_m_1 = mn.link_m;
    *pipe_link_n_1 = mn.link_n;

    volatile uint32_t* trans_ddi_func_ctl = (uint32_t*)(gpu->mmio_start + TRANS_DDI_FUNC_CTL(crtc->pipe_id));
    if(crtc->connector->type == EDP)
        trans_ddi_func_ctl = (uint32_t*)(gpu->mmio_start + TRANS_DDI_FUNC_CTL_EDP);

    uint8_t bpc_modes[] = {
        -1, -1, -1, -1, -1, -1, 2, -1, 0, -1, 1, -1, 3
    };

    uint8_t bpc_mode = bpc_modes[mode.bpc];
    if(bpc_mode == -1)
        lil_panic("Unknown BPC mode");

    // TODO: Select DDI here, ignored for eDP
    *trans_ddi_func_ctl = TRANS_DDI_FUNC_CTL_ENABLE | (TRANS_DDI_MODE_DP_SST << TRANS_DDI_MODE_SHIFT) | (bpc_mode << TRANS_DDI_BPC_SHIFT) | (3 << 16) | (((dp_aux_native_read(gpu, 2) & 0xF) - 1) << 1);

    volatile uint32_t* trans_conf = (uint32_t*)(gpu->mmio_start + TRANS_CONF(crtc->pipe_id));
    if(crtc->connector->type == EDP)
        trans_conf = (uint32_t*)(gpu->mmio_start + TRANS_CONF_EDP);

    *trans_conf |= TRANS_CONF_ENABLE;

    while(!(*trans_conf & TRANS_CONF_STATUS))
        ;
}
