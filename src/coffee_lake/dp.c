#include "dp.h"
#include "../imports.h"
#include "../edid.h"

#define PWR_WELL_CTL2 0x45404

#define DC_STATE_EN 0x45504

#define PP_STATUS 0xC7200
#define PP_CONTROL 0xC7204

#define PP_CONTROL_ON (1 << 0)
#define PP_CONTROL_RESET (1 << 1)
#define PP_CONTROL_BACKLIGHT (1 << 2)
#define PP_CONTROL_FORCE_VDD (1 << 3)

#define PP_STATUS_ON_STATUS (1 << 31)
#define PP_STATUS_GET_SEQUENCE_PROGRESS(v) (((v) >> 28) & 0x3)
#define PP_STATUS_SEQUENCE_NONE 0

#define DDI_AUX_CTL(c) (0x64010 + ((c) * 0x100))
#define DDI_AUX_DATA(c) (0x64014 + ((c) * 0x100))

#define DDI_AUX_CTL_BUSY (1 << 31)
#define DDI_AUX_CTL_DONE (1 << 30)
#define DDI_AUX_CTL_IRQ (1 << 29)
#define DDI_AUX_CTL_TIMEOUT (1 << 28)
#define DDI_AUX_CTL_TIMEOUT_VAL_MAX (0x3 << 26)
#define DDI_AUX_CTL_RX_ERR (1 << 25)
#define DDI_AUX_SET_MSG_SIZE(sz) (((sz) & 0x1F) << 20)
#define DDI_AUX_GET_MSG_SIZE(ctl) (((ctl) >> 20) & 0x1F)

#define DDI_AUX_CTL_SKL_FW_SYNC_PULSE(v) (((v) - 1) << 5)
#define DDI_AUX_CTL_SKL_SYNC_PULSE(v) ((v) - 1)

#define DDI_AUX_I2C_WRITE 0x0
#define DDI_AUX_I2C_READ 0x1
#define DDI_AUX_I2C_MOT (1 << 2) // Middle of Transfer

#define DDI_AUX_NATIVE_WRITE 0x8
#define DDI_AUX_NATIVE_READ 0x9

#define DPCD_REV 0x0
#define DPCD_DOWNSTREAMPORT_PRESENT 0x5
#define DPCD_EDP_CONFIGURATION_CAP 0xD
#define DPCD_DOWNSTREAM_PORT0_CAP 0x80

#define DPCD_SET_POWER 0x600
#define DPCD_POWER_D0 1
#define DPCD_POWER_D3 2


#define DP_TP_CTL(c) (0x64040 + ((c) * 0x100))
#define DP_TP_STS(c) (0x64044 + ((c) * 0x100))

#define DP_TP_CTL_ENABLE (1 << 31)
#define DP_TP_CTL_TRAIN_MASK (7 << 8)
#define DP_TP_CTL_TRAIN_PATTERN1 (0 << 8)

#define DDI_BUF_CTL(c) (0x64000 + ((c) * 0x100))

#define DDI_BUF_CTL_ENABLE (1 << 31)
#define DDI_BUF_CTL_IDLE (1 << 7)
#define DDI_BUF_CTL_DISPLAY_DETECTED (1 << 0)

#define BLC_PWM_CPU_CTL2 0x48250
#define BLC_PWM_CPU_CTL 0x48254

#define VIDEO_DIP_CTL(transcoder) (((transcoder) == TRANSCODER_EDP) ? 0x6F200 : (0x60200 + ((transcoder) * 0x1000)))

#define TRANS_CLK_SEL(pipe) (0x46140 + ((pipe) * 4))

#define DPLL_CTRL2 0x6C05C
#define DPLL_CTRL2_DDI_CLK_OFF(port) (1 << ((port) + 15))

typedef struct AuxRequest {
    uint8_t request;
    uint32_t address;
    uint8_t size;
    uint8_t tx[16];
} AuxRequest;

typedef struct AuxResponse {
    uint8_t reply, size;
    uint8_t data[20];
} AuxResponse;

static uint32_t dp_pack_aux(uint8_t* data, uint8_t size) {
    uint32_t v = 0;
    if(size > 4)
        size = 4;

    for(uint8_t i = 0; i < size; i++)
        v |= ((uint32_t)data[i]) << ((3 - i) * 8);

    return v;
}

static void dp_unpack_aux(uint32_t src, uint8_t* data, uint8_t size) {
    if(size > 4)
        size = 4;

    for(uint8_t i = 0; i < size; i++)
        data[i] = (src >> ((3 - i) * 8));
}

static uint32_t dp_get_aux_clock_div(struct LilGpu* gpu, int i) {
    if(gpu->gen >= 9) {
        return i ? 0 : 1; // Skylake and up will automatically derive the divider
    } else {
        lil_panic("DP Unknown GPU Gen");
    }
}

static uint32_t dp_get_aux_send_ctl(struct LilGpu* gpu, uint8_t txsize) {
    if(gpu->gen >= 9) {
        return DDI_AUX_CTL_BUSY | DDI_AUX_CTL_DONE | DDI_AUX_CTL_TIMEOUT | DDI_AUX_CTL_TIMEOUT_VAL_MAX | DDI_AUX_CTL_RX_ERR | DDI_AUX_SET_MSG_SIZE(txsize) | DDI_AUX_CTL_SKL_FW_SYNC_PULSE(32) | DDI_AUX_CTL_SKL_SYNC_PULSE(32);
    } else {
        lil_panic("DP Unknown GPU gen");
    }
}

static uint32_t dp_aux_wait(struct LilGpu* gpu) {
    volatile uint32_t* ctl = (uint32_t*)(gpu->mmio_start + DDI_AUX_CTL(0));
    int timeout = 10;

    while(timeout > 0) {
        if((*ctl & DDI_AUX_CTL_BUSY) == 0)
            return *ctl;

        lil_sleep(1);
        timeout--;
    }

    return *ctl;
}

static uint8_t dp_aux_xfer(struct LilGpu* gpu, uint8_t* tx, uint8_t tx_size, uint8_t* rx, uint8_t rxsize) {
    volatile uint32_t* ctl = (uint32_t*)(gpu->mmio_start + DDI_AUX_CTL(0));
    volatile uint32_t* data = (uint32_t*)(gpu->mmio_start + DDI_AUX_DATA(0));

    // Try to wait for any previous AUX cmds
    for(uint8_t i = 0; i < 3; i++) {
        if((*ctl & DDI_AUX_CTL_BUSY) == 0)
            break;
        lil_sleep(1);
    }

    if(*ctl & DDI_AUX_CTL_BUSY)
        lil_panic("DP AUX Busy");

    if(tx_size > 20 || rxsize > 20)
        lil_panic("DP AUX xfer too big");

    int clock = 0;
    bool done = false;
    uint32_t status = 0;
    uint32_t aux_clock_divider = 0;
    while((aux_clock_divider = dp_get_aux_clock_div(gpu, clock++))) {
        uint32_t send = dp_get_aux_send_ctl(gpu, tx_size);

        for(uint8_t i = 0; i < 5; i++) {
            for(size_t j = 0; j < tx_size; j += 4)
                data[j / 4] = dp_pack_aux(tx + j, tx_size - j);

            *ctl = send;

            status = dp_aux_wait(gpu);

            *ctl = status | DDI_AUX_CTL_DONE | DDI_AUX_CTL_TIMEOUT | DDI_AUX_CTL_RX_ERR;

            if(status & DDI_AUX_CTL_TIMEOUT) {
                lil_sleep(1);
                continue;
            }

            if(status & DDI_AUX_CTL_RX_ERR) {
                lil_sleep(1);
                continue;
            }

            if(status & DDI_AUX_CTL_DONE) {
                done = true;
                break;
            }
        }
    }

    if(!done)
        lil_panic("Failed to do DP AUX transfer");

    if(status & DDI_AUX_CTL_TIMEOUT)
        lil_panic("DP AUX Timeout");

    if(status & DDI_AUX_CTL_RX_ERR)
        lil_panic("DP AUX Receive error");

    
    uint8_t received_bytes = DDI_AUX_GET_MSG_SIZE(status);
    if(received_bytes == 0 || received_bytes > 20)
        lil_panic("DP AUX Unknown recv bytes");

    if(received_bytes > rxsize)
        received_bytes = rxsize;

    for(size_t i = 0; i < received_bytes; i += 4)
        dp_unpack_aux(data[i / 4], rx + i, received_bytes - i);

    return received_bytes; 
}

static AuxResponse dp_aux_cmd(struct LilGpu* gpu, AuxRequest req) {
    AuxResponse res = {0};
    uint8_t tx[20] = {0};
    uint8_t rx[20] = {0};

    // CMD Header
    tx[0] = (req.request << 4) | ((req.address >> 16) & 0xF);
    tx[1] = (req.address >> 8) & 0xFF;
    tx[2] = req.address & 0xFF;
    tx[3] = req.size - 1;

    uint8_t op = req.request & ~DDI_AUX_I2C_MOT;
    if(op == DDI_AUX_I2C_WRITE || op == DDI_AUX_NATIVE_WRITE) {
        uint8_t tx_size = req.size ? (req.size + 4) : 3,
                rx_size = 2;

        for(size_t i = 0; i < req.size; i++)
            tx[i + 4] = req.tx[i];

        uint8_t ret = dp_aux_xfer(gpu, tx, tx_size, rx, rx_size);
        if(ret > 0) {
            res.reply = rx[0] >> 4;
        }
    } else if(op == DDI_AUX_I2C_READ || op == DDI_AUX_NATIVE_READ) {
        uint8_t tx_size = req.size ? 4 : 3,
                rx_size = req.size + 1;

        uint8_t ret = dp_aux_xfer(gpu, tx, tx_size, rx, rx_size);
        if(ret > 0) {
            res.reply = rx[0] >> 4;
            res.size = ret - 1;

            for(uint8_t i = 0; i < res.size; i++)
                res.data[i] = rx[i + 1];
        }
    } else {
        lil_panic("Unknown DP AUX cmd");
    }

    return res;
}

// TODO: Check res.reply for any NACKs or DEFERs, instead of assuming i2c success, same for dp_aux_i2c_write
static void dp_aux_i2c_read(struct LilGpu* gpu, uint16_t addr, uint8_t len, uint8_t* buf) {
    AuxRequest req = {0};
    AuxResponse res = {0};
    req.request = DDI_AUX_I2C_READ | DDI_AUX_I2C_MOT;
    req.address = addr;
    req.size = 0;    
    dp_aux_cmd(gpu, req);

    for(size_t i = 0; i < len; i++) {
        req.size = 1;

        res = dp_aux_cmd(gpu, req);
        buf[i] = res.data[0];
    }

    req.request = DDI_AUX_I2C_READ;
    req.address = 0;
    req.size = 0;    
    dp_aux_cmd(gpu, req);
}

static void dp_aux_i2c_write(struct LilGpu* gpu, uint16_t addr, uint8_t len, uint8_t* buf) {
    AuxRequest req = {0};
    req.request = DDI_AUX_I2C_WRITE | DDI_AUX_I2C_MOT;
    req.address = addr;
    req.size = 0;    
    dp_aux_cmd(gpu, req);

    for(size_t i = 0; i < len; i++) {
        req.size = 1;
        req.tx[0] = buf[i];

        dp_aux_cmd(gpu, req);
    }

    req.request = DDI_AUX_I2C_READ;
    req.address = 0;
    req.size = 0;    
    dp_aux_cmd(gpu, req);
}

uint8_t dp_aux_native_read(struct LilGpu* gpu, uint16_t addr) {
    AuxRequest req = {0};
    req.request = DDI_AUX_NATIVE_READ;
    req.address = addr;
    req.size = 1;    
    AuxResponse res = dp_aux_cmd(gpu, req);

    return res.data[0];
}

static void dp_aux_native_write(struct LilGpu* gpu, uint16_t addr, uint8_t v) {
    AuxRequest req = {0};
    req.request = DDI_AUX_I2C_WRITE;
    req.address = addr;
    req.size = 1;
    req.tx[0] = v;
    dp_aux_cmd(gpu, req);
}

#define DDC_SEGMENT 0x30
#define DDC_ADDR 0x50
#define EDID_SIZE 128

static void dp_aux_read_edid(struct LilGpu* gpu, DisplayData* buf) {
    uint32_t block = 0;

    uint8_t segment = block / 2;
    dp_aux_i2c_write(gpu, DDC_SEGMENT, 1, &segment);
    
    uint8_t start = block * EDID_SIZE;
    dp_aux_i2c_write(gpu, DDC_ADDR, 1, &start);
    
    dp_aux_i2c_read(gpu, DDC_ADDR, EDID_SIZE, (uint8_t*)buf);
}

bool lil_cfl_dp_is_connected (struct LilGpu* gpu, struct LilConnector* connector) {
    volatile uint32_t* ctl = (uint32_t*)(gpu->mmio_start + DDI_BUF_CTL(0));
    return *ctl & DDI_BUF_CTL_DISPLAY_DETECTED; // TODO: This only works for DDI A, other DDIs are detected through `SFUSE_STRAP`
}

LilConnectorInfo lil_cfl_dp_get_connector_info (struct LilGpu* gpu, struct LilConnector* connector) {
    (void)connector;
    LilConnectorInfo ret = {0};
    LilModeInfo* info = lil_malloc(sizeof(LilModeInfo) * 4);
    
    DisplayData edid = {0};
    dp_aux_read_edid(gpu, &edid);

    int j = 0;
    for(int i = 0; i < 4; i++) { // Maybe 4 Detailed Timings
        if(edid.detailTimings[i].pixelClock == 0)
            continue; // Not a timing descriptor

        edid_timing_to_mode(edid.detailTimings[i], &info[j++]);
    }

    ret.modes = info;
    ret.num_modes = j;
    return ret;
}

void lil_cfl_dp_set_state (struct LilGpu* gpu, struct LilConnector* connector, uint32_t state) {
    // TODO
}

uint32_t lil_cfl_dp_get_state (struct LilGpu* gpu, struct LilConnector* connector) {
    // TODO
}

static uint32_t get_pp_control(struct LilGpu* gpu) {
    volatile uint32_t* ctl = (uint32_t*)(gpu->mmio_start + PP_CONTROL);
    
    uint32_t v = *ctl;

    // Completely undocumented? Linux does it under the guise of "Unlocking registers??"
    if((v >> 16) != 0xABCD) {
        v &= ~(0xFFFF << 16);
        v |= (0xABCD << 16);
    }

    return v;
}

static void edp_panel_on(struct LilGpu* gpu, struct LilConnector* connector) {
    if(connector->type != EDP)
        return;
    
    volatile uint32_t* sts = (uint32_t*)(gpu->mmio_start + PP_STATUS);
    volatile uint32_t* ctl = (uint32_t*)(gpu->mmio_start + PP_CONTROL);
    
    uint32_t v = get_pp_control(gpu);
    v |= PP_CONTROL_ON | PP_CONTROL_RESET | PP_CONTROL_FORCE_VDD;

    *ctl = v;
    (void)*ctl;

    while(true) {
        v = *sts;
        if(v & PP_STATUS_ON_STATUS && PP_STATUS_GET_SEQUENCE_PROGRESS(v) == PP_STATUS_SEQUENCE_NONE) // Wait until the display is on, and not in a power sequence
            break;
    }
}

static void edp_panel_off(struct LilGpu* gpu, struct LilConnector* connector) {
    if(connector->type != EDP)
        return;
    
    volatile uint32_t* sts = (uint32_t*)(gpu->mmio_start + PP_STATUS);
    volatile uint32_t* ctl = (uint32_t*)(gpu->mmio_start + PP_CONTROL);
    
    uint32_t v = get_pp_control(gpu);
    v &= ~(PP_CONTROL_ON | PP_CONTROL_RESET | PP_CONTROL_BACKLIGHT | PP_CONTROL_FORCE_VDD);

    *ctl = v;
    (void)*ctl;

    while(true) {
        v = *sts;
        if((v & PP_STATUS_ON_STATUS) == 0 && PP_STATUS_GET_SEQUENCE_PROGRESS(v) == PP_STATUS_SEQUENCE_NONE) // Wait until the display is off, and not in a power sequence
            break;
    }
}

static void edp_panel_vdd_on(struct LilGpu* gpu, struct LilConnector* connector) {
    if(connector->type != EDP)
        return;
    
    volatile uint32_t* sts = (uint32_t*)(gpu->mmio_start + PP_STATUS);
    volatile uint32_t* ctl = (uint32_t*)(gpu->mmio_start + PP_CONTROL);
    
    uint32_t v = get_pp_control(gpu);
    v |= PP_CONTROL_FORCE_VDD;

    *ctl = v;
    (void)*ctl;
}

static void edp_panel_backlight_off(struct LilGpu* gpu, struct LilConnector* connector) {
    if(connector->type != EDP)
        return;
    
    volatile uint32_t* sts = (uint32_t*)(gpu->mmio_start + PP_STATUS);
    volatile uint32_t* ctl = (uint32_t*)(gpu->mmio_start + PP_CONTROL);
    
    uint32_t v = get_pp_control(gpu);
    v &= ~PP_CONTROL_BACKLIGHT;
    *ctl = v;
    (void)*ctl;

    volatile uint32_t* blc_ctl2 = (uint32_t*)(gpu->mmio_start + BLC_PWM_CPU_CTL2);
    volatile uint32_t* blc_ctl = (uint32_t*)(gpu->mmio_start + BLC_PWM_CPU_CTL);

    *blc_ctl2 &= ~(1 << 31);
    *blc_ctl &= ~(1 << 31);

    v = *blc_ctl;
    v &= ~0xFFFF; // Duty Cycle 0
    *blc_ctl = v;
}

static void dp_set_sink_power(struct LilGpu* gpu, struct LilConnector* connector, bool on) {
    uint8_t rev = dp_aux_native_read(gpu, DPCD_REV);
    if(rev < 0x11)
        return;

    if(on) {
        lil_panic("TODO: Turn Sink on");
    } else {
        uint8_t downstream = dp_aux_native_read(gpu, DPCD_DOWNSTREAMPORT_PRESENT);
        if(rev == 0x11 && (downstream & 1)) {
            uint8_t port0 = dp_aux_native_read(gpu, DPCD_DOWNSTREAM_PORT0_CAP);
            if(port0 & (1 << 3)) { // HPD Aware
                return;
            }
        }

        dp_aux_native_write(gpu, DPCD_SET_POWER, DPCD_POWER_D3);
    }
}

void lil_cfl_dp_init(struct LilGpu* gpu, struct LilConnector* connector) {
    // From Skylake PRMs Vol Display, AUX Programming sequence
    volatile uint32_t* pwr = (uint32_t*)(gpu->mmio_start + PWR_WELL_CTL2);
    volatile uint32_t* cstate = (uint32_t*)(gpu->mmio_start + DC_STATE_EN);

    *pwr |= (1 << 29); // Request Power Well 1 start

    while(((*pwr >> 28) & 1) == 0)
        ;

    *pwr |= (1 << 3); // Request DDI A and E start

    while(((*pwr >> 2) & 1) == 0)
        ;

    *pwr |= (1 << 1); // Request AUXIO start

    while(((*pwr >> 0) & 1) == 0)
        ;

    *cstate &= ~0x3; // Disable DC5 and DC6 state

    uint8_t cap = dp_aux_native_read(gpu, DPCD_EDP_CONFIGURATION_CAP);
    connector->type = (cap != 0) ? EDP : DISPLAYPORT; // Hacky, but it should work on any eDP display that is semi-modern, better option is to parse VBIOS

    edp_panel_on(gpu, connector);
}

void lil_cfl_dp_disable(struct LilGpu* gpu, struct LilConnector* connector) {
    edp_panel_backlight_off(gpu, connector);
}

void lil_cfl_dp_post_disable(struct LilGpu* gpu, struct LilConnector* connector) {
    volatile uint32_t* video_dip_ctl = (uint32_t*)(gpu->mmio_start + VIDEO_DIP_CTL(connector->crtc->transcoder));
    uint32_t v = *video_dip_ctl;
    v &= ~((1 << 12) | (1 << 16) | (1 << 8) | (1 << 4) | (1 << 0) | (1 << 28) | (1 << 20));
    *video_dip_ctl = v;
    (void)*video_dip_ctl;

    dp_set_sink_power(gpu, connector, false);

    if(connector->crtc->transcoder != TRANSCODER_EDP) {
        volatile uint32_t* trans_clk_sel = (uint32_t*)(gpu->mmio_start + TRANS_CLK_SEL(connector->crtc->pipe_id));
        *trans_clk_sel = 0;
    }


    bool wait = false;
    volatile uint32_t* ddi_buf_ctl = (uint32_t*)(gpu->mmio_start + DDI_BUF_CTL(connector->crtc->pipe_id));
    v = *ddi_buf_ctl;
    if(v & DDI_BUF_CTL_ENABLE) {
        v &= ~DDI_BUF_CTL_ENABLE;
        *ddi_buf_ctl = v;
        wait = true;
    }

    volatile uint32_t* dp_tp_ctl = (uint32_t*)(gpu->mmio_start + DP_TP_CTL(connector->crtc->pipe_id));
    v = *dp_tp_ctl;
    v &= ~(DP_TP_CTL_ENABLE | DP_TP_CTL_TRAIN_MASK);
    v |= DP_TP_CTL_TRAIN_PATTERN1;
    *dp_tp_ctl = v;

    if(wait)
        lil_sleep(1); // 518us - 1000us, just give it some time

    edp_panel_vdd_on(gpu, connector);
    edp_panel_off(gpu, connector);

    volatile uint32_t* dpll_ctl2 = (uint32_t*)(gpu->mmio_start + DPLL_CTRL2);
    v = *dpll_ctl2;
    v |= DPLL_CTRL2_DDI_CLK_OFF(connector->crtc->pipe_id);
    *dpll_ctl2 = v;
}