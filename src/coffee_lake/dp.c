#include "dp.h"
#include "../imports.h"
#include "../edid.h"

#define PWR_WELL_CTL2 0x45404

#define DC_STATE_EN 0x45504

#define PP_STATUS 0xC7200
#define PP_CONTROL 0xC7204

#define PP_CONTROL_ON (1 << 0)
#define PP_CONTROL_RESET (1 << 1)
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


#define DP_TP_CTL(c) (0x64040 + ((c) * 0x100))
#define DP_TP_STS(c) (0x64044 + ((c) * 0x100))

#define DDI_BUF_CTL(c) (0x64000 + ((c) * 0x100))

#define DDI_BUF_CTL_DISPLAY_DETECTED (1 << 0)

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
    if(op == DDI_AUX_I2C_WRITE) {
        uint8_t tx_size = req.size ? (req.size + 4) : 3,
                rx_size = 2;

        for(size_t i = 0; i < req.size; i++)
            tx[i + 4] = req.tx[i];

        uint8_t ret = dp_aux_xfer(gpu, tx, tx_size, rx, rx_size);
        if(ret > 0) {
            res.reply = rx[0] >> 4;
        }
    } else if(op == DDI_AUX_I2C_READ) {
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

// TODO: Check res.reply for any NACKs or DEFERs, instead of assuming i2c success, same for dp_aux_write
static void dp_aux_read(struct LilGpu* gpu, uint16_t addr, uint8_t len, uint8_t* buf) {
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

static void dp_aux_write(struct LilGpu* gpu, uint16_t addr, uint8_t len, uint8_t* buf) {
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

#define DDC_SEGMENT 0x30
#define DDC_ADDR 0x50
#define EDID_SIZE 128

static void dp_aux_read_edid(struct LilGpu* gpu, DisplayData* buf) {
    uint32_t block = 0;

    uint8_t segment = block / 2;
    dp_aux_write(gpu, DDC_SEGMENT, 1, &segment);
    
    uint8_t start = block * EDID_SIZE;
    dp_aux_write(gpu, DDC_ADDR, 1, &start);
    
    dp_aux_read(gpu, DDC_ADDR, EDID_SIZE, (uint8_t*)buf);
}

bool lil_cfl_dp_is_connected (struct LilGpu* gpu, struct LilConnector* connector) {
    volatile uint32_t* ctl = (uint32_t*)(gpu->mmio_start + DDI_AUX_CTL(0));
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

static void edp_panel_on(struct LilGpu* gpu) {
    volatile uint32_t* sts = (uint32_t*)(gpu->mmio_start + PP_STATUS);
    volatile uint32_t* ctl = (uint32_t*)(gpu->mmio_start + PP_CONTROL);
    
    uint32_t v = *ctl;

    // Completely undocumented? Linux does it under the guise of "Unlocking registers??"
    if((v >> 16) != 0xABCD) {
        v &= ~(0xFFFF << 16);
        v |= (0xABCD << 16);
    }

    v |= PP_CONTROL_ON | PP_CONTROL_RESET | PP_CONTROL_FORCE_VDD;

    *ctl = v;
    (void)*ctl;

    while(true) {
        v = *sts;
        if(v & PP_STATUS_ON_STATUS && PP_STATUS_GET_SEQUENCE_PROGRESS(v) == PP_STATUS_SEQUENCE_NONE) // Wait until the display is on, and not in a power sequence
            break;
    }
}

void lil_cfl_dp_init(struct LilGpu* gpu, struct LilConnector* connector) {
    edp_panel_on(gpu);

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
}