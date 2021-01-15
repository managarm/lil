#include "gmbus.h"
#include "imports.h"

#include <stddef.h>

#define GMBUS_SELECT 0x5100
#define GMBUS_COMMAND_STATUS 0x5104
#define GMBUS_STATUS 0x5108
#define GMBUS_DATA 0x510C
#define GMBUS_IRMASK 0x5110
#define GMBUS_2BYTEINDEX 0x5120

#define GMBUS_HW_RDY (1 << 11)
#define GMBUS_NAK (1 << 10)
#define GMBUS_HW_WAIT_PHASE (1 << 14)
#define GMBUS_CYCLE_WAIT (1 << 25)
#define GMBUS_CYCLE_INDEX (1 << 26)
#define GMBUS_LEN_MASK 511
#define GMBUS_LEN_SHIFT 16
#define GMBUS_LEN(x) ((x & GMBUS_LEN_MASK) << GMBUS_LEN_SHIFT)
#define GMBUS_OFFSET_MASK 127
#define GMBUS_OFFSET_SHIFT 1
#define GMBUS_OFFSET(x) ((x & GMBUS_OFFSET_MASK) << GMBUS_OFFSET_SHIFT)
#define GMBUS_READ 1
#define GMBUS_SW_READY (1 << 30)
#define GMBUS_CLEAR_INTERRUPT (1 << 31)

typedef struct EdidTiming {
    uint8_t resolution;
    uint8_t frequency;
} EdidTiming;

typedef struct DetailTiming {
    uint16_t pixelClock;
    uint8_t horzActive;
    uint8_t horzBlank;
    uint8_t horzActiveBlankMsb;
    uint8_t vertActive;
    uint8_t vertBlank;
    uint8_t vertActiveBlankMsb;
    uint8_t horzSyncOffset;
    uint8_t horzSyncPulse;
    uint8_t vertSync;
    uint8_t syncMsb;
    uint8_t dimensionWidth;
    uint8_t dimensionHeight;
    uint8_t dimensionMsb;
    uint8_t horzBorder;
    uint8_t vertBorder;
    uint8_t features;
} DetailTiming;

typedef struct DisplayData {
    uint8_t magic[8];
    uint16_t vendorId;
    uint16_t productId;
    uint32_t serialNumber;
    uint8_t manufactureWeek;
    uint8_t manufactureYear;
    uint8_t structVersion;
    uint8_t structRevision;
    uint8_t inputParameters;
    uint8_t screenWidth;
    uint8_t screenHeight;
    uint8_t gamma;
    uint8_t features;
    uint8_t colorCoordinates[10];
    uint8_t estTimings1;
    uint8_t estTimings2;
    uint8_t vendorTimings;
    EdidTiming standardTimings[8];
    DetailTiming detailTimings[4];
    uint8_t numExtensions;
    uint8_t checksum;
} DisplayData;

static void gmbus_wait_progress(LilGpu* gpu) {
    volatile uint32_t* gmbus_status = (uint32_t*)(gpu->mmio_start + GMBUS_STATUS + gpu->gpio_start);
    while(1) {
        uint32_t status = *gmbus_status;
        if(status & (GMBUS_NAK)) {
            lil_panic("NAK on gmbus");
        }
        if(status & GMBUS_HW_RDY) {
            break;
        }
    }
}

static void gmbus_wait_completion(LilGpu* gpu) {
    volatile uint32_t* gmbus_status = (uint32_t*)(gpu->mmio_start + GMBUS_STATUS + gpu->gpio_start);
    while(1) {
        uint32_t status = *gmbus_status;
        if(status & (GMBUS_NAK)) {
            lil_panic("NAK on gmbus");
        }
        if(status & GMBUS_HW_WAIT_PHASE) {
            break;
        }
    }
}

static void gmbus_read(LilGpu* gpu, int pin_pair, uint32_t offset, uint32_t len, uint8_t* buf) {
    volatile uint32_t* gmbus_select = (uint32_t*)(gpu->mmio_start + GMBUS_SELECT + gpu->gpio_start);
    volatile uint32_t* gmbus_command = (uint32_t*)(gpu->mmio_start + GMBUS_COMMAND_STATUS + gpu->gpio_start);
    volatile uint32_t* gmbus_data = (uint32_t*)(gpu->mmio_start + GMBUS_DATA + gpu->gpio_start);

    uint32_t gmbus_select_val = *gmbus_select;
    uint32_t gmbus_command_val = *gmbus_command;
    gmbus_select_val &= 0xFFFFF800;
    gmbus_select_val = pin_pair;
    gmbus_command_val = GMBUS_CYCLE_WAIT | GMBUS_CYCLE_INDEX |
                        GMBUS_LEN(len) | GMBUS_OFFSET(offset) |
                        GMBUS_READ | GMBUS_SW_READY;
    *gmbus_command = 0;
    uint32_t temp = *gmbus_command;
    temp |= GMBUS_CLEAR_INTERRUPT;
    *gmbus_command = temp;
    temp = *gmbus_command;
    temp &= ~GMBUS_CLEAR_INTERRUPT;
    *gmbus_command = temp;
    *gmbus_select = gmbus_select_val;
    *gmbus_command = gmbus_command_val;

    size_t progress = 0;
    while(progress < len) {
        gmbus_wait_progress(gpu);
        uint32_t data = *gmbus_data;
        for(int i = 0; i < 4; i++) {
            if(progress == len)
                break;
            buf[progress++] |= data >> (8 * i);
        }
    }
    gmbus_wait_completion(gpu);
}

static void edid_to_mode(DisplayData edid, LilModeInfo* mode) {
    mode->clock = edid.detailTimings[0].pixelClock * 10;

    uint32_t horz_active = edid.detailTimings[0].horzActive | ((uint32_t)(edid.detailTimings[0].horzActiveBlankMsb >> 4) << 8);
    uint32_t horz_blank = edid.detailTimings[0].horzBlank | ((uint32_t)(edid.detailTimings[0].horzActiveBlankMsb & 0xF) << 8);
    uint32_t horz_sync_offset = edid.detailTimings[0].horzSyncOffset | ((uint32_t)(edid.detailTimings[0].syncMsb >> 6) << 8);
    uint32_t horz_sync_pulse = edid.detailTimings[0].horzSyncPulse | (((uint32_t)(edid.detailTimings[0].syncMsb >> 4) & 0x3) << 8);
    mode->hactive = horz_active;
    mode->hsyncStart = horz_active + horz_sync_offset;
    mode->hsyncEnd = horz_active + horz_sync_offset + horz_sync_pulse;
    mode->htotal = horz_active + horz_blank;

    uint32_t vert_active =  edid.detailTimings[0].vertActive | ((uint32_t)(edid.detailTimings[0].vertActiveBlankMsb >> 4) << 8);
    uint32_t vert_blank = edid.detailTimings[0].vertBlank | ((uint32_t)(edid.detailTimings[0].vertActiveBlankMsb & 0xF) << 8);
    uint32_t vert_sync_offset = (edid.detailTimings[0].vertSync >> 4) | (((uint32_t)(edid.detailTimings[0].syncMsb >> 2) & 0x3) << 4);
    uint32_t vert_sync_pulse = (edid.detailTimings[0].vertSync & 0xF) | ((uint32_t)(edid.detailTimings[0].syncMsb & 0x3) << 4);
    mode->vactive = vert_active;
    mode->vsyncStart = vert_active + vert_sync_offset;
    mode->vsyncEnd = vert_active + vert_sync_offset + vert_sync_pulse;
    mode->vtotal = vert_active + vert_blank;
}

void lil_get_mode_info(LilGpu* gpu, LilModeInfo* out, int pin_pair) {
    DisplayData edid = {0};
    gmbus_read(gpu, pin_pair, 0x50, 128, (uint8_t*)&edid);

    for (int i = 0; i < 4; i++) {
        edid_to_mode(edid, &out[i]);
    }
}
