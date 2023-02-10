#include "gmbus.h"
#include "edid.h"

#include <lil/imports.h>

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
#define GMBUS_CLEAR_INTERRUPT (1u << 31)



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

uint8_t lil_gmbus_get_mode_info(LilGpu* gpu, LilModeInfo* out, int pin_pair) {
    DisplayData edid = {0};
    gmbus_read(gpu, pin_pair, 0x50, 128, (uint8_t*)&edid);

    int j = 0;
    for (int i = 0; i < 4; i++) {
        if(edid.detailTimings[i].pixelClock == 0)
            continue; // Not a timing descriptor

        edid_timing_to_mode(&edid, edid.detailTimings[i], &out[j++]);
    }

    return j;
}
