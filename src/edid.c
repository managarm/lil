#include "edid.h"

void edid_timing_to_mode(DetailTiming timing, LilModeInfo* mode) {
    mode->clock = timing.pixelClock * 10;

    uint32_t horz_active = timing.horzActive | ((uint32_t)(timing.horzActiveBlankMsb >> 4) << 8);
    uint32_t horz_blank = timing.horzBlank | ((uint32_t)(timing.horzActiveBlankMsb & 0xF) << 8);
    uint32_t horz_sync_offset = timing.horzSyncOffset | ((uint32_t)(timing.syncMsb >> 6) << 8);
    uint32_t horz_sync_pulse = timing.horzSyncPulse | (((uint32_t)(timing.syncMsb >> 4) & 0x3) << 8);
    mode->hactive = horz_active;
    mode->hsyncStart = horz_active + horz_sync_offset;
    mode->hsyncEnd = horz_active + horz_sync_offset + horz_sync_pulse;
    mode->htotal = horz_active + horz_blank;

    uint32_t vert_active =  timing.vertActive | ((uint32_t)(timing.vertActiveBlankMsb >> 4) << 8);
    uint32_t vert_blank = timing.vertBlank | ((uint32_t)(timing.vertActiveBlankMsb & 0xF) << 8);
    uint32_t vert_sync_offset = (timing.vertSync >> 4) | (((uint32_t)(timing.syncMsb >> 2) & 0x3) << 4);
    uint32_t vert_sync_pulse = (timing.vertSync & 0xF) | ((uint32_t)(timing.syncMsb & 0x3) << 4);
    mode->vactive = vert_active;
    mode->vsyncStart = vert_active + vert_sync_offset;
    mode->vsyncEnd = vert_active + vert_sync_offset + vert_sync_pulse;
    mode->vtotal = vert_active + vert_blank;
}