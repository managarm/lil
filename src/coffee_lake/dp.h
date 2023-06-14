#pragma once

#include <lil/intel.h>

void lil_cfl_dp_get_mode_info(LilGpu* gpu, LilModeInfo* out);

bool lil_cfl_dp_is_connected (struct LilGpu* gpu, struct LilConnector* connector);
LilConnectorInfo lil_cfl_dp_get_connector_info (struct LilGpu* gpu, struct LilConnector* connector);
void lil_cfl_dp_set_state (struct LilGpu* gpu, struct LilConnector* connector, uint32_t state);
uint32_t lil_cfl_dp_get_state (struct LilGpu* gpu, struct LilConnector* connector);

void lil_cfl_dp_init(struct LilGpu* gpu, struct LilConnector* connector);

void lil_cfl_dp_disable(struct LilGpu* gpu, struct LilConnector* connector);
void lil_cfl_dp_post_disable(struct LilGpu* gpu, struct LilConnector* connector);

void lil_cfl_dp_pre_enable(struct LilGpu* gpu, struct LilConnector* connector);

typedef struct {
    uint64_t link_m, link_n;
    uint64_t data_m, data_n;
} LilDpMnValues;
LilDpMnValues lil_cfl_dp_calculate_mn(LilGpu* gpu, LilModeInfo* mode);

uint8_t dp_aux_native_read(struct LilGpu* gpu, uint16_t addr);
void dp_aux_native_readn(struct LilGpu* gpu, uint16_t addr, size_t n, void *buf);
void dp_aux_native_write(struct LilGpu* gpu, uint16_t addr, uint8_t v);
void dp_aux_native_writen(struct LilGpu* gpu, uint16_t addr, size_t n, void *buf);
