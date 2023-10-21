#pragma once

#include <lil/intel.h>

enum DPCD_ADDRESSES {
	DPCD_REV = 0x0,
	MAX_LINK_RATE = 0x1,
	MAX_LANE_COUNT = 0x2,
	MAX_DOWNSPREAD = 0x3,
	DOWNSTREAMPORT_PRESENT = 0x5,
	EDP_CONFIGURATION_CAP = 0xD,
	TRAINING_AUX_RD_INTERVAL = 0xE,
	DOWNSTREAM_PORT0_CAP = 0x80,
	LINK_BW_SET = 0x100,
	LANE_COUNT_SET = 0x101,
	TRAINING_PATTERN_SET = 0x102,
	TRAINING_LANE0_SET = 0x103,
	LINK_RATE_SET = 0x115,
	DP_LANE0_1_STATUS = 0x202,
	DP_LANE2_3_STATUS = 0x203,
	LANE_ALIGN_STATUS_UPDATED = 0x204,
	ADJUST_REQUEST_LANE0_1 = 0x206,
	SET_POWER = 0x600,
	EDP_DPCD_REV = 0x700,
};

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
