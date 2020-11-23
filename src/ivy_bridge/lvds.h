#pragma once

#include "../intel.h"

bool lil_ivb_lvds_is_connected (struct LilGpu* gpu, struct LilConnector* connector);
LilConnectorInfo lil_ivb_lvds_get_connector_info (struct LilGpu* gpu, struct LilConnector* connector);
void lil_ivb_lvds_set_state (struct LilGpu* gpu, struct LilConnector* connector, uint32_t state);
uint32_t lil_ivb_lvds_get_state (struct LilGpu* gpu, struct LilConnector* connector);
