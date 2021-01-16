#include "cfl.h"
#include "../pci.h"
#include "../imports.h"
#include "../lvds.h"

#include "dp.h"

void lil_init_cfl_gpu(LilGpu* ret, void* device) {
    ret->gpio_start = 0xC0000;

    uintptr_t base;
    uintptr_t len;
    lil_get_bar(device, 0, &base, &len);
    ret->mmio_start = (uintptr_t)lil_map(base, len);

    ret->num_connectors = 1;
    ret->connectors = lil_malloc(sizeof(LilConnector) * ret->num_connectors);

    ret->connectors[0].id = 0;
    ret->connectors[0].on_pch = true;
    ret->connectors[0].get_connector_info = lil_cfl_dp_get_connector_info;
    ret->connectors[0].is_connected = lil_cfl_dp_is_connected;
    ret->connectors[0].get_state = lil_cfl_dp_get_state;
    ret->connectors[0].set_state = lil_cfl_dp_set_state;

    lil_cfl_dp_init(ret, &ret->connectors[0]);
}
