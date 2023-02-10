#include "cfl.h"
#include "../pci.h"
#include "../lvds.h"

#include <lil/imports.h>

#include "crtc.h"
#include "dp.h"
#include "gtt.h"
#include "plane.h"

#define PCI_MGGC0 0x50

static uint32_t get_gtt_size(void* device) {
    uint16_t mggc0 = lil_pci_readw(device, PCI_MGGC0);
    uint8_t size = 1 << ((mggc0 >> 6) & 0b11);

    return size * 1024 * 1024;
}


void lil_init_cfl_gpu(LilGpu* ret, void* device) {
    ret->gpio_start = 0xC0000;

    uintptr_t base;
    uintptr_t len;
    lil_get_bar(device, 0, &base, &len);
    ret->mmio_start = (uintptr_t)lil_map(base, len);

    ret->gtt_address = ret->mmio_start + (len / 2); // Half of the BAR space is registers, half is GTT PTEs
    ret->gtt_size = get_gtt_size(device);

    ret->vmem_clear = lil_cfl_vmem_clear;
    ret->vmem_map = lil_cfl_vmem_map;

    lil_get_bar(device, 2, &base, &len);
    ret->vram = (uintptr_t)lil_map(base, len);

    ret->num_connectors = 1;
    ret->connectors = lil_malloc(sizeof(LilConnector) * ret->num_connectors);

    ret->connectors[0].id = 0;
    ret->connectors[0].type = DISPLAYPORT;
    ret->connectors[0].on_pch = true;
    ret->connectors[0].get_connector_info = lil_cfl_dp_get_connector_info;
    ret->connectors[0].is_connected = lil_cfl_dp_is_connected;
    ret->connectors[0].get_state = lil_cfl_dp_get_state;
    ret->connectors[0].set_state = lil_cfl_dp_set_state;

    lil_cfl_dp_init(ret, &ret->connectors[0]);

    ret->connectors[0].crtc = lil_malloc(sizeof(LilCrtc));
    ret->connectors[0].crtc->transcoder = TRANSCODER_EDP;
    ret->connectors[0].crtc->connector = &ret->connectors[0];
    ret->connectors[0].crtc->num_planes = 1;
    ret->connectors[0].crtc->planes = lil_malloc(sizeof(LilPlane));
    for (int i = 0; i < ret->connectors[0].crtc->num_planes; i++) {
        ret->connectors[0].crtc->planes[i].enabled = 0;
        ret->connectors[0].crtc->planes[i].pipe_id = 0;
        ret->connectors[0].crtc->planes[i].update_surface = lil_cfl_update_primary_surface;
    }
    ret->connectors[0].crtc->pipe_id = 0;
    ret->connectors[0].crtc->commit_modeset = lil_cfl_commit_modeset;
    ret->connectors[0].crtc->shutdown = lil_cfl_shutdown;
}
