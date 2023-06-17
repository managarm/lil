#include "ivb.h"
#include "../pci.h"
#include <lil/intel.h>
#include "../lvds.h"
#include "../gmbus.h"
#include "crtc.h"
#include "interrupt.h"
#include "plane.h"
#include "gtt.h"

#include <lil/imports.h>

#define PCI_MGGC0 0x50

static uint32_t get_gtt_size(void* device) {
    uint16_t mggc0 = lil_pci_readw(device, PCI_MGGC0);
    uint8_t size = ((mggc0 >> 8) & 0b11);
    if(size == 3)
        lil_panic("Reserved GTT Size");

    return size * 1024 * 1024;
}

PllLilLimits ivb_limits_single_lvds = {
    .dot = { .min = 25000, .max = 350000 },
    .vco = { .min = 1760000, .max = 3510000 },
    .n = { .min = 1, .max = 3 },
    .m = { .min = 79, .max = 118 },
    .m1 = { .min = 12, .max = 22 },
    .m2 = { .min = 5, .max = 9 },
    .p = { .min = 28, .max = 112 },
    .p1 = { .min = 2, .max = 8 },
    .p2 = { .dot_limit = 225000, .slow = 14, .fast = 14 },
};

void lil_init_ivb_gpu(LilGpu* ret, void* device) {
    ret->gpio_start = 0xC0000;
    uintptr_t base;
    uintptr_t len;
    lil_get_bar(device, 0, &base, &len);
    ret->mmio_start = (uintptr_t)lil_map(base, len);

    ret->gtt_address = ret->mmio_start + (2 * 1024 * 1024);
    ret->gtt_size = get_gtt_size(device);

    ret->gtt_address = ret->mmio_start + (len / 2); // Half of the BAR space is registers, half is GTT PTEs
    ret->gtt_size = get_gtt_size(device);
    ret->vmem_clear = lil_ivb_vmem_clear;
    ret->vmem_map = lil_ivb_vmem_map;

    lil_get_bar(device, 2, &base, &len);
    ret->vram = (uintptr_t)lil_map(base, len);

    //TODO currently we only support LVDS

    ret->num_connectors = 1;
    ret->connectors = lil_malloc(sizeof(LilConnector) * ret->num_connectors);

    ret->connectors[0].id = 0;
    ret->connectors[0].type = LVDS;
    ret->connectors[0].get_connector_info = lil_lvds_get_connector_info;
    ret->connectors[0].is_connected = lil_lvds_is_connected;
    ret->connectors[0].get_state = lil_lvds_get_state;
    ret->connectors[0].set_state = lil_lvds_set_state;
    ret->connectors[0].limits = ivb_limits_single_lvds;
    ret->connectors[0].on_pch = true;
    ret->connectors[0].crtc = lil_malloc(sizeof(LilCrtc));
    ret->connectors[0].crtc->connector = &ret->connectors[0];
    ret->connectors[0].crtc->num_planes = 1;
    ret->connectors[0].crtc->planes = lil_malloc(sizeof(LilPlane));
    for (int i = 0; i < ret->connectors[0].crtc->num_planes; i++) {
        ret->connectors[0].crtc->planes[i].enabled = 0;
        ret->connectors[0].crtc->planes[i].pipe_id = 0;
        ret->connectors[0].crtc->planes[i].update_surface = lil_ivb_update_primary_surface;
    }
    ret->connectors[0].crtc->pipe_id = 0;
    ret->connectors[0].crtc->commit_modeset = lil_ivb_commit_modeset;
    ret->connectors[0].crtc->shutdown = lil_ivb_shutdown;
    ret->enable_display_interrupt = lil_ivb_enable_display_interrupt;
    ret->process_interrupt = lil_ivb_process_display_interrupt;
}
