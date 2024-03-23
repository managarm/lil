#include <lil/intel.h>
#include "ivy_bridge/ivb.h"

#include <lil/imports.h>

void lil_init_gpu(LilGpu* ret, void* device) {
   uint32_t config_0 = lil_pci_readd(device, 0);
    if (config_0 == 0xffffffff) {
        return;
    }
    uint16_t device_id = (uint16_t)(config_0 >> 16);
    uint16_t vendor_id = (uint16_t)config_0;

    if (vendor_id != 0x8086) {
        return;
    }

	ret->dev = device;

    switch (device_id) {
        case 0x0166 : {
            ret->gen = GEN_IVB;
            lil_init_ivb_gpu(ret, device);
            break;
        }
    }
}
