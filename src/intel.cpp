#include <lil/intel.h>
#include <lil/imports.h>

#include "src/ivy_bridge/ivb.h"
#include "src/kaby_lake/kbl.hpp"
#include "src/pch.hpp"
#include "src/pci.h"
#include "src/vbt/vbt.hpp"

bool lil_init_gpu(LilGpu* ret, void* device, uint16_t pch_id) {
   uint32_t config_0 = lil_pci_readd(device, PCI_HDR_VENDOR_ID);
	if (config_0 == 0xffffffff)
		return false;

	uint16_t device_id = (uint16_t) (config_0 >> 16);
	uint16_t vendor_id = (uint16_t) config_0;
	if (vendor_id != 0x8086)
		return false;

	uint16_t pci_class = lil_pci_readw(device, PCI_HDR_SUBCLASS);
	if (pci_class != 0x300)
		return false;

	ret->dev = device;
	ret->pch_dev = pch_id;

	pch::get_gen(ret);
	ret->subgen = SUBGEN_NONE;

	switch (device_id) {
		case 0x0116:
		case 0x0166: {
			ret->gen = GEN_IVB;
			lil_init_ivb_gpu(ret, device);
			return true;
		}

		case 0x3184:
		case 0x3185:
		case 0x5916:
		case 0x3E9B:
		case 0x5917: {
			vbt_init(ret);

			uint8_t prog_if = lil_pci_readb(device, PCI_HDR_PROG_IF);
			lil_assert(!prog_if);
			kbl::init_gpu(ret);
			return true;
		}

		default:
			return false;
	}
}

