#include <lil/intel.h>
#include <lil/imports.h>
#include <lil/vbt.h>

#include "src/pci.h"
#include "src/vbt/opregion.hpp"
#include "src/vbt/vbt.hpp"

//
// TODO: vbt parsing should be generation independent
//

const struct vbt_header *lil_vbt_locate(LilGpu *gpu) {
	uint32_t asls_phys = lil_pci_readd(gpu->dev, PCI_ASLS);
	if(!asls_phys) {
		lil_panic("ACPI OpRegion not supported");
	}

	auto opregion = reinterpret_cast<struct opregion_header *>(lil_map(asls_phys, 0x2000));
	if(memcmp(opregion, OPREGION_SIGNATURE, 16)) {
		lil_panic("ACPI OpRegion signature mismatch");
	}

	lil_log(VERBOSE, "ACPI OpRegion %u.%u.%u\n", opregion->over.major, opregion->over.minor, opregion->over.revision);
	opregion_asle *asle = NULL;

	if(opregion->mbox & MBOX_ASLE) {
		asle = reinterpret_cast<opregion_asle *>(uintptr_t(opregion) + OPREGION_ASLE_OFFSET);
	}

	if(opregion->over.major >= 2 && asle && asle->rvda && asle->rvds) {
		uint64_t rvda = asle->rvda;
		lil_log(VERBOSE, "Raw RVDA in asle->rvda: 0x%lx\n", asle->rvda);

		// In OpRegion v2.1+, rvda was changed to a relative offset
		if(opregion->over.major > 2 || (opregion->over.major == 2 && opregion->over.minor >= 1)) {
			if(rvda < OPREGION_SIZE) {
				lil_log(WARNING, "VBT base shouldn't be within OpRegion, but it is!\n");
			}

			rvda += asls_phys;
		}

		lil_log(INFO, "VBT RVDA: 0x%lx\n", rvda);

		/* OpRegion 2.0: rvda is a physical address */
		void *vbt = lil_map(rvda, asle->rvds);

		const struct vbt_header *vbt_header = vbt_get_header(vbt, asle->rvds);
		if(!vbt_header) {
			lil_log(WARNING, "VBT header from ACPI OpRegion ASLE is invalid!\n");
			lil_unmap(vbt, asle->rvds);
		} else {
			return vbt_header;
		}
	}

	if(!(opregion->mbox & MBOX_VBT)) {
		lil_log(ERROR, "ACPI OpRegion does not support VBT mailbox when it should!\n");
		lil_panic("Invalid ACPI OpRegion");
	}

	size_t vbt_size = ((opregion->mbox & MBOX_ASLE_EXT) ? OPREGION_ASLE_EXT_OFFSET : OPREGION_SIZE) - OPREGION_VBT_OFFSET;
	void *vbt = lil_map(asls_phys + OPREGION_VBT_OFFSET, vbt_size);

	const struct vbt_header *vbt_header = vbt_get_header(vbt, vbt_size);

	if(!vbt_header) {
		lil_log(ERROR, "Reading VBT from ACPI OpRegion mailbox 4 failed!\n");
		lil_panic("No supported method to obtain VBT left");
	}

	return vbt_header;
}

void vbt_init(LilGpu *gpu) {
	gpu->vbt_header = lil_vbt_locate(gpu);
	lil_log(VERBOSE, "lil: gpu->vbt_header addr 0x%lx\n", (uintptr_t) gpu->vbt_header);

	const struct bdb_header *bdb_hdr = vbt_get_bdb_header(gpu->vbt_header);
	if(!bdb_hdr)
		lil_panic("BDB header not found");

	auto driver_features_block = vbt_get_bdb_block<bdb_driver_features>(gpu->vbt_header, BDB_DRIVER_FEATURES);
	if(!driver_features_block)
		lil_panic("BDB driver features not found");

	auto general_defs = vbt_get_bdb_block<bdb_general_definitions>(gpu->vbt_header, BDB_GENERAL_DEFINITIONS);
	if(!general_defs)
		lil_panic("BDB general definitions not found");
}

void vbt_setup_children(LilGpu *gpu) {
	size_t con_id = 0;

	const struct bdb_driver_features *driver_features = vbt_get_bdb_block<bdb_driver_features>(gpu->vbt_header, BDB_DRIVER_FEATURES);
	const struct bdb_general_definitions *general_defs = vbt_get_bdb_block<bdb_general_definitions>(gpu->vbt_header, BDB_GENERAL_DEFINITIONS);
	gpu->num_connectors = con_id;
}
