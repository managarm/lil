#include "pci.h"

#include <lil/imports.h>

#include <stddef.h>

void lil_get_bar(void* device, int bar, uintptr_t* obase, uintptr_t* len) {
	/* disable Bus Mastering and Memory + I/O space access */
	uint16_t command = lil_pci_readw(device, 4);
	lil_pci_writew(device, 4, command & ~7); /* Bus Master | Memory Space | I/O Space */

    size_t reg_index = 0x10 + bar * 4;
    uint64_t bar_low = lil_pci_readd(device, reg_index), bar_size_low;
    uint64_t bar_high = 0, bar_size_high = ~0;

    uintptr_t base;
    size_t size;

    int is_mmio = !(bar_low & 1);
    int is_64bit = is_mmio && ((bar_low >> 1) & 0b11) == 0b10;

    if (is_64bit)
        bar_high = lil_pci_readd(device, reg_index + 4);

    base = ((bar_high << 32) | bar_low) & ~(is_mmio ? (0b1111) : (0b11));

    lil_pci_writed(device, reg_index, 0xFFFFFFFF);
    bar_size_low = lil_pci_readd(device, reg_index);
    lil_pci_writed(device, reg_index, bar_low);

    if (is_64bit) {
        lil_pci_writed(device, reg_index + 4, 0xFFFFFFFF);
        bar_size_high = lil_pci_readd(device, reg_index + 4);
        lil_pci_writed(device, reg_index + 4, bar_high);
    }

    size = ((bar_size_high << 32) | bar_size_low) & ~(is_mmio ? (0b1111) : (0b11));
    size = ~size + 1;

    *obase = base;
    *len = size;

	/* restore Bus Mastering and Memory + I/O space access */
	lil_pci_writew(device, 4, command);
}
