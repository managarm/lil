#pragma once

#include <stdint.h>

void lil_get_bar(void* device, int num, uintptr_t* base, uintptr_t* len);

#define PCI_MGGC0 0x50
#define PCI_ASLS 0xFC
