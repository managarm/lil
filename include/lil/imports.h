#pragma once

#include <stdint.h>
#include <stddef.h>

enum LilLogType {
	ERROR,
	WARNING,
	INFO,
	VERBOSE,
};

__attribute__((weak)) void lil_pci_writeb(void* device, uint16_t offset, uint8_t val);
__attribute__((weak)) uint8_t lil_pci_readb(void* device, uint16_t offset);
__attribute__((weak)) void lil_pci_writew(void* device, uint16_t offset, uint16_t val);
__attribute__((weak)) uint16_t lil_pci_readw(void* device, uint16_t offset);
__attribute__((weak)) void lil_pci_writed(void* device, uint16_t offset, uint32_t val);
__attribute__((weak)) uint32_t lil_pci_readd(void* device, uint16_t offset);
__attribute__((weak)) void lil_sleep(uint64_t ms);
__attribute__((weak)) void lil_usleep(uint64_t us);
__attribute__((weak)) void* lil_malloc(size_t s);
__attribute__((weak)) void lil_free(void* p);
__attribute__((weak)) void* lil_map(size_t loc, size_t len);
__attribute__((weak)) void lil_log(enum LilLogType type, const char *fmt, ...);
__attribute__((weak, noreturn)) void lil_panic(const char* msg);
