#pragma once

#include <stdint.h>
#include <stddef.h>

enum LilLogType {
	ERROR,
	WARNING,
	INFO,
	VERBOSE,
};

void *memcpy(void *restrict dest, const void *restrict src, size_t n);
int memcmp(const void *s1, const void *s2, size_t n);
void *memset(void *s, int c, size_t n);
int strcmp(const char *s1, const char *s2);

__attribute__((weak)) void lil_pci_writeb(void* device, uint16_t offset, uint8_t val);
__attribute__((weak)) uint8_t lil_pci_readb(void* device, uint16_t offset);
__attribute__((weak)) void lil_pci_writew(void* device, uint16_t offset, uint16_t val);
__attribute__((weak)) uint16_t lil_pci_readw(void* device, uint16_t offset);
__attribute__((weak)) void lil_pci_writed(void* device, uint16_t offset, uint32_t val);
__attribute__((weak)) uint32_t lil_pci_readd(void* device, uint16_t offset);

__attribute__((weak)) void lil_outb(uint16_t port, uint8_t value);
__attribute__((weak)) void lil_outw(uint16_t port, uint16_t value);
__attribute__((weak)) void lil_outd(uint16_t port, uint32_t value);
__attribute__((weak)) uint8_t lil_inb(uint16_t port);
__attribute__((weak)) uint16_t lil_inw(uint16_t port);
__attribute__((weak)) uint32_t lil_ind(uint16_t port);

__attribute__((weak)) void lil_sleep(uint64_t ms);
__attribute__((weak)) void lil_usleep(uint64_t us);
__attribute__((weak)) void* lil_malloc(size_t s);
__attribute__((weak)) void lil_free(void* p);
__attribute__((weak)) void* lil_map(size_t loc, size_t len);
__attribute__((weak)) void lil_unmap(void *loc, size_t len);
__attribute__((weak)) void lil_log(enum LilLogType type, const char *fmt, ...);
__attribute__((weak, noreturn)) void lil_panic(const char* msg);
