#pragma once

#include <concepts>
#include <stddef.h>
#include <lil/imports.h>

struct Base {
	static void *operator new(size_t size) {
		return lil_malloc(size);
	}

	static void *operator new[](size_t size) {
		return lil_malloc(size);
	}

	static void operator delete(void *ptr) {
		lil_free(ptr);
	}

	static void operator delete[](void *ptr) {
		lil_free(ptr);
	}

	virtual ~Base() = default;
};

template <typename T>
concept PciAccessType =
    std::same_as<T, uint8_t> || std::same_as<T, uint16_t> || std::same_as<T, uint32_t>;

enum LilGpuGen {
	GEN_IVB = 7,
	GEN_SKL = 9,
};

enum LilGpuSubGen {
	SUBGEN_NONE,
	SUBGEN_GEMINI_LAKE, // Gen9LP
	SUBGEN_KABY_LAKE, // Gen9p5
	SUBGEN_COFFEE_LAKE,
};

enum LilGpuVariant {
	H,
	ULT,
	ULX,
};

struct Gpu : LilGpu, public Base {
	Gpu(void *dev, uint16_t pch_dev)
	: pch_dev{pch_dev} {
		this->dev = dev;
	}

	template <PciAccessType T>
	T pci_read(uint16_t offset) {
		if constexpr (std::same_as<T, uint8_t>) {
			return lil_pci_readb(dev, offset);
		} else if constexpr (std::same_as<T, uint16_t>) {
			return lil_pci_readw(dev, offset);
		} else if constexpr (std::same_as<T, uint32_t>) {
			return lil_pci_readd(dev, offset);
		}
	}

	template <PciAccessType T>
	void pci_write(uint16_t offset, T value) {
		if constexpr (std::same_as<T, uint8_t>) {
			lil_pci_writeb(dev, offset, value);
		} else if constexpr (std::same_as<T, uint16_t>) {
			lil_pci_writew(dev, offset, value);
		} else if constexpr (std::same_as<T, uint32_t>) {
			lil_pci_writed(dev, offset, value);
		}
	}

	uint16_t pch_dev;
	const struct vbt_header *vbt_header;

	LilGpuGen gen;
	LilGpuSubGen subgen;
	LilGpuVariant variant;
	LilPchGen pch_gen;

	/* reference clock frequency in MHz */
	uint32_t ref_clock_freq;

	uint32_t mem_latency_first_set;
	uint32_t mem_latency_second_set;

	bool vco_8640;
	uint32_t boot_cdclk_freq;
	uint32_t cdclk_freq;
};
