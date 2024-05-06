#pragma once

#include <limits.h>
#include <optional>
#include <utility>

#include <lil/imports.h>

#include "src/helpers.h"

#include <stddef.h>
#include <stdint.h>

namespace kbl::watermarks {

std::optional<std::pair<size_t, size_t>> calculate(size_t pixel_clock_khz, size_t width, size_t htotal, uint8_t wm, size_t level);

}

# define do_div(n,base) ({					\
	uint32_t __base = (base);				\
	uint32_t __rem;						\
	__rem = ((uint64_t)(n)) % __base;			\
	(n) = ((uint64_t)(n)) / __base;				\
	__rem;							\
 })

static inline uint64_t mul_u32_u32(uint32_t a, uint32_t b)
{
	uint32_t high, low;

	asm ("mull %[b]" : "=a" (low), "=d" (high)
			 : [a] "a" (a), [b] "rm" (b) );

	return low | ((uint64_t)high) << 32;
}

#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))

#define DIV_ROUND_DOWN_ULL(ll, d) \
	({ unsigned long long _tmp = (ll); do_div(_tmp, d); _tmp; })

#define DIV_ROUND_UP_ULL(ll, d) \
	DIV_ROUND_DOWN_ULL((unsigned long long)(ll) + (d) - 1, (d))

#define min(a, b) (a < b ? a : b)
#define max(a, b) (a > b ? a : b)

typedef struct {
	uint32_t val;
} uint_fixed_16_16_t;

#define FP_16_16_MAX ((uint_fixed_16_16_t){ .val = UINT_MAX })

static inline bool is_fixed16_zero(uint_fixed_16_16_t val)
{
	return val.val == 0;
}

static inline uint_fixed_16_16_t u32_to_fixed16(uint32_t val)
{
	uint_fixed_16_16_t fp = { .val = val << 16 };

	lil_assert(!(val > UINT16_MAX));

	return fp;
}

static inline uint32_t fixed16_to_u32_round_up(uint_fixed_16_16_t fp)
{
	return DIV_ROUND_UP(fp.val, 1 << 16);
}

static inline uint32_t fixed16_to_u32(uint_fixed_16_16_t fp)
{
	return fp.val >> 16;
}

static inline uint_fixed_16_16_t min_fixed16(uint_fixed_16_16_t min1,
					     uint_fixed_16_16_t min2)
{
	uint_fixed_16_16_t min = { .val = min(min1.val, min2.val) };

	return min;
}

static inline uint_fixed_16_16_t max_fixed16(uint_fixed_16_16_t max1,
					     uint_fixed_16_16_t max2)
{
	uint_fixed_16_16_t max = { .val = max(max1.val, max2.val) };

	return max;
}

static inline uint_fixed_16_16_t clamp_u64_to_fixed16(uint64_t val)
{
	uint_fixed_16_16_t fp = { .val = (uint32_t)val };

	lil_assert(!(val > UINT32_MAX));

	return fp;
}

static inline uint32_t div_round_up_fixed16(uint_fixed_16_16_t val,
				       uint_fixed_16_16_t d)
{
	return DIV_ROUND_UP(val.val, d.val);
}

static inline uint32_t mul_round_up_u32_fixed16(uint32_t val, uint_fixed_16_16_t mul)
{
	uint64_t tmp;

	tmp = mul_u32_u32(val, mul.val);
	tmp = DIV_ROUND_UP_ULL(tmp, 1 << 16);
	lil_assert(!(tmp > UINT32_MAX));

	return (uint32_t)tmp;
}

static inline uint_fixed_16_16_t mul_fixed16(uint_fixed_16_16_t val,
					     uint_fixed_16_16_t mul)
{
	uint64_t tmp;

	tmp = mul_u32_u32(val.val, mul.val);
	tmp = tmp >> 16;

	return clamp_u64_to_fixed16(tmp);
}

static inline uint_fixed_16_16_t div_fixed16(uint32_t val, uint32_t d)
{
	uint64_t tmp;

	tmp = (uint64_t)val << 16;
	tmp = DIV_ROUND_UP_ULL(tmp, d);

	return clamp_u64_to_fixed16(tmp);
}

static inline uint32_t div_round_up_u32_fixed16(uint32_t val, uint_fixed_16_16_t d)
{
	uint64_t tmp;

	tmp = (uint64_t)val << 16;
	tmp = DIV_ROUND_UP_ULL(tmp, d.val);
	lil_assert(!(tmp > UINT32_MAX));

	return (uint32_t)tmp;
}

static inline uint_fixed_16_16_t mul_u32_fixed16(uint32_t val, uint_fixed_16_16_t mul)
{
	uint64_t tmp;

	tmp = mul_u32_u32(val, mul.val);

	return clamp_u64_to_fixed16(tmp);
}

static inline uint_fixed_16_16_t add_fixed16(uint_fixed_16_16_t add1,
					     uint_fixed_16_16_t add2)
{
	uint64_t tmp;

	tmp = (uint64_t)add1.val + add2.val;

	return clamp_u64_to_fixed16(tmp);
}

static inline uint_fixed_16_16_t add_fixed16_u32(uint_fixed_16_16_t add1,
						 uint32_t add2)
{
	uint_fixed_16_16_t tmp_add2 = u32_to_fixed16(add2);
	uint64_t tmp;

	tmp = (uint64_t)add1.val + tmp_add2.val;

	return clamp_u64_to_fixed16(tmp);
}
