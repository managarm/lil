#pragma once

#include <lil/intel.h>

namespace kbl::pcode {

enum class Mailbox : uint32_t {
};

bool rw(LilGpu *gpu, uint32_t *data0, uint32_t *data1, Mailbox mbox, uint32_t *timeout);

}
