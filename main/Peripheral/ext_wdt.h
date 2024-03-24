#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void ext_wdt_start(uint32_t pulse_interval_ms);
void ext_wdt_stop(void);

#ifdef __cplusplus
}
#endif