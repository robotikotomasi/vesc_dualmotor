/*
 * chtypes.h compatibility shim
 * Provides ChibiOS type definitions.
 */

#ifndef CHTYPES_H_COMPAT
#define CHTYPES_H_COMPAT

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef uint32_t systime_t;
typedef uint32_t sysinterval_t;
typedef uint32_t eventmask_t;
typedef uint32_t eventflags_t;
typedef int32_t  msg_t;

/* ChibiOS ST frequency (tick rate) */
#ifndef CH_CFG_ST_FREQUENCY
#define CH_CFG_ST_FREQUENCY  1000
#endif

#endif /* CHTYPES_H_COMPAT */
