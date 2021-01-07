// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#ifndef _MYST_DEFS_H
#define _MYST_DEFS_H

#define MYST_PRINTF_FORMAT(N, M) __attribute__((format(printf, N, M)))

#define MYST_STATIC_ASSERT(COND) _Static_assert(COND, __FILE__)

#define MYST_INLINE static __inline__

#define MYST_WEAK __attribute__((weak))

#define MYST_NORETURN __attribute__((__noreturn__))

#define MYST_COUNTOF(ARR) (sizeof(ARR) / sizeof((ARR)[0]))

#define MYST_WEAK_ALIAS(OLD, NEW) \
    extern __typeof(OLD) NEW __attribute__((__weak__, alias(#OLD)))

#define MYST_NOINLINE __attribute__((noinline))

#endif /* _MYST_DEFS_H */
