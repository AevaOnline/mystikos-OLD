// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#ifndef _MYST_INTERNAL_MMAN_H
#define _MYST_INTERNAL_MMAN_H

#include <myst/spinlock.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MYST_PROT_NONE 0
#define MYST_PROT_READ 1
#define MYST_PROT_WRITE 2
#define MYST_PROT_EXEC 4

#define MYST_MAP_SHARED 1
#define MYST_MAP_PRIVATE 2
#define MYST_MAP_FIXED 16
#define MYST_MAP_ANONYMOUS 32

#define MYST_MREMAP_MAYMOVE 1

#define MYST_MMAN_ERROR_SIZE 256

#define MYST_MMAN_MAGIC 0xcc8e1732ebd80b0b

#define MYST_MMAN_ERR_SIZE 256

typedef struct myst_mnode myst_mnode_t;

/* myst_mman_t data structures and fields */
typedef struct myst_mman
{
    /* Magic number (MYST_MMAN_MAGIC) */
    uint64_t magic;

    /* True if myst_mman_init() has been called */
    bool initialized;

    /* Base of heap (aligned on page boundary) */
    uintptr_t base;

    /* Size of heap (a multiple of MYST_PAGE_SIZE) */
    size_t size;

    /* Start of heap (immediately after VADs array) */
    uintptr_t start;

    /* End of heap (points to first page after end of heap) */
    uintptr_t end;

    /* Current break value: top of break memory partition (grows positively) */
    uintptr_t brk;

    /* Current map value: top of mapped memory partition (grows negatively) */
    uintptr_t map;

    /* the page bitmap */
    uint8_t* bitmap;
    size_t bitmap_size; /* bitmap size in bytes */

    /* Whether sanity checks are enabled: see MYST_HeapEnableSanityChecks() */
    bool sanity;

    /* Whether to scrub memory when it is unmapped (fill with 0xDD) */
    bool scrub;

    /* Heap locking */
    myst_spinlock_t lock;

    /* Error string */
    char err[MYST_MMAN_ERROR_SIZE];

} myst_mman_t;

int myst_mman_init(myst_mman_t* heap, uintptr_t base, size_t size);

int myst_mman_mmap(
    myst_mman_t* heap,
    void* addr,
    size_t length,
    int prot,
    int flags,
    void** ptr);

int myst_mman_mremap(
    myst_mman_t* heap,
    void* addr,
    size_t old_size,
    size_t new_size,
    int flags,
    void** ptr);

int myst_mman_munmap(myst_mman_t* heap, void* address, size_t size);

void myst_mman_dump(const myst_mman_t* h, bool full);

int myst_mman_sbrk(myst_mman_t* heap, ptrdiff_t increment, void** ptr);

int myst_mman_brk(myst_mman_t* mman, void* addr, void** ptr);

void myst_mman_set_sanity(myst_mman_t* heap, bool sanity);

bool myst_mman_is_sane(myst_mman_t* heap);

int myst_mman_total_size(myst_mman_t* mman, size_t* size);

int myst_mman_free_size(myst_mman_t* mman, size_t* size);

void myst_mman_dump_vads(myst_mman_t* mman);

#endif /* _MYST_INTERNAL_MMAN_H */
