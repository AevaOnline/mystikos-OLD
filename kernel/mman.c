// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

/*
**==============================================================================
**
** OVERVIEW:
** =========
**
** This file implements the following operations over a flat memory space,
** called a heap.
**
**     BRK   - changes the 'break value' of the memory region
**     SBRK  - reserves a chunk of memory
**     MMAP  - reserves an area of memory
**     MREMAP - expands or shrinks a memory area obtained with MAP
**     MUNMAP - releases a memory area obtained with MAP
**
** The memory space has the following layout.
**
**     <--BITMAP--><---BREAK---><--UNASSIGNED--><---------MAPPED---------->
**     [..................................................................]
**     ^           ^            ^               ^                         ^
**    BASE       START         BRK             MAP                       END
**
** The memory space is partitioned into four sections:
**
**     BITMAP     - bitmap of used nodes
**     BREAK      - Managed by the BRK and SBRK: [START, BRK)
**     UNASSIGNED - Unassigned memory: [BRK, MAP)
**     MAPPED     - Manged by the MAP, REMAP, and UNMAP: [MAP, END)
**
** The following diagram depicts the values of BASE, START, BRK, MAP, and
** END for a freshly initialized memory space.
**
**     <--BITMAP--><---------------UNASSIGNED----------------------------->
**     [..................................................................]
**     ^           ^                                                      ^
**    BASE       START                                                   END
**                 ^                                                      ^
**                BRK                                                    MAP
**
** The BREAK section expands by increasing the BRK value. The MAPPED section
** expands by decreasing the MAP value. The BRK and MAP value grow towards
** one another until all unassigned memory is exhausted.
**
**==============================================================================
*/

#include <stdio.h>
#include <string.h>

#include <errno.h>
#include <myst/defs.h>
#include <myst/fsgs.h>
#include <myst/mman.h>
#include <myst/round.h>
#include <myst/spinlock.h>
#include <myst/strings.h>

/*
**==============================================================================
**
** mman helper functions
**
**==============================================================================
*/

/* Lock the mman and set the 'locked' parameter to true */
MYST_INLINE void _mman_lock(myst_mman_t* mman, bool* locked)
{
    myst_spin_lock(&mman->lock);
    *locked = true;
}

/* Unlock the mman and set the 'locked' parameter to false */
MYST_INLINE void _mman_unlock(myst_mman_t* mman, bool* locked)
{
    if (*locked)
    {
        myst_spin_unlock(&mman->lock);
        *locked = false;
    }
}

/* Clear the mman error message */
static void _mman_clear_err(myst_mman_t* mman)
{
    if (mman)
        mman->err[0] = '\0';
}

/* Set the mman error message */
static void _mman_set_err(myst_mman_t* mman, const char* str)
{
    if (mman && str)
        myst_strlcpy(mman->err, str, sizeof(mman->err));
}

/* Inline Helper function to check mman sanity (if enable) */
static bool _mman_is_sane(myst_mman_t* mman)
{
    if (mman->sanity)
        return myst_mman_is_sane(mman);

    return true;
}

MYST_INLINE bool _test_bit(const uint8_t* data, uint32_t index)
{
    uint32_t byte = index / 8;
    uint32_t bit = index % 8;
    return ((uint32_t)(data[byte]) & (1 << bit)) ? 1 : 0;
}

MYST_INLINE void _set_bit(uint8_t* data, uint32_t index)
{
    uint32_t byte = index / 8;
    uint32_t bit = index % 8;
    data[byte] |= (1 << bit);
}

MYST_INLINE void _clear_bit(uint8_t* data, uint32_t index)
{
    uint32_t byte = index / 8;
    uint32_t bit = index % 8;
    data[byte] &= ~(1 << bit);
}

/* find a run of zero bits that is npages or longer */
static ssize_t _find_free_bits(myst_mman_t* mman, size_t nbits)
{
    ssize_t ret = -1;
    size_t n = 0;
    size_t m = mman->bitmap_size * 8;

    for (size_t i = 0; i < m; i++)
    {
        /* optimization: skip full bytes */
        if (mman->bitmap[i / 8] == 0xff)
        {
            i += 8;
            n = 0;
            continue;
        }

        if (_test_bit(mman->bitmap, i))
            n = 0;
        else
            n++;

        if (n == nbits)
        {
            ret = i - n;
            break;
        }
    }

    return ret;
}

static uintptr_t _index_to_ptr(myst_mman_t* mman, size_t index)
{
    uintptr_t ptr = mman->end - (index * PAGE_SIZE) - PAGE_SIZE;

    if (ptr < mman->brk)
        return 0;

    return ptr;
}

static ssize_t _ptr_to_index(myst_mman_t* mman, uintptr_t ptr)
{
    if (!(ptr >= mman->map && ptr < mman->end))
        return -1;

    const ptrdiff_t delta = mman->end - ptr;

    return delta / PAGE_SIZE;
}

static int _munmap(myst_mman_t* mman, void* addr, size_t length)
{
    int ret = -1;
    myst_mnode_t* mnode = NULL;

    _mman_clear_err(mman);

    /* Reject invaid parameters */
    if (!mman || mman->magic != MYST_MMAN_MAGIC || !addr || !length)
    {
        _mman_set_err(mman, "bad parameter");
        ret = -EINVAL;
        goto done;
    }

    if (!_mman_is_sane(mman))
    {
        _mman_set_err(mman, "bad mman parameter");
        ret = -EINVAL;
        goto done;
    }

    /* ADDRESS must be aligned on a page boundary */
    if ((uintptr_t)addr % PAGE_SIZE)
    {
        _mman_set_err(mman, "bad addr parameter");
        ret = -EINVAL;
        goto done;
    }

    /* Align LENGTH to a multiple of the page size */
    if (length % PAGE_SIZE)
    {
        if (myst_round_up(length, PAGE_SIZE, &length) != 0)
        {
            _mman_set_err(mman, "rounding error: length");
            ret = -EINVAL;
            goto done;
        }
    }

    /* Set start and end pointers for this area */
    uintptr_t start = (uintptr_t)addr;
    uintptr_t end = (uintptr_t)addr + length;
    size_t nbits = (end - start) / PAGE_SIZE;
    size_t index;

    /* convert the pointer to a page index */
    if ((index = _ptr_to_index(mman, start)) < 0)
    {
        _mman_set_err(mman, "munmap: bad addr parameter");
        ret = -EINVAL;
        goto done;
    }

    /* verify that all bits in this range are set */
    for (size_t i = index; i < nbits; i++)
    {
        if (!_test_bit(mman->bitmap, i))
        {
            _mman_set_err(mman, "munmap: invalid address range");
            ret = -EINVAL;
            goto done;
        }
    }

    /* If the unapping does not cover the entire area given by the VAD, handle
     * the excess portions. There are 4 cases below, where u's represent
     * the portion being unmapped.
     *
     *     Case1: [uuuuuuuuuuuuuuuu]
     *     Case2: [uuuu............]
     *     Case3: [............uuuu]
     *     Case4: [....uuuu........]
     */
    if (vad->addr == start && _end(vad) == end)
    {
        /* Case1: [uuuuuuuuuuuuuuuu] */

        _list_remove(mman, vad);
        _mman_sync_top(mman);
        _free_list_put(mman, vad);
    }
    else if (vad->addr == start)
    {
        /* Case2: [uuuu............] */

        vad->addr += length;
        vad->size -= (uint32_t)length;
        _mman_sync_top(mman);
    }
    else if (_end(vad) == end)
    {
        /* Case3: [............uuuu] */

        vad->size -= (uint32_t)length;
    }
    else
    {
        /* Case4: [....uuuu........] */

        size_t vad_end = _end(vad);

        /* Adjust the left portion */
        vad->size = (uint32_t)(start - vad->addr);

        myst_vad_t* right;

        /* Create VAD for the excess right portion */
        if (!(right = _mman_new_vad(
                  mman, end, vad_end - end, vad->__prot, vad->__flags)))
        {
            _mman_set_err(mman, "out of VADs");
            ret = -EINVAL;
            goto done;
        }

        _list_insert_after(mman, vad, right);
        _mman_sync_top(mman);
    }

    /* If scrubbing is enabled, then scrub the unmapped memory */
    if (mman->scrub)
        memset(addr, 0xDD, length);

    if (!_mman_is_sane(mman))
    {
        ret = -EINVAL;
        goto done;
    }

    ret = 0;

done:
    return ret;
}

static int _mmap(
    myst_mman_t* mman,
    void* addr,
    size_t length,
    int prot,
    int flags,
    void** ptr_out)
{
    int ret = 0;
    uintptr_t start = 0;

    if (ptr_out)
        *ptr_out = NULL;

    _mman_clear_err(mman);

    /* Check for valid mman parameter */
    if (!mman || mman->magic != MYST_MMAN_MAGIC || !ptr_out)
    {
        _mman_set_err(mman, "bad mman parameter");
        ret = -EINVAL;
        goto done;
    }

    if (!_mman_is_sane(mman))
        goto done;

    /* ADDR must be page aligned */
    if (addr && (uintptr_t)addr % PAGE_SIZE)
    {
        _mman_set_err(mman, "bad addr parameter");
        ret = -EINVAL;
        goto done;
    }

    /* LENGTH must be non-zero */
    if (length == 0)
    {
        _mman_set_err(mman, "bad length parameter");
        ret = -EINVAL;
        goto done;
    }

    (void)prot;

    /* Ignore protection for SGX-1 */
#if 0
    {
        if (!(prot & MYST_PROT_READ))
        {
            _mman_set_err(mman, "bad prot parameter: need MYST_PROT_READ");
            ret = -EINVAL;
            goto done;
        }

        if (!(prot & MYST_PROT_WRITE))
        {
            _mman_set_err(mman, "bad prot parameter: need MYST_PROT_WRITE");
            ret = -EINVAL;
            goto done;
        }

        if (prot & MYST_PROT_EXEC)
        {
            _mman_set_err(mman, "bad prot parameter: remove MYST_PROT_EXEC");
            ret = -EINVAL;
            goto done;
        }
    }
#endif

    /* FLAGS must be (MYST_MAP_ANONYMOUS | MYST_MAP_PRIVATE) */
    {
        if (!(flags & MYST_MAP_ANONYMOUS))
        {
            _mman_set_err(mman, "bad flags parameter: need MYST_MAP_ANONYMOUS");
            ret = -EINVAL;
            goto done;
        }

        if (!(flags & MYST_MAP_PRIVATE))
        {
            _mman_set_err(mman, "bad flags parameter: need MYST_MAP_PRIVATE");
            ret = -EINVAL;
            goto done;
        }

        if (flags & MYST_MAP_SHARED)
        {
            _mman_set_err(mman, "bad flags parameter: remove MYST_MAP_SHARED");
            ret = -EINVAL;
            goto done;
        }

        if (flags & MYST_MAP_FIXED)
        {
            _mman_set_err(mman, "bad flags parameter: remove MYST_MAP_FIXED");
            ret = -EINVAL;
            goto done;
        }
    }

    /* Round LENGTH to multiple of page size */
    if (myst_round_up(length, PAGE_SIZE, &length) != 0)
    {
        _mman_set_err(mman, "rounding error: length");
        ret = -EINVAL;
        goto done;
    }

    if (addr)
    {
        uintptr_t start = (uintptr_t)addr;
        uintptr_t end = (uintptr_t)addr + length;
        size_t nbits = (end - start) / PAGE_SIZE;
        size_t index;

        if ((index = _ptr_to_index(mman, start)) < 0)
        {
            _mman_set_err(mman, "bad mmap addr parameter");
            ret = -EINVAL;
            goto done;
        }

        /* verify that all bits in this range are set */
        for (size_t i = index; i < nbits; i++)
        {
            if (!_test_bit(mman->bitmap, i))
            {
                _mman_set_err(mman, "bad mmap addr parameter");
                ret = -EINVAL;
                goto done;
            }
        }

        *ptr_out = addr;
        goto done;
    }
    else
    {
        /* length is a multiple of the page size */
        const size_t npages = length / PAGE_SIZE;
        ssize_t index;

        /* find a run of zero bits that is long enough */
        if ((index = _find_free_bits(mman, npages)) < 0)
        {
            _mman_set_err(mman, "out of memory");
            ret = -ENOMEM;
            goto done;
        }

        /* Get the pointer from the index */
        if ((start = _index_to_ptr(mman, index)) == 0)
        {
            _mman_set_err(mman, "out of memory");
            ret = -ENOMEM;
            goto done;
        }

        /* Adjust mman->mmap */
        if (start < mman->map)
            mman->map = start;

        /* reserve the bits */
        for (size_t i = index; i < npages; i++)
            _set_bit(mman->bitmap, i);
    }

    *ptr_out = (void*)start;

done:

    /* Zero-fill mapped memory */
    if (ptr_out && *ptr_out)
        memset(*ptr_out, 0, length);

    return ret;
}

#if 0

/*
**==============================================================================
**
** Public interface
**
**==============================================================================
*/

/*
**
** myst_mman_init()
**
**     Initialize a mman structure by setting the 'base' and 'size' and other
**     internal state variables. Note that the caller must obtain a lock if
**     one is needed.
**
** Parameters:
**     [IN] mman - mman structure to be initialized.
**     [IN] base - base address of the heap (must be must be page aligned).
**     [IN] size - size of the heap in bytes (must be multiple of page size).
**
** Returns:
**     0 if successful.
**
*/
int myst_mman_init(myst_mman_t* mman, uintptr_t base, size_t size)
{
    int ret = 0;

    _mman_clear_err(mman);

    /* Check for invalid parameters */
    if (!mman || !base || !size)
    {
        _mman_set_err(mman, "bad parameter");
        ret = -EINVAL;
        goto done;
    }

    /* BASE must be aligned on a page boundary */
    if (base % PAGE_SIZE)
    {
        _mman_set_err(mman, "bad base parameter");
        ret = -EINVAL;
        goto done;
    }

    /* SIZE must be a mulitple of the page size */
    if (size % PAGE_SIZE)
    {
        _mman_set_err(mman, "bad size parameter");
        ret = -EINVAL;
        goto done;
    }

    /* Clear the heap object */
    memset(mman, 0, sizeof(myst_mman_t));

    /* Calculate the total number of pages */
    size_t num_pages = size / PAGE_SIZE;

    /* Save the base of the heap */
    mman->base = base;

    /* Save the size of the heap */
    mman->size = size;

    /* Set the start of the heap area, which follows the VADs array */
    mman->start = base + (num_pages * sizeof(myst_vad_t));

    /* Round start up to next page multiple */
    if (myst_round_up(mman->start, PAGE_SIZE, &mman->start) != 0)
    {
        _mman_set_err(mman, "rounding error: mman->start");
        ret = -EINVAL;
        goto done;
    }

    /* Set the end of the heap area */
    mman->end = base + size;

    /* Set the top of the break memory (grows positively) */
    mman->brk = mman->start;

    /* Set the top of the mapped memory (grows negativey) */
    mman->map = mman->end;

    /* Set pointer to the next available entry in the myst_vad_t array */
    mman->next_vad = (myst_vad_t*)base;

    /* Set pointer to the end address of the myst_vad_t array */
    mman->end_vad = (myst_vad_t*)mman->start;

    /* Set the free myst_vad_t list to null */
    mman->free_vads = NULL;

    /* Set the myst_vad_t linked list to null */
    mman->vad_list = NULL;

    /* Sanity checks are disabled by default */
    mman->sanity = false;

    /* Set the magic number */
    mman->magic = MYST_MMAN_MAGIC;

    /* Finally, set initialized to true */
    mman->initialized = 1;

    /* Check sanity of mman */
    if (!_mman_is_sane(mman))
    {
        ret = -EINVAL;
        goto done;
    }

    ret = 0;

done:
    return ret;
}

/*
**
** myst_mman_sbrk()
**
**     Allocate space from the BREAK region (between the START and BRK value)
**     This increases the BRK value by at least the increment size (rounding
**     up to multiple of 8).
**
** Parameters:
**     [IN] mman - mman structure
**     [IN] increment - allocate this must space.
**
** Returns:
**     Pointer to allocate memory or NULL if BREAK memory has been exhausted.
**
** Notes:
**     This function is similar to the POSIX sbrk() function.
**
*/
int myst_mman_sbrk(myst_mman_t* mman, ptrdiff_t increment, void** ptr_out)
{
    int ret = 0;
    void* ptr = NULL;
    bool locked = false;

    if (ptr_out)
        *ptr_out = NULL;

    _mman_lock(mman, &locked);

    _mman_clear_err(mman);

    if (!_mman_is_sane(mman) || !ptr_out)
    {
        ret = -EINVAL;
        goto done;
    }

    if (increment == 0)
    {
        /* Return the current break value without changing it */
        ptr = (void*)mman->brk;
    }
    else if ((uintptr_t)increment <= mman->map - mman->brk)
    {
        /* Increment the break value and return the old break value */
        ptr = (void*)mman->brk;
        mman->brk += (uintptr_t)increment;
    }
    else
    {
        _mman_set_err(mman, "out of memory");
        ret = -ENOMEM;
        goto done;
    }

    if (!_mman_is_sane(mman))
        goto done;

    *ptr_out = ptr;

done:
    _mman_unlock(mman, &locked);
    return ret;
}

/*
**
** myst_mman_brk()
**
**     Change the BREAK value (within the BREAK region). Increasing the
**     break value has the effect of allocating memory. Decresing the
**     break value has the effect of releasing memory.
**
** Parameters:
**     [IN] mman - mman structure
**     [IN] addr - set the BREAK value to this address (must reside within
**     the break region (beween START and BREAK value).
**
** Returns:
**     0 if successful.
**
** Notes:
**     This function is similar to the POSIX brk() function.
**
*/
int myst_mman_brk(myst_mman_t* mman, void* addr, void** ptr)
{
    int ret = 0;
    bool locked = false;

    if (*ptr)
        *ptr = NULL;

    _mman_clear_err(mman);

    if (!mman || !ptr)
    {
        ret = -EINVAL;
        goto done;
    }

    _mman_lock(mman, &locked);

    if (addr == NULL)
        goto done;

    /* Fail if requested address is not within the break memory area */
    if ((uintptr_t)addr < mman->start || (uintptr_t)addr >= mman->map)
    {
        _mman_set_err(mman, "address is out of range");
        ret = -ENOMEM;
        goto done;
    }

    /* Set the break value */
    mman->brk = (uintptr_t)addr;

    if (!_mman_is_sane(mman))
    {
        _mman_set_err(mman, "bad mman parameter");
        ret = -ENOMEM;
        goto done;
    }

done:

    /* Always return the break value (even on error) */
    if (mman)
        *ptr = (void*)mman->brk;

    _mman_unlock(mman, &locked);
    return ret;
}

/*
**
** myst_mman_mmap()
**
**     Allocate 'length' bytes from the MAPPED region. The 'length' parameter
**     is rounded to a multiple of the page size.
**
** Parameters:
**     [IN] mman - mman structure
**     [IN] addr - must be null in this implementation
**     [IN] length - length in bytes of the new allocation
**     [IN] prot - must be (MYST_PROT_READ | MYST_PROT_WRITE)
**     [IN] flags - must be (MYST_MAP_ANONYMOUS | MYST_MAP_PRIVATE)
**
** Returns:
**     Pointer to newly mapped memory if successful.
**
** Notes:
**     This function is similar to the POSIX mmap() function.
**
** Implementation:
**     This function searches for a gap such that gap >= length. If found,
**     it initializes a new VAD structure and inserts it into the active
**     VAD list.
**
*/
int myst_mman_mmap(
    myst_mman_t* mman,
    void* addr,
    size_t length,
    int prot,
    int flags,
    void** ptr_out)
{
    bool locked = false;

    _mman_lock(mman, &locked);
    int ret = _mmap(mman, addr, length, prot, flags, ptr_out);
    _mman_unlock(mman, &locked);

    return ret;
}

/*
**
** myst_mman_munmap()
**
**     Release a memory mapping obtained with myst_mman_mmap() or
**     myst_mman_mremap().
**
**     Note that partial mappings are supported, in which case a portion of
**     the memory obtained with myst_mman_mmap() or myst_mman_mremap() is
**     released.
**
** Parameters:
**     [IN] mman - mman structure
**     [IN] addr - addresss or memory being released (must be page aligned).
**     [IN] length - length of memory being released (multiple of page size).
**
** Returns:
**     MYST_OK if successful.
**
** Notes:
**     This function is similar to the POSIX munmap() function.
**
** Implementation:
**     This function searches the active VAD list for a VAD that contains
**     the range given by 'addr' and 'length'. If the VAD region is larger
**     than the range being freed, then it is split into smaller VADs. The
**     leftward excess (if any) is split into its own VAD and the rightward
**     excess (if any) is split into its own VAD.
**
*/
int myst_mman_munmap(myst_mman_t* mman, void* addr, size_t length)
{
    bool locked = false;

    _mman_lock(mman, &locked);
    int ret = _munmap(mman, addr, length);
    _mman_unlock(mman, &locked);

    return ret;
}

/*
**
** myst_mman_mremap()
**
**     Remap an existing memory region, either making it bigger or smaller.
**
** Parameters:
**     [IN] mman - mman structure
**     [IN] addr - addresss being remapped (must be multiple of page size)
**     [IN] old_size - original size of the memory mapping
**     [IN] new_size - new size of memory mapping (rounded up to page multiple)
**     [IN] flags - must be MYST_MREMAP_MAYMOVE
**
** Returns:
**     Pointer to new memory region.
**
** Notes:
**     This function is similar to the POSIX mremap() function.
**
** Implementation:
**     This function attempts to keep the memory in place if possible. If
**     not, it moves it to a new location.
**
*/
int myst_mman_mremap(
    myst_mman_t* mman,
    void* addr,
    size_t old_size,
    size_t new_size,
    int flags,
    void** ptr_out)
{
    int ret = 0;
    void* new_addr = NULL;
    myst_vad_t* vad = NULL;
    bool locked = false;

    if (ptr_out)
        *ptr_out = NULL;

    _mman_lock(mman, &locked);

    _mman_clear_err(mman);

    /* Check for valid mman parameter */
    if (!mman || mman->magic != MYST_MMAN_MAGIC || !addr || !ptr_out)
    {
        _mman_set_err(mman, "invalid parameter");
        ret = -EINVAL;
        goto done;
    }

    if (!_mman_is_sane(mman))
        goto done;

    /* ADDR must be page aligned */
    if ((uintptr_t)addr % PAGE_SIZE)
    {
        _mman_set_err(
            mman, "bad addr parameter: must be multiple of page size");
        ret = -EINVAL;
        goto done;
    }

    /* OLD_SIZE must be non-zero */
    if (old_size == 0)
    {
        _mman_set_err(mman, "invalid old_size parameter: must be non-zero");
        ret = -EINVAL;
        goto done;
    }

    /* NEW_SIZE must be non-zero */
    if (new_size == 0)
    {
        _mman_set_err(mman, "invalid old_size parameter: must be non-zero");
        ret = -EINVAL;
        goto done;
    }

    /* FLAGS must be exactly MYST_MREMAP_MAYMOVE) */
    if (flags != MYST_MREMAP_MAYMOVE)
    {
        _mman_set_err(
            mman, "invalid flags parameter: must be MYST_MREMAP_MAYMOVE");
        ret = -EINVAL;
        goto done;
    }

    /* Round OLD_SIZE to multiple of page size */
    if (myst_round_up(old_size, PAGE_SIZE, &old_size) != 0)
    {
        _mman_set_err(mman, "rounding error: old_size");
        ret = -EINVAL;
        goto done;
    }

    /* Round NEW_SIZE to multiple of page size */
    if (myst_round_up(new_size, PAGE_SIZE, &new_size) != 0)
    {
        _mman_set_err(mman, "rounding error: new_size");
        ret = -EINVAL;
        goto done;
    }

    /* Set start and end pointers for this area */
    uintptr_t start = (uintptr_t)addr;
    uintptr_t old_end = (uintptr_t)addr + old_size;
    uintptr_t new_end = (uintptr_t)addr + new_size;

    /* Find the VAD containing START */
    if (!(vad = _list_find(mman, start)))
    {
        _mman_set_err(mman, "invalid addr parameter: mapping not found");
        ret = -ENOMEM;
        goto done;
    }

    /* Verify that the end address is within this VAD */
    if (old_end > _end(vad))
    {
        _mman_set_err(mman, "invalid range");
        ret = -ENOMEM;
        goto done;
    }

    /* If the area is shrinking */
    if (new_size < old_size)
    {
        /* If there are excess bytes on the right of this VAD area */
        if (_end(vad) != old_end)
        {
            myst_vad_t* right;

            /* Create VAD for rightward excess */
            if (!(right = _mman_new_vad(
                      mman,
                      old_end,
                      _end(vad) - old_end,
                      vad->__prot,
                      vad->__flags)))
            {
                _mman_set_err(mman, "out of VADs");
                ret = -ENOMEM;
                goto done;
            }

            _list_insert_after(mman, vad, right);
            _mman_sync_top(mman);
        }

        vad->size = (uint32_t)(new_end - vad->addr);
        new_addr = addr;

        /* If scrubbing is enabled, scrub the unmapped portion */
        if (mman->scrub)
            memset((void*)new_end, 0xDD, old_size - new_size);
    }
    else if (new_size > old_size)
    {
        /* Calculate difference between new and old size */
        size_t delta = new_size - old_size;

        /* If there is room for this area to grow without moving it */
        if (_end(vad) == old_end && _get_right_gap(mman, vad) >= delta)
        {
            vad->size += (uint32_t)delta;
            memset((void*)(start + old_size), 0, delta);
            new_addr = addr;

            /* If VAD is now contiguous with next one, coalesce them */
            if (vad->next && _end(vad) == vad->next->addr)
            {
                myst_vad_t* next = vad->next;
                vad->size += next->size;
                _list_remove(mman, next);
                _mman_sync_top(mman);
                _free_list_put(mman, next);
            }
        }
        else
        {
            if (_mmap(mman, NULL, new_size, vad->__prot, vad->__flags, &addr) != 0)
            {
                _mman_set_err(mman, "mapping failed");
                ret = -ENOMEM;
            }

            /* Copy over data from old area */
            memcpy(addr, (void*)start, old_size);

            /* Ummap the old area */
            if (_munmap(mman, (void*)start, old_size) != 0)
            {
                _mman_set_err(mman, "unmapping failed");
                ret = -ENOMEM;
                goto done;
            }

            new_addr = (void*)addr;
        }
    }
    else
    {
        /* Nothing to do since size did not change */
        new_addr = addr;
    }

    if (!_mman_is_sane(mman))
        goto done;

    *ptr_out = new_addr;

done:
    _mman_unlock(mman, &locked);
    return ret;
}

/*
**
** myst_mman_is_sane()
**
**     Debugging function used to check sanity (validity) of a mman structure.
**
** Parameters:
**     [IN] mman - mman structure
**
** Returns:
**     true if mman is sane
**
** Implementation:
**     Checks various contraints such as ranges being correct and VAD list
**     being sorted.
**
*/
bool myst_mman_is_sane(myst_mman_t* mman)
{
    bool result = false;

    _mman_clear_err(mman);

    if (!mman)
    {
        _mman_set_err(mman, "invalid parameter");
        goto done;
    }

    _mman_clear_err(mman);

    /* Check the magic number */
    if (mman->magic != MYST_MMAN_MAGIC)
    {
        _mman_set_err(mman, "bad magic");
        goto done;
    }

    /* Check that the mman is initialized */
    if (!mman->initialized)
    {
        _mman_set_err(mman, "uninitialized");
        goto done;
    }

    /* Check that the start of the mman is strictly less than the end */
    if (!(mman->start < mman->end))
    {
        _mman_set_err(mman, "start not less than end");
        goto done;
    }

    if (mman->size != (mman->end - mman->base))
    {
        _mman_set_err(mman, "invalid size");
        goto done;
    }

    if (!(mman->start <= mman->brk))
    {
        _mman_set_err(mman, "!(mman->start <= mman->brk)");
        goto done;
    }

    if (!(mman->map <= mman->end))
    {
        _mman_set_err(mman, "!(mman->map <= mman->end)");
        goto done;
    }

    if (mman->vad_list)
    {
        if (mman->map != mman->vad_list->addr)
        {
            _mman_set_err(mman, "mman->map != mman->vad_list->addr");
            goto done;
        }
    }
    else
    {
        if (mman->map != mman->end)
        {
            _mman_set_err(mman, "mman->map != mman->end");
            goto done;
        }
    }

    /* Verify that the list is sorted */
    {
        myst_vad_t* p;

        for (p = mman->vad_list; p; p = p->next)
        {
            myst_vad_t* next = p->next;

            if (next)
            {
                if (!(p->addr < next->addr))
                {
                    _mman_set_err(mman, "unordered VAD list (1)");
                    goto done;
                }

                /* No two elements should be contiguous due to coalescense */
                if (_end(p) == next->addr)
                {
                    _mman_set_err(mman, "contiguous VAD list elements");
                    goto done;
                }

                if (!(_end(p) <= next->addr))
                {
                    _mman_set_err(mman, "unordered VAD list (2)");
                    goto done;
                }
            }
        }
    }

    result = true;

done:
    return result;
}

/*
**
** myst_mman_set_sanity()
**
**     Enable live sanity checking on the given mman structure. Once enabled,
**     sanity checking is performed in all mapping functions. Be aware that
**     this slows down the implementation and should be used for debugging
**     and testing only.
**
** Parameters:
**     [IN] mman - mman structure
**     [IN] sanity - true to enable sanity checking; false otherwise.
**
*/
void myst_mman_set_sanity(myst_mman_t* mman, bool sanity)
{
    if (mman)
        mman->sanity = sanity;
}

/* return the total size of the mman region */
int myst_mman_total_size(myst_mman_t* mman, size_t* size)
{
    ssize_t ret = 0;

    if (*size)
        *size = 0;

    if (!mman || !size)
    {
        ret = -EINVAL;
        goto done;
    }

    myst_spin_lock(&mman->lock);
    *size = mman->size;
    myst_spin_unlock(&mman->lock);

done:
    return ret;
}

/* return the amount of free space */
int myst_mman_free_size(myst_mman_t* mman, size_t* size_out)
{
    ssize_t ret = 0;
    size_t size;

    if (*size_out)
        *size_out = 0;

    if (!mman || !size_out)
    {
        ret = -EINVAL;
        goto done;
    }

    myst_spin_lock(&mman->lock);
    {
        /* determine the bytes between the BRK value and MAP value */
        size = mman->map - mman->brk;

        /* determine the total size of all gaps */
        for (myst_vad_t* p = mman->vad_list; p; p = p->next)
            size += _get_right_gap(mman, p);
    }
    myst_spin_unlock(&mman->lock);

    *size_out = size;

done:
    return ret;
}

void myst_mman_dump_vads(myst_mman_t* mman)
{
    if (!mman)
        return;

    printf("=== myst_mman_dump_vads()\n");

    myst_spin_lock(&mman->lock);
    {
        /* determine the total size of all gaps */
        for (myst_vad_t* p = mman->vad_list; p; p = p->next)
        {
            uint64_t start = p->addr;
            uint64_t end = p->addr + p->size;

            printf("VAD(range[%lx:%lx] size=%lu)\n", start, end, end - start);
        }
    }
    myst_spin_unlock(&mman->lock);
}

#endif
