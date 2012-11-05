/*
 * heap.h: Intel(r) TXT heap definitions
 *
 * Copyright (c) 2003-2007, Intel Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 *  heap.h: Modified for use with Flicker
 */

#ifndef __TXT_HEAP_H__
#define __TXT_HEAP_H__

#include "mtrrs.h"

/*
 * BIOS structure
 */
typedef struct {
    uint32_t  version;              /* WB = 2, current = 3 */
    uint32_t  bios_sinit_size;
    uint64_t  lcp_pd_base;
    uint64_t  lcp_pd_size;
    uint32_t  num_logical_procs;
    uint64_t  flags;                /* v3+ */
} bios_data_t;

/*
 * OS/loader to MLE structure
 *   - private to tboot (so can be any format we need)
 */

typedef struct {
    mtrr_state_t      saved_mtrr_state;       /* saved prior to
                                               * changes for SINIT */
    uint32_t          saved_misc_enable_msr;  /* saved prior to
                                                    * SENTER */
} os_mle_data_t;

/*
 * TXT heap data format and field accessor fns. See MLE developer's
 * guide for details.
 */
typedef void txt_heap_t;

/* this is a common use with annoying casting, so make it an inline */
static inline txt_heap_t *get_txt_heap(void)
{
    return (txt_heap_t *)(unsigned long)read_pub_config_reg(TXTCR_HEAP_BASE);
}


static inline uint64_t get_bios_data_size(txt_heap_t *heap)
{
    return *(uint64_t *)heap;
}

static inline bios_data_t *get_bios_data_start(txt_heap_t *heap)
{
    return (bios_data_t *)((char*)heap + sizeof(uint64_t));
}

static inline uint64_t get_os_mle_data_size(txt_heap_t *heap)
{
    return *(uint64_t *)((char*)heap + get_bios_data_size(heap));
}

static inline void *get_os_mle_data_start(txt_heap_t *heap)
{
    return (void *)((char*)heap + get_bios_data_size(heap) +
                              sizeof(uint64_t));
}

#endif /* __TXT_HEAP_H__ */
