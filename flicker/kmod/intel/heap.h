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

#ifndef _WIN32
#include <linux/module.h>
#else // _WiN32
#include "wintypes.h"
#endif // _WiN32

#include "mtrrs.h"

/* Max. and Min. size of the TXT heap in bytes. MAX bytes will be
 * mapped in the virtual address space if reading TXTCR_HEAP_SIZE
 * fails or returns a ridiculous value (e.g., 0x0, or
 * 0xffffffffffffffff). */
#define MAX_TXT_HEAP_SIZE     (0x100000)
#define MIN_TXT_HEAP_SIZE          (0x8)


/*
 * data-passing structures contained in TXT heap:
 *   - BIOS to OS/loader
 *   - OS/loader to MLE
 *   - OS/loader to SINIT
 *   - SINIT to MLE
 */

#ifdef _WIN32
#pragma pack(push, 1)
#endif // _WIN32

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
 * OS/loader to SINIT structure
 */
typedef struct {
    uint32_t    version;           /* currently 4 */
    uint32_t    reserved;
    uint64_t    mle_ptab;
    uint64_t    mle_size;
    uint64_t    mle_hdr_base;
    uint64_t    vtd_pmr_lo_base;
    uint64_t    vtd_pmr_lo_size;
    uint64_t    vtd_pmr_hi_base;
    uint64_t    vtd_pmr_hi_size;
    uint64_t    lcp_po_base;
    uint64_t    lcp_po_size;
    txt_caps_t  capabilities;
} os_sinit_data_t;

/*
 * SINIT to MLE structure
 */
#define MDR_MEMTYPE_GOOD                0x00
#define MDR_MEMTYPE_SMM_OVERLAY         0x01
#define MDR_MEMTYPE_SMM_NONOVERLAY      0x02
#define MDR_MEMTYPE_PCIE_CONFIG_SPACE   0x03
#define MDR_MEMTYPE_PROTECTED           0x04

#ifndef _WIN32
typedef struct __attribute__ ((__packed__)) {
#else // _WIN32
typedef struct {
#endif // _WIN32
    uint64_t  base;
    uint64_t  length;
    uint8_t   mem_type;
    uint8_t   reserved[7];
} sinit_mdr_t;

#define SHA1_SIZE      20
typedef uint8_t   sha1_hash_t[SHA1_SIZE];

typedef struct {
    uint32_t     version;             /* currently 6 */
    sha1_hash_t  bios_acm_id;
    uint32_t     edx_senter_flags;
    uint64_t     mseg_valid;
    sha1_hash_t  sinit_hash;
    sha1_hash_t  mle_hash;
    sha1_hash_t  stm_hash;
    sha1_hash_t  lcp_policy_hash;
    uint32_t     lcp_policy_control;
    uint32_t     rlp_wakeup_addr;
    uint32_t     reserved;
    uint32_t     num_mdrs;
    uint32_t     mdrs_off;
    uint32_t     num_vtd_dmars;
    uint32_t     vtd_dmars_off;
} sinit_mle_data_t;

#ifdef _WIN32
#pragma pack(pop) // Pop the packed attribute
#endif // _WIN32

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

/* making this a function since the register in practice often has a
 * nonsense value. */
static inline uint64_t get_txt_heap_size(void)
{
    uint64_t heap_size;
    heap_size = read_priv_config_reg(TXTCR_HEAP_SIZE);
    if(heap_size <= MIN_TXT_HEAP_SIZE ||
       heap_size > MAX_TXT_HEAP_SIZE) {
        error("WARNING: TXTCR_HEAP_SIZE contains strange value: %016llx; using %08x instead.\n",
               heap_size, MAX_TXT_HEAP_SIZE);
        heap_size = MAX_TXT_HEAP_SIZE;
    }
    return heap_size;
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

static inline uint64_t get_os_sinit_data_size(txt_heap_t *heap)
{
    return *(uint64_t *)((char*)heap + get_bios_data_size(heap) +
                         get_os_mle_data_size(heap));
}

static inline os_sinit_data_t *get_os_sinit_data_start(txt_heap_t *heap)
{
    return (os_sinit_data_t *)((char*)heap + get_bios_data_size(heap) +
                               get_os_mle_data_size(heap) +
                               sizeof(uint64_t));
}

static inline uint64_t get_sinit_mle_data_size(txt_heap_t *heap)
{
    return *(uint64_t *)((char*)heap + get_bios_data_size(heap) +
                         get_os_mle_data_size(heap) +
                         get_os_sinit_data_size(heap));
}

static inline sinit_mle_data_t *get_sinit_mle_data_start(txt_heap_t *heap)
{
    return (sinit_mle_data_t *)((char*)heap + get_bios_data_size(heap) +
                                get_os_mle_data_size(heap) +
                                get_os_sinit_data_size(heap) +
                                sizeof(uint64_t));
}

/* 3 functions: verify_txt_heap, verify_bios_data, print_os_sinit_data
 * declared in heap.h in newer tboot; left in verify.h for Flicker. */

#endif /* __TXT_HEAP_H__ */


/*
 * Local variables:
 * mode: C
 * c-set-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
