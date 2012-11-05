/*
 * mtrrs.h: Intel(r) TXT MTRR-related definitions
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
 *  mtrrs.h: Modified for Flicker
 */

#ifndef __TXT_MTRRS_H__
#define __TXT_MTRRS_H__

#include <stdint.h>
#include <stdbool.h>

#include "util.h"


/* Code from <asm/msr.h> to read MSR values on x86 */

#define DECLARE_ARGS(val, low, high)   unsigned long long val
#define EAX_EDX_VAL(val, low, high)    (val)
#define EAX_EDX_ARGS(val, low, high)   "A" (val)
#define EAX_EDX_RET(val, low, high)    "=A" (val)

static inline unsigned long long native_read_msr(unsigned int msr)
{
       DECLARE_ARGS(val, low, high);
       asm volatile("rdmsr" : EAX_EDX_RET(val, low, high) : "c" (msr));
       return EAX_EDX_VAL(val, low, high);
}

static inline void native_write_msr(unsigned int msr,
                                   unsigned low, unsigned high)
{
       asm volatile("wrmsr" : : "c" (msr), "a"(low), "d" (high) : "memory");
}

#define rdmsrl(msr, val)                       \
       ((val) = native_read_msr((msr)))

#define wrmsrl(msr, val)                                               \
       native_write_msr((msr), (uint32_t)((uint64_t)(val)), (uint32_t)((uint64_t)(val) >> 32))

/* End of code from <asm/msr.h> */


enum var_mtrr_t {
    MTRR_PHYS_BASE0_MSR = 0x200,
    MTRR_PHYS_MASK0_MSR = 0x201,
    MTRR_PHYS_BASE1_MSR = 0x202,
    MTRR_PHYS_MASK1_MSR = 0x203,
    MTRR_PHYS_BASE2_MSR = 0x204,
    MTRR_PHYS_MASK2_MSR = 0x205,
    MTRR_PHYS_BASE3_MSR = 0x206,
    MTRR_PHYS_MASK3_MSR = 0x207,
    MTRR_PHYS_BASE4_MSR = 0x208,
    MTRR_PHYS_MASK4_MSR = 0x209,
    MTRR_PHYS_BASE5_MSR = 0x20A,
    MTRR_PHYS_MASK5_MSR = 0x20B,
    MTRR_PHYS_BASE6_MSR = 0x20C,
    MTRR_PHYS_MASK6_MSR = 0x20D,
    MTRR_PHYS_BASE7_MSR = 0x20E,
    MTRR_PHYS_MASK7_MSR = 0x20F
};

typedef union {
    uint64_t    raw;
    struct {
        uint64_t vcnt        : 8;    /* num variable MTRR pairs */
        uint64_t fix         : 1;    /* fixed range MTRRs are supported */
        uint64_t reserved1   : 1;
        uint64_t wc          : 1;    /* write-combining mem type supported */
        uint64_t reserved2   : 53;
    };
} mtrr_cap_t;

typedef union {
    uint64_t    raw;
    struct {
        uint64_t type        : 8;
        uint64_t reserved1   : 2;
        uint64_t fe          : 1;    /* fixed MTRR enable */
        uint64_t e           : 1;    /* (all) MTRR enable */
        uint64_t reserved2   : 52;
    };
} mtrr_def_type_t;

typedef union {
    uint64_t    raw;
    struct {
        uint64_t type      : 8;
        uint64_t reserved1 : 4;
        uint64_t base      : 24;
        uint64_t reserved2 : 28;
    };
} mtrr_physbase_t;

typedef union {
    uint64_t    raw;
    struct {
        uint64_t reserved1 : 11;
        uint64_t v         : 1;      /* valid */
        uint64_t mask      : 24;
        uint64_t reserved2 : 28;
    };
} mtrr_physmask_t;

/* current procs only have 8, so this should hold us for a while */
#define MAX_VARIABLE_MTRRS      16

typedef struct {
    mtrr_def_type_t        mtrr_def_type;
    int                    num_var_mtrrs;
    mtrr_physbase_t     mtrr_physbases[MAX_VARIABLE_MTRRS];
    mtrr_physmask_t     mtrr_physmasks[MAX_VARIABLE_MTRRS];
} mtrr_state_t;


extern void set_all_mtrrs(bool enable);

extern void restore_mtrrs(mtrr_state_t *saved_state);


#endif /*__TXT_MTRRS_H__ */

