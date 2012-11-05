/*
 * mtrrs.c: support functions for manipulating MTRRs
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
 *  mtrrs.c: Modified for Flicker
 */

#ifndef _WIN32
#include <asm/msr.h> /* for MSR_MTRRcap */
#else  // _WIN32
#include "wintypes.h"
#include "msr.h"
#endif // _WIN32


#include "mtrrs.h"


void restore_mtrrs(mtrr_state_t *saved_state)
{
    int ndx;
    int num_var_mtrrs;

    /* disable all MTRRs first */
    set_all_mtrrs(false);

    num_var_mtrrs = saved_state->num_var_mtrrs;

    /* physmask's and physbase's */
    for ( ndx = 0; ndx < num_var_mtrrs; ndx++ ) {
        /* Careful, wrmsrl might be interacting with the
           counter of the for loop */
        wrmsrl(MTRR_PHYS_MASK0_MSR + ndx*2,
               saved_state->mtrr_physmasks[ndx].raw);
        wrmsrl(MTRR_PHYS_BASE0_MSR + ndx*2,
               saved_state->mtrr_physbases[ndx].raw);
    }

    /* IA32_MTRR_DEF_TYPE MSR */
    wrmsrl(MSR_MTRRdefType, saved_state->mtrr_def_type.raw);

}



/* enable/disable all MTRRs */
void set_all_mtrrs(bool enable)
{
    mtrr_def_type_t mtrr_def_type;

    rdmsrl(MSR_MTRRdefType, mtrr_def_type.raw);
    mtrr_def_type.e = enable ? 1 : 0;
    wrmsrl(MSR_MTRRdefType, mtrr_def_type.raw);
}

