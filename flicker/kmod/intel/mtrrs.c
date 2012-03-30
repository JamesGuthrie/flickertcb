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
#include <linux/module.h>
#include <asm/msr.h> /* for MSR_MTRRcap */
#else  // _WIN32
#include "wintypes.h"
#include "msr.h"
#endif // _WIN32

#include "acpi.h"
#include "mtrrs.h"
#include "tpm.h"

#define MTRR_TYPE_MIXED         -1
#define MMIO_APIC_BASE          0xFEE00000
#define NR_MMIO_APIC_PAGES      1
#define NR_MMIO_IOAPIC_PAGES    1
#define NR_MMIO_PCICFG_PAGES    1

static void print_mtrrs(const mtrr_state_t *saved_state)
{
    int i;
    dbg("mtrr_def_type: e = %d, fe = %d, type = %x",
        saved_state->mtrr_def_type.e, saved_state->mtrr_def_type.fe,
        saved_state->mtrr_def_type.type );
    dbg("mtrrs:");
    dbg("\t\tbase\tmask\ttype\tv");
    for ( i = 0; i < saved_state->num_var_mtrrs; i++ ) {
        dbg("\t\t%6.6x\t%6.6x\t%2.2x\t%d",
            saved_state->mtrr_physbases[i].base,
            saved_state->mtrr_physmasks[i].mask,
            saved_state->mtrr_physbases[i].type,
            saved_state->mtrr_physmasks[i].v );
    }
}

void save_mtrrs(mtrr_state_t *saved_state)
{
    mtrr_cap_t mtrr_cap;
    int ndx;

//    if (!validate_mtrrs(saved_state)) {
//      dbg("Validating MTRRs before saving them failed!");
//    } else {
//      dbg("MTRRs validated successfully");
//    }
    dbg("Reading the MTRR_DEF_TYPE MSR");

    /* IA32_MTRR_DEF_TYPE MSR */
    rdmsrl(MSR_MTRRdefType, saved_state->mtrr_def_type.raw);

    /* number variable MTTRRs */
    rdmsrl(MSR_MTRRcap, mtrr_cap.raw);
    if ( mtrr_cap.vcnt > MAX_VARIABLE_MTRRS ) {
        /* print warning but continue saving what we can */
        /* (set_mem_type() won't exceed the array, so we're safe doing this) */
        dbg("actual # var MTRRs (%d) > MAX_VARIABLE_MTRRS (%d)",
               mtrr_cap.vcnt, MAX_VARIABLE_MTRRS);
        saved_state->num_var_mtrrs = MAX_VARIABLE_MTRRS;
    }
    else
        saved_state->num_var_mtrrs = (int) mtrr_cap.vcnt;

    /* physmask's and physbase's */
    for ( ndx = 0; ndx < saved_state->num_var_mtrrs; ndx++ ) {
        rdmsrl(MTRR_PHYS_MASK0_MSR + ndx*2,
               saved_state->mtrr_physmasks[ndx].raw);
        rdmsrl(MTRR_PHYS_BASE0_MSR + ndx*2,
               saved_state->mtrr_physbases[ndx].raw);
    }
//    if (!validate_mtrrs(saved_state)) {
//      dbg("Validating MTRRs after saving them failed!");
//    } else {
//      dbg("MTRRs validated successfully after saving ");
//    }
    print_mtrrs(saved_state);
}

/* base should be 4k-bytes aligned, no invalid overlap combination */
//static int get_page_type(const mtrr_state_t *saved_state, uint32_t base)
//{
//    int type = -1;
//    int i;
//    bool wt = false;
//
//    /* omit whether the fix mtrrs are enabled, just check var mtrrs */
//    base >>= PAGE_SHIFT;
//    for ( i = 0; i < saved_state->num_var_mtrrs; i++ ) {
//        const mtrr_physbase_t *base_i = &saved_state->mtrr_physbases[i];
//        const mtrr_physmask_t *mask_i = &saved_state->mtrr_physmasks[i];
//
//        if ( mask_i->v == 0 )
//            continue;
//        if ( (base & mask_i->mask)
//                != (base_i->base & mask_i->mask) )
//            continue;
//
//        type = base_i->type;
//        if ( type == MTRR_TYPE_UNCACHABLE )
//            return MTRR_TYPE_UNCACHABLE;
//        if ( type == MTRR_TYPE_WRTHROUGH )
//            wt = true;
//    }
//    if ( wt )
//        return MTRR_TYPE_WRTHROUGH;
//    if ( type != -1 )
//        return type;
//
//    return saved_state->mtrr_def_type.type;
//}

//static int get_region_type(const mtrr_state_t *saved_state,
//                           uint32_t base, uint32_t pages)
//{
//    int type;
//    uint32_t end;
//
//    if ( pages == 0 )
//        return MTRR_TYPE_MIXED;
//
//    /* wrap the 4G address space */
//    if ( ((uint32_t)(~0) - base) < (pages << PAGE_SHIFT) )
//        return MTRR_TYPE_MIXED;
//
//    if ( saved_state->mtrr_def_type.e == 0 )
//        return MTRR_TYPE_UNCACHABLE;
//
//    /* align to 4k page boundary */
//    base &= PAGE_MASK;
//    end = base + (pages << PAGE_SHIFT);
//
//    type = get_page_type(saved_state, base);
//    base += PAGE_SIZE;
//    for ( ; base < end; base += PAGE_SIZE )
//        if ( type != get_page_type(saved_state, base) )
//            return MTRR_TYPE_MIXED;
//
//    return type;
//}

/* static bool validate_mmio_regions(const mtrr_state_t *saved_state) */
/* { */
/*     acpi_table_mcfg_t *acpi_table_mcfg; */
/*     acpi_table_ioapic_t *acpi_table_ioapic; */

/*     dbg("enter."); */

/*     /\* mmio space for TXT private config space should be UC*\/ */
/*     if ( get_region_type(saved_state, TXT_PRIV_CONFIG_REGS_BASE,  */
/*                          NR_TXT_CONFIG_PAGES) */
/*            != MTRR_TYPE_UNCACHABLE ) { */
/*         dbg("MMIO space for TXT private config space should be UC"); */
/*         return false; */
/*     } */

/*     /\* mmio space for TXT public config space should be UC*\/ */
/*     if ( get_region_type(saved_state, TXT_PUB_CONFIG_REGS_BASE,  */
/*                          NR_TXT_CONFIG_PAGES) */
/*            != MTRR_TYPE_UNCACHABLE ) { */
/*         dbg("MMIO space for TXT public config space should be UC"); */
/*         return false; */
/*     } */

/*     /\* mmio space for TPM should be UC *\/ */
/*     if ( get_region_type(saved_state, TPM_LOCALITY_BASE,  */
/*                          NR_TPM_LOCALITY_PAGES * TPM_NR_LOCALITIES) */
/*            != MTRR_TYPE_UNCACHABLE ) { */
/*         dbg("MMIO space for TPM should be UC"); */
/*         return false; */
/*     } */

/*     /\* mmio space for APIC should be UC *\/ */
/*     if ( get_region_type(saved_state, MMIO_APIC_BASE, NR_MMIO_APIC_PAGES) */
/*            != MTRR_TYPE_UNCACHABLE ) { */
/*         dbg("MMIO space for APIC should be UC"); */
/*         return false; */
/*     } */

/*     dbg("about to get_acpi_ioapic_table()"); */
/*     /\* mmio space for IOAPIC should be UC *\/ */
/*     acpi_table_ioapic = (acpi_table_ioapic_t *)get_acpi_ioapic_table(); */
/*     if ( acpi_table_ioapic == NULL) { */
/*         dbg("acpi_table_ioapic == NULL"); */
/*         return false; */
/*     } */
/*     dbg("acpi_table_ioapic @ %p, .address = %x", */
/*            acpi_table_ioapic, acpi_table_ioapic->address); */
/*     if ( get_region_type(saved_state, acpi_table_ioapic->address, */
/*                          NR_MMIO_IOAPIC_PAGES) */
/*            != MTRR_TYPE_UNCACHABLE ) { */
/*         dbg("MMIO space(%x) for IOAPIC should be UC", */
/*                acpi_table_ioapic->address); */
/*         return false; */
/*     } */

/*     /\* mmio space for PCI config space should be UC *\/ */
/*     acpi_table_mcfg = (acpi_table_mcfg_t *)get_acpi_mcfg_table(); */
/*     if ( acpi_table_mcfg == NULL) { */
/*         dbg("acpi_table_mcfg == NULL"); */
/*         return false; */
/*     } */
/*     dbg("acpi_table_mcfg @ %p, .base_address = %x", */
/*            acpi_table_mcfg, acpi_table_mcfg->base_address); */
/*     if ( get_region_type(saved_state, acpi_table_mcfg->base_address, */
/*                          NR_MMIO_PCICFG_PAGES) */
/*            != MTRR_TYPE_UNCACHABLE ) { */
/*         dbg("MMIO space(%x) for PCI config space should be UC", */
/*                acpi_table_mcfg->base_address); */
/*         return false; */
/*     } */

/*     dbg("exit."); */

/*     return true; */
/* } */

bool validate_mtrrs(const mtrr_state_t *saved_state)
{
    mtrr_cap_t mtrr_cap;
    int ndx;

    /* check is meaningless if MTRRs were disabled */
    if ( saved_state->mtrr_def_type.e == 0 ) {
    dbg("MTRRs were disabled! *************");
        return true;
    }

    /* number variable MTRRs */
    rdmsrl(MSR_MTRRcap, mtrr_cap.raw);
    if ( mtrr_cap.vcnt < saved_state->num_var_mtrrs ) {
        dbg("actual # var MTRRs (%d) < saved # (%d)",
               mtrr_cap.vcnt, saved_state->num_var_mtrrs);
        return false;
    }

    /* variable MTRRs describing non-contiguous memory regions */
    /* TBD: assert(MAXPHYADDR == 36); */
    for ( ndx = 0; ndx < saved_state->num_var_mtrrs; ndx++ ) {
        uint64_t tb;

        if ( saved_state->mtrr_physmasks[ndx].v == 0 )
            continue;

        for ( tb = 0x1; tb != 0x1000000; tb = tb << 1 )
            if ( (tb & saved_state->mtrr_physmasks[ndx].mask) != 0 )
                break;
        for ( ; tb != 0x1000000; tb = tb << 1 )
            if ( (tb & saved_state->mtrr_physmasks[ndx].mask) == 0 )
                break;
        if ( tb != 0x1000000 ) {
            dbg("var MTRRs with non-contiguous regions: "
                   "base=%06x, mask=%06x",
                   (unsigned int) saved_state->mtrr_physbases[ndx].base,
                   (unsigned int) saved_state->mtrr_physmasks[ndx].mask);
            print_mtrrs(saved_state);
            return false;
        }
    }

    /* overlaping regions with invalid memory type combinations */
    for ( ndx = 0; ndx < saved_state->num_var_mtrrs; ndx++ ) {
        int i;
        const mtrr_physbase_t *base_ndx = &saved_state->mtrr_physbases[ndx];
        const mtrr_physmask_t *mask_ndx = &saved_state->mtrr_physmasks[ndx];

        if ( mask_ndx->v == 0 )
            continue;

        for ( i = ndx + 1; i < saved_state->num_var_mtrrs; i++ ) {
            int j;
            const mtrr_physbase_t *base_i = &saved_state->mtrr_physbases[i];
            const mtrr_physmask_t *mask_i = &saved_state->mtrr_physmasks[i];

            if ( mask_i->v == 0 )
                continue;

            if ( (base_ndx->base & mask_ndx->mask & mask_i->mask)
                    != (base_i->base & mask_i->mask)
                 && (base_i->base & mask_i->mask & mask_ndx->mask)
                    != (base_ndx->base & mask_ndx->mask) )
                continue;

            if ( base_ndx->type == base_i->type )
                continue;
            if ( base_ndx->type == MTRR_TYPE_UNCACHABLE
                 || base_i->type == MTRR_TYPE_UNCACHABLE )
                continue;
            if ( base_ndx->type == MTRR_TYPE_WRTHROUGH
                 && base_i->type == MTRR_TYPE_WRBACK )
                continue;
            if ( base_ndx->type == MTRR_TYPE_WRBACK
                 && base_i->type == MTRR_TYPE_WRTHROUGH )
                continue;

            /* 2 overlapped regions have invalid mem type combination, */
            /* need to check whether there is a third region which has type */
            /* of UNCACHABLE and contains at least one of these two regions. */
            /* If there is, then the combination of these 3 region is valid */
            for ( j = 0; j < saved_state->num_var_mtrrs; j++ ) {
                const mtrr_physbase_t *base_j
                        = &saved_state->mtrr_physbases[j];
                const mtrr_physmask_t *mask_j
                        = &saved_state->mtrr_physmasks[j];

                if ( mask_j->v == 0 )
                    continue;

                if ( base_j->type != MTRR_TYPE_UNCACHABLE )
                    continue;

                if ( (base_ndx->base & mask_ndx->mask & mask_j->mask)
                        == (base_j->base & mask_j->mask)
                     && (mask_j->mask & ~mask_ndx->mask) == 0 )
                    break;

                if ( (base_i->base & mask_i->mask & mask_j->mask)
                        == (base_j->base & mask_j->mask)
                     && (mask_j->mask & ~mask_i->mask) == 0 )
                    break;
            }
            if ( j < saved_state->num_var_mtrrs )
                continue;

            dbg("var MTRRs overlaping regions, invalid type combinations");
            print_mtrrs(saved_state);
            return false;
        }
    }

    dbg("IO REGION VALIDATION DISABLED");
/*     if ( !validate_mmio_regions(saved_state) ) { */
/*         dbg("Some mmio region should be UC type"); */
/*         print_mtrrs(saved_state); */
/*         return false; */
/*     } */

    print_mtrrs(saved_state);
    return true;
}

void restore_mtrrs(mtrr_state_t *saved_state)
{
    int ndx;

    /* disable all MTRRs first */
    set_all_mtrrs(false);

    /* physmask's and physbase's */
    for ( ndx = 0; ndx < saved_state->num_var_mtrrs; ndx++ ) {
        wrmsrl(MTRR_PHYS_MASK0_MSR + ndx*2,
               saved_state->mtrr_physmasks[ndx].raw);
        wrmsrl(MTRR_PHYS_BASE0_MSR + ndx*2,
               saved_state->mtrr_physbases[ndx].raw);
    }

    /* IA32_MTRR_DEF_TYPE MSR */
    wrmsrl(MSR_MTRRdefType, saved_state->mtrr_def_type.raw);

    dbg("Restored the MTRRs");
    print_mtrrs(saved_state);
}

/*
 * set the memory type for specified range (base to base+size)
 * to mem_type and everything else to UC
 */
bool set_mem_type(void *base, uint32_t size, uint32_t mem_type)
{
    int num_pages;
    int ndx;
    mtrr_def_type_t mtrr_def_type;
    mtrr_cap_t mtrr_cap;
    mtrr_physmask_t mtrr_physmask;
    mtrr_physbase_t mtrr_physbase;

    /*
     * disable all fixed MTRRs
     * set default type to UC
     */
    rdmsrl(MSR_MTRRdefType, mtrr_def_type.raw);
    mtrr_def_type.fe = 0;
    mtrr_def_type.type = MTRR_TYPE_UNCACHABLE;
    dbg("mtrr_def_type.type=%08x", mtrr_def_type.type);
    dbg("mtrr_def_type.fe  =%08x", mtrr_def_type.fe);
    dbg("mtrr_def_type.e   =%08x", mtrr_def_type.e);
    wrmsrl(MSR_MTRRdefType, mtrr_def_type.raw);

    /*
     * initially disable all variable MTRRs (we'll enable the ones we use)
     */
    rdmsrl(MSR_MTRRcap, mtrr_cap.raw);
    dbg("mtrr_cap.vcnt=%08x", mtrr_cap.vcnt);
    dbg("mtrr_cap.fix =%08x", mtrr_cap.fix);
    dbg("mtrr_cap.wc  =%08x", mtrr_cap.wc);
    dbg("mtrr_cap.rsv2=%08x", (unsigned int)mtrr_cap.reserved2);
    for ( ndx = 0; ndx < mtrr_cap.vcnt; ndx++ ) {
        rdmsrl(MTRR_PHYS_MASK0_MSR + ndx*2, mtrr_physmask.raw);
        mtrr_physmask.v = 0;
        wrmsrl(MTRR_PHYS_MASK0_MSR + ndx*2, mtrr_physmask.raw);
    }

    /*
     * map all AC module pages as mem_type
     */

    num_pages = (size + PAGE_SIZE - 1) >> PAGE_SHIFT;
    ndx = 0;

    dbg("setting MTRRs for acmod: base=%p, size=%x, num_pages=%d",
           base, size, num_pages);

    while ( num_pages > 0 ) {
        uint32_t pages_in_range;

        /* set the base of the current MTRR */
        rdmsrl(MTRR_PHYS_BASE0_MSR + ndx*2, mtrr_physbase.raw);
        mtrr_physbase.base = (unsigned long)base >> PAGE_SHIFT;
        mtrr_physbase.type = mem_type;
        /* set explicitly in case base field is >24b (MAXPHYADDR >36) */
        mtrr_physbase.reserved2 = 0;

        wrmsrl(MTRR_PHYS_BASE0_MSR + ndx*2, mtrr_physbase.raw);

        /*
         * calculate MTRR mask
         * MTRRs can map pages in power of 2
         * may need to use multiple MTRRS to map all of region
         */
        pages_in_range = 1 << (fls(num_pages) - 1);

        rdmsrl(MTRR_PHYS_MASK0_MSR + ndx*2, mtrr_physmask.raw);
        mtrr_physmask.mask = ~(pages_in_range - 1);
        mtrr_physmask.v = 1;
        /* set explicitly in case mask field is >24b (MAXPHYADDR >36) */
    mtrr_physmask.reserved2 = 0;

        wrmsrl(MTRR_PHYS_MASK0_MSR + ndx*2, mtrr_physmask.raw);

        /* Volume3A_SystemProgrammingGuide.pdf */
        /* 10.11.2.3 Variable Range MTTRs */
        /* Figure 10-6 defines the PHYSBASE and PHYSMASK register */
        dbg(" base=%08x", mtrr_physbase.base);
        dbg(" type=%08x", mtrr_physbase.type);
        dbg(" mask=%08x", mtrr_physmask.mask);

        /* prepare for the next loop depending on number of pages
         * We figure out from the above how many pages could be used in this
         * mtrr. Then we decrement the count, increment the base,
         * increment the mtrr we are dealing with, and if num_pages is
         * still not zero, we do it again.
         */
        base = (char*)base + (pages_in_range * PAGE_SIZE);
        num_pages -= pages_in_range;
        ndx++;
        if ( ndx == mtrr_cap.vcnt ) {
            dbg("exceeded number of var MTRRs when mapping range");
            return false;
        }
    }

    return true;
}

/* enable/disable all MTRRs */
void set_all_mtrrs(bool enable)
{
    mtrr_def_type_t mtrr_def_type;

    rdmsrl(MSR_MTRRdefType, mtrr_def_type.raw);
    mtrr_def_type.e = enable ? 1 : 0;
    wrmsrl(MSR_MTRRdefType, mtrr_def_type.raw);
}

