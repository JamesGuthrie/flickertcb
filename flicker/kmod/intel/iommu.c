/*
 * iommu.c: Intel(r) VT-d support functions.
 *
 * Copyright (c) 2012, Jonathan McCune
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

#ifndef _WIN32
#include <linux/module.h>
#include <linux/dmar.h>
#include <linux/list.h>
#include <linux/intel-iommu.h>
#include <linux/kallsyms.h> /* kallsyms_lookup_name(); Yikes! */
#include <asm/io.h> /* readl(), writel(), virt_to_phys() */
#else
#error "Windows DMAR / IOMMU currently unsupported"
#endif

#include "flicker.h"

/**
 * The "real" dmar_drhd_units symbol is defined extern in dmar.h,
 * but the actual variable can't be found when the module is
 * inserted because it's not an exported symbol.  Do the hideous
 * thing: use kallsyms_lookup_name(). */
static inline struct list_head * get_dmar_drhd_units(void) {
    return (struct list_head *)kallsyms_lookup_name("dmar_drhd_units");
}

/**
 * Per section 10.4.16 in the Intel VT-d spec, the Protected Memory
 * Enable Register has two bits of interest.  The "Enable Protected
 * Memory (EPM)" bit, which controls the desired state of the PMRs,
 * and the "Protected Region Status (PRS)" bit, which gives the
 * region's status.
 *
 * The basic idea here is to disable the PMRs by clearing EPM, and
 * then waiting until the PMRs are actually disabled, which is evident
 * by observing that the PRS bit has also cleared.
 */
int linux_intel_disable_pmr(void)
{
    uint32_t pmen;
    int max_loop;
    int rv = 0;
    
    struct list_head *dmar_drhd_units;
    struct dmar_drhd_unit *drhd;

    dmar_drhd_units = get_dmar_drhd_units();
    //dbg("Symbol dmar_drhd_units found at %p", dmar_drhd_units);
    if(NULL == dmar_drhd_units) { return -ENODEV; }

    /* Can't use for_each_active_iommu or for_each_iommu, because
       dmar_drhd_units is expected to be a global, not a pointer. */
    list_for_each_entry(drhd, dmar_drhd_units, list) {        
        if(NULL != drhd->iommu && 0xffffffff != (uint32_t)drhd->iommu) {
            pmen = readl(drhd->iommu->reg + DMAR_PMEN_REG);
            pmen &= ~DMA_PMEN_EPM;
            writel(pmen, drhd->iommu->reg + DMAR_PMEN_REG);
            
            max_loop = 100; /* XXX Not sure what a sane value might be. */
            do {
                pmen = readl(drhd->iommu->reg + DMAR_PMEN_REG);
                //dbg("Checking DMA_PMEN_PRS: max_loop=%d, pmen=0x%08x", max_loop, pmen);
                max_loop--;
            } while((pmen & DMA_PMEN_PRS) && max_loop > 0);
            
            if(max_loop <= 0) {
                rv = -ENODEV;
            }
        }
    }

    return rv;    
}


/**
 * Prints some debug information about VT-d DRHD.
 */
void linux_drhd_iommu_dbg(void) {
    struct list_head *dmar_drhd_units;
    struct dmar_drhd_unit *drhd;

    dmar_drhd_units = get_dmar_drhd_units();
    dbg("Symbol dmar_drhd_units found at %p", dmar_drhd_units);
    if(NULL == dmar_drhd_units) { return; }

    /* Can't use for_each_[active_]iommu, because dmar_drhd_units is
       expected to be a global, not a pointer. */
    list_for_each_entry(drhd, dmar_drhd_units, list) {        
        dbg("drhd found at %p, v2p %x", drhd, virt_to_phys(drhd));
        dbg("drhd->reg_base_addr %Lx", drhd->reg_base_addr);
        dbg("drhd->iommu found with value %p, v2p %x", drhd->iommu,
            virt_to_phys(drhd->iommu));
        if(NULL != drhd->iommu && 0xffffffff != (uint32_t)drhd->iommu) {
            dbg("drhd->iommu->reg  %p, v2p %x", drhd->iommu->reg,
                virt_to_phys(drhd->iommu->reg));
            dbg("drhd->iommu->name %s", drhd->iommu->name);
        }
    }
}

/*
 * Local variables:
 * mode: C
 * c-set-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
