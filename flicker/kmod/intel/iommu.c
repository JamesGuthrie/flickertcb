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
#include <asm/io.h> /* virt_to_phys() */
#else
#error "Windows DMAR / IOMMU currently unsupported"
#endif

#include "flicker.h"

int linux_drhd_iommu_dbg(void) {
    struct dmar_drhd_unit *drhd;
    struct intel_iommu *iommu = NULL;

    /* This symbol is defined extern in dmar.h, but the actual
    variable can't be found when the module is inserted because it's
    not an exported symbol.  Do the hideous thing: use
    kallsyms_lookup_name() */
    struct list_head *dmar_drhd_units;

    dmar_drhd_units = (struct list_head *)kallsyms_lookup_name("dmar_drhd_units");

    dbg("Symbol dmar_drhd_units found at %p", dmar_drhd_units);

    if(NULL == dmar_drhd_units) { return -ENODEV; }
    
    /* Can't use for_each_active_iommu or for_each_iommu, because
       dmar_drhd_units is expected to be a global, not a pointer. */
    list_for_each_entry(drhd, dmar_drhd_units, list) {
        iommu=drhd->iommu;
        dbg("drhd found at %p, v2p %x", drhd, virt_to_phys(drhd));
        dbg("drhd->reg_base_addr %Lx", drhd->reg_base_addr);
        dbg("drhd->iommu found with value %p, v2p %x", iommu, virt_to_phys(iommu));
        if(NULL != iommu && 0xffffffff != (uint32_t)iommu) {
            dbg("iommu->reg  %p, v2p %x", iommu->reg, virt_to_phys(iommu->reg));
            dbg("iommu->name %s", iommu->name);
        }
    }

    return 0; /* success */
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
