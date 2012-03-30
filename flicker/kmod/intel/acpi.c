/*
 * acpi.c: ACPI utility fns
 *
 * Copyright (c) 2010, Intel Corporation
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
 */

/*
 *  acpi.c: Modified for Flicker
 */


#ifndef _WIN32
#include <linux/module.h>
#include <asm/io.h> //inw
#else  // _WIN32
#include "wintypes.h"
#endif // _WIN32

#include "acpi.h"
#include "log.h"

bool g_no_usb = true;

static struct acpi_rsdp *rsdp;
static struct acpi_table_header *g_dmar_table;
static bool g_hide_dmar;

/* static void dump_gas(const char *reg_name, */
/*                      const tboot_acpi_generic_address_t *reg) */
/* { */
/*     const char *space_id[] = { "memory", "I/O", "PCI config space", "EC", */
/*                                "SMBus" }; */

/*     logit("%s GAS @ %p:\n", reg_name, reg); */
/*     if ( reg == NULL ) */
/*         return; */

/*     if ( reg->space_id >= ARRAY_SIZE(space_id) ) */
/*         logit("\t space_id: unsupported (%u)\n", reg->space_id); */
/*     else */
/*         logit("\t space_id: %s\n", space_id[reg->space_id]); */
/*     logit("\t bit_width: %u\n", reg->bit_width); */
/*     logit("\t bit_offset: %u\n", reg->bit_offset); */
/*     logit("\t access_width: %u\n", reg->access_width); */
/*     logit("\t address: %Lx\n", reg->address); */
/* } */

static inline struct acpi_rsdt *get_rsdt(void)
{
    return (struct acpi_rsdt *)rsdp->rsdp1.rsdt;
}

static inline struct acpi_xsdt *get_xsdt(void)
{
    if ( rsdp->rsdp_xsdt >= 0x100000000ULL ) {
        logit("XSDT above 4GB\n");
        return NULL;
    }
    return (struct acpi_xsdt *)(uintptr_t)rsdp->rsdp_xsdt;
}

static bool verify_acpi_checksum(uint8_t *start, uint8_t len)
{
    uint8_t sum = 0;
    while ( len ) {
        sum += *start++;
        len--;
    }
    return (sum == 0);
}

static bool find_rsdp_in_range(void *start, void *end)
{
#define RSDP_BOUNDARY    16  /* rsdp ranges on 16-byte boundaries */
#define RSDP_CHKSUM_LEN  20  /* rsdp check sum length, defined in ACPI 1.0 */

    for ( ; (char*)start < (char*)end; start = (char*)start + RSDP_BOUNDARY ) {
        rsdp = (struct acpi_rsdp *)start;

        if ( memcmp(rsdp->rsdp1.signature, RSDP_SIG,
                    sizeof(rsdp->rsdp1.signature)) == 0 ) {
            if ( verify_acpi_checksum((uint8_t *)rsdp, RSDP_CHKSUM_LEN) ) {
                logit("RSDP (v%u, %.6s) @ %p\n", rsdp->rsdp1.revision,
                       rsdp->rsdp1.oemid, rsdp);
                return true;
            }
            else {
                logit("checksum failed.\n");
                return false;
            }
        }
    }
    return false;
}

static bool find_rsdp(void)
{

    if ( rsdp != NULL )
        return true;

    /*  0x00 - 0x400 */
    if ( find_rsdp_in_range(RSDP_SCOPE1_LOW, RSDP_SCOPE1_HIGH) )
        return true;

    /* 0xE0000 - 0x100000 */
    if ( find_rsdp_in_range(RSDP_SCOPE2_LOW, RSDP_SCOPE2_HIGH) )
        return true;

    logit("cann't find RSDP\n");
    rsdp = NULL;
    return false;
}

/* this function can find dmar table whether or not it was hidden */
static struct acpi_table_header *find_table(const char *table_name)
{
    struct acpi_table_header *table = NULL;

    if ( !find_rsdp() ) {
        logit("no rsdp to use\n");
        return NULL;
    }

    if ( rsdp->rsdp1.revision == 2 ) { /*  ACPI 2.0 */
        uint64_t *curr_table;
        struct acpi_xsdt *xsdt = get_xsdt();

        for ( curr_table = xsdt->table_offsets;
              curr_table < (uint64_t *)((char *)xsdt + xsdt->hdr.length);
              curr_table++ ) {
            table = (struct acpi_table_header *)(uintptr_t)*curr_table;
            if ( memcmp(table->signature, table_name,
                        sizeof(table->signature)) == 0 )
                return table;
        }
    }
    else {                             /* ACPI 1.0 */
        uint32_t *curr_table;
        struct acpi_rsdt *rsdt = get_rsdt();

        for ( curr_table = rsdt->table_offsets;
              curr_table < (uint32_t *)((char *)rsdt + rsdt->hdr.length);
              curr_table++ ) {
            table = (struct acpi_table_header *)(uintptr_t)*curr_table;
            if ( memcmp(table->signature, table_name,
                        sizeof(table->signature)) == 0 )
                return table;
        }
    }

    logit("cann't find %s table.\n", table_name);
    return NULL;
}

static struct acpi_dmar *get_vtd_dmar_table(void)
{
    return (struct acpi_dmar *)find_table(DMAR_SIG);
}

bool save_vtd_dmar_table(void)
{
    /* find DMAR table and save it */
    g_dmar_table = (struct acpi_table_header *)get_vtd_dmar_table();

    logit("DMAR table @ %p saved.\n", g_dmar_table);
    return true;
}

bool restore_vtd_dmar_table(void)
{
    struct acpi_table_header *hdr;

    g_hide_dmar = false;

    /* find DMAR table first */
    hdr = (struct acpi_table_header *)get_vtd_dmar_table();
    if ( hdr != NULL ) {
        logit("DMAR table @ %p is still there, skip restore step.\n", hdr);
        return true;
    }

    /* check saved DMAR table */
    if ( g_dmar_table == NULL ) {
        logit("No DMAR table saved, abort restore step.\n");
        return false;
    }

    /* restore DMAR if needed */
    memcpy(g_dmar_table->signature, DMAR_SIG, sizeof(g_dmar_table->signature));

    /* need to hide DMAR table while resume from S3 */
    g_hide_dmar = true;
    logit("DMAR table @ %p restored.\n", hdr);
    return true;
}

bool remove_vtd_dmar_table(void)
{
    struct acpi_table_header *hdr;

    /* check whether it is needed */
    if ( !g_hide_dmar ) {
        logit("No need to hide DMAR table.\n");
        return true;
    }

    /* find DMAR table */
    hdr = (struct acpi_table_header *)get_vtd_dmar_table();
    if ( hdr == NULL ) {
        logit("No DMAR table, skip remove step.\n");
        return true;
    }

    /* remove DMAR table */
    hdr->signature[0] = '\0';
    logit("DMAR table @ %p removed.\n", hdr);
    return true;
}

static struct acpi_madt *get_apic_table(void)
{
    return (struct acpi_madt *)find_table(MADT_SIG);
}

struct acpi_table_ioapic *get_acpi_ioapic_table(void)
{
    union acpi_madt_entry *entry;
    struct acpi_madt *madt = get_apic_table();
    if ( madt == NULL ) {
        logit("no MADT table found\n");
        return NULL;
    }

    /* APIC tables begin after MADT */
    entry = (union acpi_madt_entry *)(madt + 1);

    while ( (char *)entry < ((char *)madt + madt->hdr.length) ) {
    	uint8_t length = entry->madt_lapic.length;

    	if ( entry->madt_lapic.apic_type == ACPI_MADT_IOAPIC ) {
    		if ( length != sizeof(entry->madt_ioapic) ) {
                logit("APIC length error.\n");
                return NULL;
            }
            return (struct acpi_table_ioapic *)entry;
        }
    	entry = (union acpi_madt_entry *)((char *)entry + length);
    }
    logit("no IOAPIC type.\n");
    return NULL;
}

struct acpi_mcfg *get_acpi_mcfg_table(void)
{
    return (struct acpi_mcfg *)find_table(MCFG_SIG);
}

/* static bool write_to_reg(const tboot_acpi_generic_address_t *reg, */
/*                          uint32_t val) */
/* { */
/*     if ( reg->address >= 100000000ULL ) { */
/*         logit("GAS address >4GB (0x%Lx)\n", reg->address); */
/*         return false; */
/*     } */
/*     uint32_t address = (uint32_t)reg->address; */

/*     if ( reg->space_id == GAS_SYSTEM_IOSPACE ) { */
/*         switch ( reg->access_width ) { */
/*             case GAS_ACCESS_BYTE: */
/*                 outb(address, (uint8_t)val); */
/*                 return true; */
/*             case GAS_ACCESS_WORD: */
/*             case GAS_ACCESS_UNDEFINED: */
/*                 outw(address, (uint16_t)val); */
/*                 return true; */
/*             case GAS_ACCESS_DWORD: */
/*                 outl(address, val); */
/*                 return true; */
/*             default: */
/*                 logit("unsupported GAS access width: %u\n", reg->access_width); */
/*                 return false; */
/*         } */
/*     } */
/*     else if ( reg->space_id == GAS_SYSTEM_MEMORY ) { */
/*         switch ( reg->access_width ) { */
/*             case GAS_ACCESS_BYTE: */
/*                 writeb(address, (uint8_t)val); */
/*                 return true; */
/*             case GAS_ACCESS_WORD: */
/*             case GAS_ACCESS_UNDEFINED: */
/*                 writew(address, (uint16_t)val); */
/*                 return true; */
/*             case GAS_ACCESS_DWORD: */
/*                 writel(address, val); */
/*                 return true; */
/*             default: */
/*                 logit("unsupported GAS access width: %u\n", reg->access_width); */
/*                 return false; */
/*         } */
/*     } */

/*     logit("unsupported GAS addr space ID: %u\n", reg->space_id); */
/*     return false; */
/* } */

/* static bool read_from_reg(const tboot_acpi_generic_address_t *reg, */
/*                           uint32_t *val) */
/* { */
/*     if ( reg->address >= 100000000ULL ) { */
/*         logit("GAS address >4GB (0x%Lx)\n", reg->address); */
/*         return false; */
/*     } */
/*     uint32_t address = (uint32_t)reg->address; */

/*     if ( reg->space_id == GAS_SYSTEM_IOSPACE ) { */
/*         switch ( reg->access_width ) { */
/*             case GAS_ACCESS_BYTE: */
/*                 *val = inb(address); */
/*                 return true; */
/*             case GAS_ACCESS_WORD: */
/*             case GAS_ACCESS_UNDEFINED: */
/*                 *val = inw(address); */
/*                 return true; */
/*             case GAS_ACCESS_DWORD: */
/*                 *val = inl(address); */
/*                 return true; */
/*             default: */
/*                 logit("unsupported GAS access width: %u\n", reg->access_width); */
/*                 return false; */
/*         } */
/*     } */
/*     else if ( reg->space_id == GAS_SYSTEM_MEMORY ) { */
/*         switch ( reg->access_width ) { */
/*             case GAS_ACCESS_BYTE: */
/*                 *val = readb(address); */
/*                 return true; */
/*             case GAS_ACCESS_WORD: */
/*             case GAS_ACCESS_UNDEFINED: */
/*                 *val = readw(address); */
/*                 return true; */
/*             case GAS_ACCESS_DWORD: */
/*                 *val = readl(address); */
/*                 return true; */
/*             default: */
/*                 logit("unsupported GAS access width: %u\n", reg->access_width); */
/*                 return false; */
/*         } */
/*     } */

/*     logit("unsupported GAS addr space ID: %u\n", reg->space_id); */
/*     return false; */
/* } */

/* static void wait_to_sleep(const tboot_acpi_sleep_info_t *acpi_sinfo) */
/* { */
/* #define WAKE_STATUS    0x8000    /\* the 15th bit *\/ */
/*     while ( true ) { */
/*         uint32_t pm1a_value = 0, pm1b_value = 0; */

/*         if ( acpi_sinfo->pm1a_evt_blk.address ) { */
/*             if ( read_from_reg(&acpi_sinfo->pm1a_evt_blk, &pm1a_value) && */
/*                  ( pm1a_value & WAKE_STATUS ) ) */
/*                 return; */
/*         } */

/*         if ( acpi_sinfo->pm1b_evt_blk.address ) { */
/*             if ( read_from_reg(&acpi_sinfo->pm1b_evt_blk, &pm1b_value) && */
/*                  ( pm1b_value & WAKE_STATUS ) ) */
/*                 return; */
/*         } */
/*     } */
/* } */

/* bool machine_sleep(const tboot_acpi_sleep_info_t *acpi_sinfo) */
/* { */
/*     dump_gas("PM1A", &acpi_sinfo->pm1a_cnt_blk); */
/*     dump_gas("PM1B", &acpi_sinfo->pm1b_cnt_blk); */

/*     wbinvd(); */

/*     if ( acpi_sinfo->pm1a_cnt_blk.address ) { */
/*         if ( !write_to_reg(&acpi_sinfo->pm1a_cnt_blk, acpi_sinfo->pm1a_cnt_val) ) */
/*             return false;; */
/*     } */

/*     if ( acpi_sinfo->pm1b_cnt_blk.address ) { */
/*         if ( !write_to_reg(&acpi_sinfo->pm1b_cnt_blk, acpi_sinfo->pm1b_cnt_val) ) */
/*             return false; */
/*     } */

/*     /\* just to wait, the machine may shutdown before here *\/ */
/*     wait_to_sleep(acpi_sinfo); */
/*     return true;  */
/* } */

/* void set_s3_resume_vector(const tboot_acpi_sleep_info_t *acpi_sinfo, */
/*                           uint64_t resume_vector) */
/* { */
/*     if ( acpi_sinfo->vector_width <= 32 ) */
/*         *(uint32_t *)(unsigned long)(acpi_sinfo->wakeup_vector) = */
/*                                     (uint32_t)resume_vector; */
/*     else if ( acpi_sinfo->vector_width <= 64 ) */
/*         *(uint64_t *)(unsigned long)(acpi_sinfo->wakeup_vector) = */
/*                                     resume_vector; */
/*     else */
/*         logit("vector_width error.\n"); */

/*     dbg("wakeup_vector_address = %llx\n", acpi_sinfo->wakeup_vector); */
/*     dbg("wakeup_vector_value = %llxx\n", resume_vector); */
/* } */

/* void disable_smis(void) */
/* { */
/*     if ( g_no_usb ) { */
/*         logit("disabling legacy USB SMIs\n"); */
/*         uint32_t pmbase = pcireg_cfgread(0, 31, 0, 0x40, 4) & ~1; */
/*         uint32_t smi_en = inl(pmbase + 0x30); */
/*         smi_en &= ~0x20008; */
/*         outl(pmbase + 0x30, smi_en); */
/*     } */
/* } */

/*
 * Local variables:
 * mode: C
 * c-set-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
