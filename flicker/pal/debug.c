/*
 * Copyright (C) 2012 Jonathan M. McCune
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 */

#include "malloc.h"
#include "printk.h"
#include "tpm.h"
#include "sha1.h"
#include "util.h"
#include "../common/resume.h" /* cpu_t */
#include "debug.h"

/**
 * The functions in this file exist solely for debug reasons.
 * Understanding what they print out is very useful in understanding
 * exactly what is happening when a PAL executes.  None of these are
 * actually necessary in a stable, mature PAL.
 */

/* debug code to dump populated regions of page tables */
void dump_populated_areas(uint8_t *pts) {
    int i;
    uint8_t* ptr;
    int ever_print = 0;
    uint32_t val;
    printk("dump_populated_areas @ %p ENTER\n", pts);
    for(i=0; i<12*1024/4; i++) { /* 12KB of memory to scan, prints 4 bytes/iter. */
        ptr = pts + i*4;
        val = *((uint32_t*)ptr);
        if(0 != val) { /*non-zero*/
            printk("PT addr %p, contents %08x\n", ptr, val);
            ever_print++;
        }
    }
    printk("Printed %d lines of PT\n", ever_print);
    printk("dump_populated_areas EXIT\n");
}

void dump_cpu_t(cpu_t* s) {
    if(s == NULL) { return; }

    printk("eax %08lx, ebx %08lx, ecx %08lx, edx %08lx, edi %08lx, esi %08lx\n",
        s->eax, s->ebx, s->ecx, s->edx, s->edi, s->esi);

    printk("cs %04x, ds %04x, es %04x, fs %04x, gs %04x, ss %04x\n",
        s->cs, s->ds, s->es, s->fs, s->gs, s->ss);

    printk("eflags %08lx, efer %08lx, esp %08lx, ebp %08lx\n",
        s->eflags, s->efer, s->esp, s->ebp);

    printk("cr0 %08lx, cr2 %08lx, cr3 %08lx, cr4 %08lx\n",
        s->cr0, s->cr2, s->cr3, s->cr4);

    printk("gdt_base:gdt_limit %08lx:%04x, idt_base:idt_limit %08lx:%04x\n",
        s->gdt_base, s->gdt_limit, s->idt_base, s->idt_limit);

    printk("ldt %04x, tss %04x, tr %08lx, safety %08lx, return_address %08lx\n",
        s->ldt, s->tss, s->tr, s->safety, s->return_address);

    printk("skinit timer sh:sl eh:el 0x%08lx:0x%08lx 0x%08lx:0x%08lx\n",
        s->starth, s->startl, s->endh, s->endl);

    printk("p2v_offset 0x%08lx\n", s->p2v_offset);
}

void get_pcr17(void) {
    unsigned char v[SHA_DIGEST_LENGTH];
    int result;
    int locality = 2;

    result = tpm_pcr_read(locality, 17, (tpm_pcr_value_t*)v);
    if(TPM_SUCCESS != result) {
        log_event(LOG_LEVEL_VERBOSE, "ERROR: Failed to read PCR-17");
    }
    log_event(LOG_LEVEL_VERBOSE,
              "PCR-17: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x "
              "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
              v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[9],
              v[10], v[11], v[12], v[13], v[14], v[15], v[16], v[17], v[18], v[19]);
}

void gdt_debug(void) {
    uint8_t * gdt_ptr;

    printk("slb_base_phys(): 0x%08x\n", slb_base_phys());
    printk("GDT:\n");

    gdt_ptr = (uint8_t*)52; /* 4-byte AMD header + 44-byte Intel header; hackish */

    dump_bytes(gdt_ptr, 8); gdt_ptr += 8;
    dump_bytes(gdt_ptr, 8); gdt_ptr += 8;
    dump_bytes(gdt_ptr, 8); gdt_ptr += 8;
    dump_bytes(gdt_ptr, 8); gdt_ptr += 8;
    dump_bytes(gdt_ptr, 8); gdt_ptr += 8;
    dump_bytes(gdt_ptr, 8); gdt_ptr += 8;
    dump_bytes(gdt_ptr, 8); gdt_ptr += 8;
    dump_bytes(gdt_ptr, 8); gdt_ptr += 8;
    dump_bytes(gdt_ptr, 8); gdt_ptr += 8;
    dump_bytes(gdt_ptr, 8); gdt_ptr += 8;
    dump_bytes(gdt_ptr, 8); gdt_ptr += 8;

    printk("TSS:\n");
    dump_bytes(gdt_ptr, 4); gdt_ptr += 4;
    dump_bytes(gdt_ptr, 8); gdt_ptr += 8;
    dump_bytes(gdt_ptr, 8); gdt_ptr += 8;
    dump_bytes(gdt_ptr, 8); gdt_ptr += 8;

    printk("SLB/MLE size: %08x\n", *((unsigned short*)(2))); /* grabs from SLB header */
}

#define HOWMANYPTRS 12
#define MAXALLOC (30*1024)
static int stress_malloc_internal(void)
{
    int i, j, rv;
    unsigned int size, got = sizeof(size);
    char *ptrs[HOWMANYPTRS];

    for(i=0; i<HOWMANYPTRS; i++) ptrs[i] = NULL;

    printk("Hello from stress_malloc()\n");
    for(i=0; i<HOWMANYPTRS; i++) {
        printk("malloc stress iteration %d\n", i);
        if(TPM_SUCCESS != (rv = tpm_get_random(2, (uint8_t*)&size, &got)) ||
           got != sizeof(size)) {
            printk("FATAL ERROR: tpm_get_random failed with rv %d, got %d\n", rv, got);
            return -1;
        }
        size %= MAXALLOC;
        printk("  allocating %d bytes..\n", size);
        ptrs[i] = malloc(size);
        if(NULL != ptrs[i]) {
            for(j=0; j<size; j++) {
                ptrs[i][j] = j; /* touch every location */
            }
        } else {
            printk("  ALLOCATION FAILED! malloc() returned NULL\n");
        }
    }

    for(i=0; i<HOWMANYPTRS; i++) {
        printk("Freeing ptrs[%d] (%s)\n", i, ptrs[i] ? "NON-NULL" : "NULL");
        free(ptrs[i]);
    }
    return 0;
}

void stress_malloc(void) {
    int i;
    for(i=0; i<10; i++) {
        printk("************************************* %d\n", i);
        stress_malloc_internal();
    }
}
