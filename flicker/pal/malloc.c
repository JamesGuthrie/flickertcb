/**
 * malloc.c - Provide memory allocation / freeing to the PAL.
 *
 * Copyright (C) 2006-2009 Bryan Parno
 * Copyright (C) 2012 Jonathan McCune
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
#include "tlsf.h"

#define HEAP_SIZE (1024*128)
static tlsf_pool g_pool;

void static_malloc_init(void) {
    static unsigned char heap[HEAP_SIZE];
    g_pool = tlsf_create(heap, HEAP_SIZE);
}

void *malloc(size_t bytes) {
    return tlsf_malloc(g_pool, bytes);
}

void *memalign(size_t align, size_t bytes) {
    return tlsf_memalign(g_pool, align, bytes);
}

void *realloc(void *ptr, size_t size) {
    return tlsf_realloc(g_pool, ptr, size);
}

void free(void *ptr) {
    tlsf_free(g_pool, ptr);
}

