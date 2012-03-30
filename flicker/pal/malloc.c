/**
 * malloc.c - Provide memory allocation / freeing to the PAL.
 *
 * Copyright (C) 2006-2009 Bryan Parno
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

/* Define TEST_MEM to perform diagnostics on the memory system */
#define TEST_MEM

/* Define DEBUG_MEM to print out debugging info and perform additional
 * consistency checks. BE CAREFUL: This can generate _a lot_ of
 * output! */
//#define DEBUG_MEM

/***********************************************************************
 * Global Variable Declarations
 ***********************************************************************/

/* Pool of memory */
unsigned int memBuffer[MEMORY_BUFFER_SIZE];

/* Tracks which memory slots are available */
Bitslot memAvail[BITSET_SIZE];

/* Index i is set to 1 if it was allocated along with the previous slot */
Bitslot memContig[BITSET_SIZE];

/*
 * Returns a pointer to size bytes of memory or NULL if such a swath
 * is not available.  static_malloc_init() had better have been called
 * already!
 */
void *static_malloc(unsigned int size) {
    int index = 0;
    /* Calculate the number of memory slots we'll need */
    int numSlots = (size % MEMORY_SLOT_SIZE == 0) ? (size / MEMORY_SLOT_SIZE)
        : (size / MEMORY_SLOT_SIZE) + 1;
/* #ifdef DEBUG_MEM */
/*     slb_out_description("Looking for bytes:", size); */
/*     slb_out_description("...in slots:", numSlots); */
/*     slb_out_info("Current availability:"); */
/*     printAvail(); */
/*     slb_out_info("Current contiguity  :"); */
/*     printContig(); */
/* #endif */

    /* Find enough slots to satisfy the request */
    int freeIndex = findFreeSlots(numSlots);

    if (freeIndex == -1) {
        /* Failed to find enough available memory */
        //#ifdef DEBUG_MEM
        printk("Couldn't allocate bytes: %u\n", size);
        //#endif
        return NULL;
    }

    /* Update the availability and contig records */
    for (index = freeIndex; index < freeIndex + numSlots; index++) {
        setSlotAvail(index, FALSE);
        if (index == freeIndex) {
            /* The first contig record is zero to indicate a boundary */
            setSlotContig(index, FALSE);
        } else {
            /* This chunk belongs with the previous slot */
            setSlotContig(index, TRUE);
        }
    }

/* #ifdef DEBUG_MEM */
/*     slb_out_info("Result of allocation:"); */
/*     printAvail(); */
/*     slb_out_info("Result on contig    :"); */
/*     printContig(); */
/* #endif */

    /* Return the memory */
    return (memBuffer + freeIndex);

}

/*
 * Initialize the memory regions
 */
void static_malloc_init() {
    bitsetSetAll(memAvail, BITSET_SIZE);
    bitsetClearAll(memContig, BITSET_SIZE);
}

/*
 * Find enough contiguous slots to allocate memory for this request
 */
int findFreeSlots(int numSlots) {
    int index;
    for (index = 0; index < MEMORY_BUFFER_SIZE - numSlots + 1; index++) {
/* #if 0 */
/*     	if (index % BITSET_SLOT_SIZE == 0 && !testBitSlot(index)) { */
/*     		/\* Check an entire slot at one time as a small optimization *\/ */
/*     		index += BITSET_SLOT_SIZE; */
/*     		continue; */
/*     	} */
/* #endif  */
        if (getSlotAvail(index)) {
            if (numSlots == 1) { /* This is the easy case */
                return index;
            } else if (checkContigSlots(index + 1, numSlots - 1)) {
                /* There are enough subsequent slots */
                return index;
            } else {
                /* Not enough slots in a row.  Keep going */
            }

/* #if 0 */
/*             /\* Check the next numSlots - 1 slots to see if they're available too *\/ */
/*             for (checkIndex = 1; checkIndex < numSlots - 1; checkIndex++) { */
/*                 if (!getSlotAvail(index + checkIndex)) { */
/*                     /\* Oops, found a slot in the middle that isn't available *\/ */
/*                     break; */
/*                 } */
/*             } */
/*             if (checkIndex == numSlots - 1) { /\* The necessary slots are all available *\/ */
/*                 return index; */
/*             } else { */
/*                 /\* The necessary slots are not present here  */
/*                  * Advance the current index, so that we don't recheck this section */
/*                  *\/ */
/*                 index += checkIndex; */
/*             } */
/*         } */
/* #endif */
    }
}

/* If we reach here, we failed to find enough contiguous slots */
return -1;
}

/*
 * Determines whether there are numSlots available starting at index
 */
bool checkContigSlots(int index, int numSlots) {
    int stop = index + numSlots;
    for (;index < stop; index++) {
        if (!getSlotAvail(index)) {
            return FALSE;
        }
    }
    return TRUE;
}

/*
 * Get the availability of memory slot i
 */
bool getSlotAvail(int i) {
    return testBit(memAvail, i);
}

/*
 * Set the availability of memory slot i
 */
void setSlotAvail(int i, bool avail) {
    setBit(memAvail, i, avail);
}

/*
 * Determine if this slot (slot i) is part of the previous slot
 */
bool getSlotContig(int i) {
    return testBit(memContig, i);
}

/*
 * Adjust the continguity record
 */
void setSlotContig(int i, bool contig) {
    setBit(memContig, i, contig);
}

/*
 * Return the pointer's memory chunk back to the general pool
 */
void static_free(void* target) {
    int targetIndex = ((int) target - (int) memBuffer) / MEMORY_SLOT_SIZE;

    do {
/* #ifdef DEBUG_MEM */
/*     	slb_out_description("Freeing index:", targetIndex); */
/* #endif  */
        /* Set this memory free and clear the contig flag */
        setSlotAvail(targetIndex, TRUE);
        setSlotContig(targetIndex, FALSE);
        targetIndex++;
    } while (getSlotContig(targetIndex));  /* Keep going if the next slot belongs to us */

/* #ifdef DEBUG_MEM */
/*     slb_out_info("Result of freeing is:"); */
/*     printAvail(); */
/*     slb_out_info("and effect on contig:"); */
/*     printContig(); */
/* #endif */
}

/* /\* */
/*  * Debug routine to display the bitmaps */
/*  *\/ */
/* void printAvail() { */
/* #ifdef DEBUG_MEM */
/*     int i; */
/*     for (i = 0; i < MEMORY_BUFFER_SIZE; i++) { */
/*     	if (getSlotAvail(i)) { */
/*     		slb_outchar('_'); */
/*     	} else { */
/*     		slb_outchar('+'); */
/*     	} */
/*     } */
/*     slb_outchar('\n'); */
/*     slb_outchar('\r'); */
/* #endif */
/* } */

/* void printContig() { */
/* #ifdef DEBUG_MEM */
/*     int i; */
/*     int ret; */
/*     for (i = 0; i < MEMORY_BUFFER_SIZE; i++) { */
/*     	if ((ret = getSlotContig(i))) { */
/*     		slb_outchar('0'+ret); */
/*     	} else { */
/*     		slb_outchar('-'); */
/*     	} */
/*     } */
/*     slb_outchar('\n'); */
/*     slb_outchar('\r'); */
/* #endif */
/* } */



/***********************************************************************
 * Bitset implementation
 ***********************************************************************/

/* Test whether a particular bit is set */
bool testBit(Bitset bits, int index) {
    /* Find the bucket that this index belongs to */
    Bitslot slot = bits[(int) (index / (int) BITSET_SLOT_SIZE)];

    /* Create a mask for the appropriate entry in the bucket */
    Bitslot mask = 0x1 << (index % BITSET_SLOT_SIZE);

    return (mask & slot) ? TRUE : FALSE;
}

/* Set the value of a particular bit */
void setBit(Bitset bits, int index, bool val) {
    /* Find the bucket that this index belongs to */
    Bitslot* slot = bits + ((int) (index / (int) BITSET_SLOT_SIZE));

    /* Create a mask for the appropriate entry in the bucket */
    Bitslot mask = 0x1 << (index % BITSET_SLOT_SIZE);

    if (val) {
        /* Turn on the bit using 00..1..00*/
        (*slot) = (*slot) | mask;
    } else {
        /* Turn off the bit using 11..0..11 */
        (*slot) = (*slot) & (~mask);
    }
}

/* Determine if there are any empty bits in the slot for the index indicated */
Bitslot testBitSlot(Bitset bits, int slotIndex) {
    return bits[(int) (slotIndex / (int) BITSET_SLOT_SIZE)];
}

/* Set all of the bits in the bitset (also need the length of the bitset) */
void bitsetSetAll(Bitset bits, int len) {
    int i;
    for (i = 0; i < len; i++) {
        bits[i] = BITSET_SLOT_ALL;
    }
}

/* Clear all of the bits in the bitset (also need the length of the bitset) */
void bitsetClearAll(Bitset bits, int len) {
    int i;
    for (i = 0; i < len; i++) {
        bits[i] = 0;
    }
}


/* /\*********************************************************************** */
/*  * Test harness */
/*  ***********************************************************************\/ */
/* #ifdef TEST_MEM */

/* #define TEST_SIZE 100 */

/* int static_malloc_test(void) { */
/*     Bitslot bits1[TEST_SIZE]; */
/*     Bitslot bits2[TEST_SIZE]; */
/*     int i; */
/*     bool pass = TRUE; */
/*     bool overallPass = TRUE; */

/*     /\* */
/*      * Test the bitset implementation  */
/*      *\/ */

/*     /\* Test1: Setting and clearing an entire bitset *\/ */
/*     bitsetClearAll(bits1, TEST_SIZE); */
/*     bitsetSetAll(bits2, TEST_SIZE); */

/*     for (i = 0; i < TEST_SIZE * BITSET_SLOT_SIZE; i++) { */
/*     	if (testBit(bits1, i)) { */
/*     		slb_out_description("Test1: Failed to clear bit:", i); */
/*     		pass = FALSE; */
/*     	} */
/*     	if (!testBit(bits2, i)) { */
/*     		slb_out_description("Test1: Failed to set bit:", i); */
/*     		pass = FALSE; */
/*     	} */
/*     } */
/*     if (pass) { */
/*     	slb_out_info("Test1: Passed!"); */
/*     } else { */
/*     	overallPass = FALSE; */
/*     	slb_out_info("Test1: FAILED"); */
/*     } */


/*     /\* Test2: Setting and clearing alternating individual bits *\/ */
/*     pass = TRUE; */
/*     bitsetClearAll(bits1, TEST_SIZE); */
/*     bitsetClearAll(bits2, TEST_SIZE); */

/*     /\* set alternating bits *\/ */
/*     for (i = 0; i < TEST_SIZE * BITSET_SLOT_SIZE; i++) { */
/*     	if (i % 2) { */
/*     		setBit(bits1, i, TRUE); */
/*     	} else { */
/*     		setBit(bits2, i, TRUE); */
/*     	} */
/*     } */

/*     /\* Test for success *\/ */
/*     for (i = 0; i < TEST_SIZE; i++) { */
/*     	if (bits1[i] & bits2[i]) { */
/*     		slb_out_description("Test2: Bad bits in slot _ don't alternate", i); */
/*     		pass = FALSE; */
/*     	} else if ((bits1[i] | bits2[i]) != BITSET_SLOT_ALL) { */
/*     		slb_out_description("Test2: Failed to jointly set all of the bits in slot:", i); */
/*     		pass=FALSE; */
/*     	} */
/*     } */
/*     if (pass) { */
/*     	slb_out_info("Test2: Passed!"); */
/*     } else { */
/*     	overallPass = FALSE; */
/*     	slb_out_info("Test2: FAILED"); */
/*     } */


/*     /\* */
/*      * Test3: malloc and free various sizes */
/*      *\/ */
/*     char *ptr = NULL; */
/*     char *prev = NULL; */
/*     pass = TRUE; */

/*     for (i = 0; i < 32; i++) { */
/*     	prev = ptr; */
/*     	ptr = static_malloc(i + 1); */
/*     	if (!ptr) { */
/*     		pass = FALSE; */
/*     		slb_out_description("Test3: Failed to allocate memory chunk of size:", (i+1)); */
/*     	} */

/*     	if (prev != NULL) { */
/*     		static_free(prev); */
/*     	} */
/*     } */
/*     static_free(ptr); */

/*     for (i = 0; i < MEMORY_BUFFER_SIZE; i+= BITSET_SLOT_SIZE) { */
/*     	if (testBitSlot(memAvail, i) != BITSET_SLOT_ALL) { */
/*     		slb_out_description("Test3: Memory in slot _ isn't available but should be", i); */
/*     		pass = FALSE; */
/*     	}  */
/*     	if (testBitSlot(memContig, i) != 0) { */
/*     		slb_out_description("Test3: Memory in slot _ is contig but shouldn't be", i); */
/*     		pass = FALSE; */
/*     	} */
/*     } */
/*     if (pass) { */
/*     	slb_out_info("Test3: Passed!"); */
/*     } else { */
/*     	overallPass = FALSE; */
/*     	slb_out_info("Test3: FAILED"); */
/*     } */


/*     /\* */
/*      * Test4: malloc and free lots of chunks  */
/*      *\/ */
/*     char* temp[MEMORY_BUFFER_SIZE / 2]; */
/*     pass = TRUE; */

/*     /\* Use up all available memory *\/ */
/*     for (i = 0; i < MEMORY_BUFFER_SIZE / 2; i++) { */
/*     	temp[i] = static_malloc(MEMORY_SLOT_SIZE * 2); */
/*     	if (!ptr) { */
/*     		pass = FALSE; */
/*     		slb_out_description("Test4: Failed to allocate memory chunk", i); */
/*             slb_out_description("...of size", MEMORY_SLOT_SIZE * 2); */
/*     	} */
/*     } */

/*     /\* Make sure there's no memory left *\/ */
/*     for (i = 0; i < MEMORY_BUFFER_SIZE; i+= BITSET_SLOT_SIZE) { */
/*     	if (testBitSlot(memAvail, i) != 0) { */
/*     		slb_out_description("Test4: Memory in slot _ is available but shouldn't be", i); */
/*     		pass = FALSE; */
/*     	} */
/*     } */

/*     /\* Now free up the memory *\/ */
/*     for (i = 0; i < MEMORY_BUFFER_SIZE / 2; i++) { */
/*     	static_free(temp[i]); */
/*     } */

/*     /\* Make sure everything got cleaned up *\/ */
/*     for (i = 0; i < MEMORY_BUFFER_SIZE; i+= BITSET_SLOT_SIZE) { */
/*     	if (testBitSlot(memAvail, i) != BITSET_SLOT_ALL) { */
/*     		slb_out_description("Test4: Memory in slot _ isn't available but should be", i); */
/*     		pass = FALSE; */
/*     	}  */
/*     	if (testBitSlot(memContig, i) != 0) { */
/*     		slb_out_description("Test4: Memory in slot _ is contig but shouldn't be", i); */
/*     		pass = FALSE; */
/*     	} */
/*     } */
/*     if (pass) { */
/*     	slb_out_info("Test4: Passed"); */
/*     } else { */
/*     	overallPass = FALSE; */
/*     	slb_out_info("Test4: FAILED"); */
/*     } */




/*     if (overallPass) { */
/*     	slb_out_info("Test Suite: Passed!"); */
/*     } else { */
/*     	overallPass = FALSE; */
/*     	slb_out_info("Test Suite: FAILED"); */
/*     } */

/*     return overallPass; */
/* } */
/* #else */
int static_malloc_test(void) {
    return 0;
}
/* #endif */
