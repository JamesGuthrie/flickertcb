#ifndef __DEBUG_H__
#define __DEBUG_H__

void dump_populated_areas(uint8_t *pts);
void dump_cpu_t(cpu_t* s);
void get_pcr17(void);
void gdt_debug(void);
void stress_malloc(void);

#endif /* __DEBUG_H__ */
