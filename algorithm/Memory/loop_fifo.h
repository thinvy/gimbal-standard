#ifndef LOOP_FIFO_H
#define LOOP_FIFO_H

#include "struct_typedef.h"

extern void LoopFifoFp32_init(LoopFifoFp32_t *fifo, fp32 *address, uint32_t size);
extern void LoopFifoFp32_push(LoopFifoFp32_t *fifo, fp32 data);
extern fp32 LoopFifoFp32_read(LoopFifoFp32_t *fifo, uint32_t delta);
extern void LoopFifoFp32_clear(LoopFifoFp32_t *fifo);

#endif
