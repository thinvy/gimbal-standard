#include "loop_fifo.h"
#include "string.h"

void LoopFifoFp32_init(LoopFifoFp32_t *fifo, fp32 *address, uint32_t size)
{
    memset(address, 0, size);
    fifo->ptr = address;
    fifo->size = size;
}

void LoopFifoFp32_push(LoopFifoFp32_t *fifo, fp32 data)
{
    if (fifo->size == 0) {
        return;
    }
    
    *fifo->ptr = data;
    
    if (fifo->offset+1 < fifo->size) {
        fifo->offset++;
        fifo->ptr++;
    }
    else {
        fifo->offset = 0;
        fifo->ptr -= (fifo->size - 1);
    }
}


fp32 LoopFifoFp32_read(LoopFifoFp32_t *fifo, uint32_t delta)
{
    if (delta < 0) {
        delta = 0;
    }
    
    if (delta >= fifo->size) {
        delta = fifo->size - 1;
    }
    
    if (fifo->offset >= delta) {
        return *(fifo->ptr  - delta);
    }
    else {
        return *(fifo->ptr + fifo->size - delta);
    }
}


void LoopFifoFp32_clear(LoopFifoFp32_t *fifo)
{
    fifo->ptr -= fifo->offset;
    fifo->offset = 0;
    memset(fifo->ptr, 0, fifo->size);
}

