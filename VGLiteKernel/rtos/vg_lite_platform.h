/****************************************************************************
*
*    Copyright (c) 2014 - 2022 Vivante Corporation
*
*    Permission is hereby granted, free of charge, to any person obtaining a
*    copy of this software and associated documentation files (the "Software"),
*    to deal in the Software without restriction, including without limitation
*    the rights to use, copy, modify, merge, publish, distribute, sublicense,
*    and/or sell copies of the Software, and to permit persons to whom the
*    Software is furnished to do so, subject to the following conditions:
*
*    The above copyright notice and this permission notice shall be included in
*    all copies or substantial portions of the Software.
*
*    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
*    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
*    DEALINGS IN THE SOFTWARE.
*
*****************************************************************************/

#ifndef _VG_LITE_PLATFORM_H
#define _VG_LITE_PLATFORM_H

#include "stdint.h"
#include "stdlib.h"
#include <stdio.h>
#include "../vg_lite_debug.h"
#include "../vg_lite_type.h"

#define _BAREMETAL 0

/* Implementation of list. ****************************************/
typedef struct list_head {
    struct list_head *next;
    struct list_head *prev;
}list_head_t;

typedef struct heap_node {
    list_head_t list;
    uint32_t offset;        /* physical in DMA allocator or offset in Reserved memory */
    unsigned long size;     /* allocate bytes in DMA allocator */
    int32_t status;
    uint32_t flags;         /* allocate memory from DMA or Reserved region */
    void* memory;           /* DMA allocator: user logical */
    void* kmemory;          /* DMA allocator: kernel logical  */
}heap_node_t;

/*!
@brief Initialize the hardware mem setting.
*/
void vg_lite_init_mem(uint32_t register_mem_base,
                      uint32_t gpu_mem_base,
                      volatile void * contiguous_mem_base,
                      uint32_t contiguous_mem_size);

/*!
@brief The hardware IRQ handler.
*/
void vg_lite_IRQHandler(void);

#endif
