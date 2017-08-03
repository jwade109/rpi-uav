#ifndef SMEM_H
#define SMEM_H

#include <sys/mman.h>

// a convenience function since I use this a lot

void* sharedmem(size_t size);

#endif // SMEM_H
