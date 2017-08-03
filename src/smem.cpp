#include <sys/mman.h>
#include <smem.h>

void* sharedmem(size_t size)
{
    int prot = PROT_READ | PROT_WRITE;
    int visib = MAP_ANONYMOUS | MAP_SHARED;
    return mmap(0, size, prot, visib, 0, 0);
}
