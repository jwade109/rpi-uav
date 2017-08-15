#include <stdio.h>
#include <inttypes.h>
#include <timeutil.h>

int main()
{
    printf("Wait for 2500 ms: ");
    timer();
    waitfor(2500);
    uint64_t t = timer(micro);
    printf("\t%" PRIu64 " us \n", t);
    printf("Wait for 34123 us: ");
    timer();
    waitfor(34123, micro);
    t = timer(micro);
    printf("\t%" PRIu64 " us\n", t);
    return 0;
}
