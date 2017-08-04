#include <stdio.h>
#include <string.h>
#include <malloc.h>
#include <filebuffer.h>

typedef struct
{
    uint8_t x;
    uint16_t y;
    uint8_t z;
    uint16_t m[3];
}
example;

int main()
{
    FileBuffer bin("data.bin");

    example ex = {45, 4.5, 255, {43, 30, 19}};

    printf("%d %d %d\n", sizeof(ex.x), sizeof(ex.y), sizeof(ex.z));
    printf("%d %d %d\n", &ex, &ex.x, &ex.y, &ex.z);
    char buf[sizeof(example)];
    memcpy(buf, (char*) &ex, sizeof(example));
    bin.push(buf, sizeof(example));
    bin.flush();
    for (int i = 0; i < sizeof(example); i++)
        printf("%02x ", buf[i]);
    printf("\n");
    return 0;
}
