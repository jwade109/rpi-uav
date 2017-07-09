#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <stdlib.h>
#include <unistd.h>
#include <TimeUtil.h>

#define MAX_COUNT 200
#define BUF_SIZE  100

int main()
{
    pid_t pid;
    int i;
    char buf[BUF_SIZE];

    printf("Forking...\n");
    fork();
    pid = getpid();

    waitFor(5, SEC);

    // for (i = 1; i <= MAX_COUNT; i++) {
         sprintf(buf, "This line is from pid %d, value = %d\n", pid, i);
         write(1, buf, strlen(buf));
    // }
    return 0;
}
