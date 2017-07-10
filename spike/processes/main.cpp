#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <stdlib.h>
#include <unistd.h>
#include <TimeUtil.h>

int main(int argc, char** argv)
{
    char buf[100];

    if (argc == 1)
    {
        printf("Counting to 100 in 2 processes.\n");
        pid_t pid = fork();
        if (pid > 0) // parent
        {
            for (int i = 1; i < 100; i+=2)
            {
                waitFor(50, MILLI);
                sprintf(buf, "pid: %d, i: %d\n", pid, i);
                write(1, buf, strlen(buf));
            }
        }
        else if (pid == 0) // child
        {
            for (int i = 0; i < 100; i+=2)
            {
                sprintf(buf, "pid: %d, i: %d\n", pid, i);
                write(1, buf, strlen(buf));
                waitFor(50, MILLI);
            }
        }
        return 0;
    }
    else
    {
        printf("Counting to 100 with just one process.\n");
        for (int i = 0; i < 100; i++)
        {
            printf("%d\n", i);
            waitFor(50, MILLI);
        }
    }
    return 1;
}
