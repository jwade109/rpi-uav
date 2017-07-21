#include <unistd.h>
#include <sys/mman.h>
#include <fstream>
#include <Message.h>

#define MSG_LEN 100

class SerialIMU
{
    public:

    Message last;

    SerialIMU();
    ~SerialIMU();

    int begin();
    Message get();

    private:

    char buffer[MSG_LEN];
    std::ifstream in;
    char* mem;
    int child_pid;

    char* create_shared_memory(size_t size);
    Message parseMessage(char* buffer);
};
