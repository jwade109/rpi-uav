CC = g++
CFLAGS = -g
LIB = -lm
INC = -I include -I /usr/local/include/eigen3

all: takeoff pid

takeoff: build/main.o
	$(CC) $(CFLAGS) build/main.o -o takeoff $(LIB)

build/main.o: src/main.cpp
	$(CC) -c $(INC) src/main.cpp -o build/main.o

pid:
	$(CC) $(CFLAGS) test/pidtest.cpp src/pid_lib.c $(INC) $(LIB) -o pid

clean:
	rm build/* takeoff pid
