CC = g++
CFLAGS = -g
LIB = -lm
INC = -I include

all: takeoff pid

takeoff: build/main.o
	mkdir -p bin
	$(CC) $(CFLAGS) build/main.o -o bin/takeoff $(LIB)

build/main.o: src/main.cpp
	mkdir -p build
	$(CC) -c $(INC) src/main.cpp -o build/main.o

pid:
	mkdir -p bin
	$(CC) $(CFLAGS) test/pidtest.cpp src/pid_lib.c $(INC) $(LIB) -o bin/pid

clean:
	rm -r build bin
