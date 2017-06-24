CC = g++
CFLAGS = -g
LIB = -lm
INC = -I include

all: takeoff pid

takeoff: build/main.o
	@$(CC) $(CFLAGS) build/main.o -o takeoff $(LIB)

build/main.o: src/main.cpp
	@mkdir -p build
	@$(CC) -c $(INC) src/main.cpp -o build/main.o

pid:
	@$(CC) $(CFLAGS) test/pidtest.cpp src/pid_lib.c $(INC) $(LIB) -o pid

clean:
	@rm -r build takeoff pid
