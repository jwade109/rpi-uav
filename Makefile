CC = g++
CFLAGS = -g
LIB = -lm
INC = -I include
EIGEN = -I /usr/local/include/eigen3

all: takeoff pid time

takeoff: build/main.o
	$(CC) $(CFLAGS) build/main.o -o takeoff $(LIB)

build/main.o: src/main.cpp
	$(CC) -c $(INC) $(EIGEN)  src/main.cpp -o build/main.o

pid: test/pidtest.cpp src/PID.cpp include/PID.h src/TimeUtil.cpp include/TimeUtil.h
	$(CC) $(CFLAGS) test/pidtest.cpp src/PID.cpp src/TimeUtil.cpp $(INC) $(LIB) -o pid

time: test/time.cpp src/TimeUtil.cpp include/TimeUtil.h
	$(CC) $(CFLAGS) test/time.cpp src/TimeUtil.cpp $(INC) -o time

clean:
	rm build/* takeoff pid time
