CC = g++
CFLAGS = -g
LIB = -lm
INC = -I include
EIGEN = -I /usr/local/include/eigen3

all: pid time kalman

pid: build/TimeUtil.o build/PID.o build/pidtest.o
	@echo "Building pid"
	$(CC) $(CFLAGS) build/TimeUtil.o build/PID.o build/pidtest.o -o pid $(LIB)

time: build/TimeUtil.o build/timetest.o
	@echo "Building time"
	$(CC) $(CFLAGS) build/TimeUtil.o build/timetest.o -o time $(LIB)

kalman: build/Kalman.o build/kalmantest.o build/TimeUtil.o
	@echo "Building kalman"
	$(CC) $(CFLAGS) build/Kalman.o build/kalmantest.o build/TimeUtil.o -o kalman $(LIB)

build/kalmantest.o: test/kalmantest.cpp
	@echo "Building kalmantest.o"
	$(CC) -c $(INC) $(EIGEN) test/kalmantest.cpp -o build/kalmantest.o

build/Kalman.o: src/Kalman.cpp include/Kalman.h
	@echo "Building Kalman.o"
	$(CC) -c $(INC) $(EIGEN) src/Kalman.cpp -o build/Kalman.o

build/timetest.o: test/timetest.cpp
	@echo "Building timetest.o"
	$(CC) -c $(INC) test/timetest.cpp -o build/timetest.o

build/TimeUtil.o: src/TimeUtil.cpp include/TimeUtil.h
	@echo "Building TimeUtil.o" 
	$(CC) -c $(INC) src/TimeUtil.cpp -o build/TimeUtil.o

build/pidtest.o: test/pidtest.cpp
	@echo "Building pidtest.o"
	$(CC) -c $(INC) test/pidtest.cpp -o build/pidtest.o

build/PID.o: src/PID.cpp include/PID.h
	@echo "Building PID.o"
	$(CC) -c $(INC) src/PID.cpp -o build/PID.o

clean:
	@echo "Cleaning..."
	rm build/* pid time kalman
