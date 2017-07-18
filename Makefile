CC = g++
CFLAGS = -g -std="c++11"
LIB = -lm -lwiringPi -L/usr/local/lib
INC = -I include
EIGEN = -I /usr/local/include/eigen3

all: bin/pid bin/time bin/kalman bin/gpiotest bin/serialtest

pid: bin/pid
time: bin/time
kalman: bin/kalman
gpiotest: bin/gpiotest
serialtest: bin/serialtest

bin/pid: build/TimeUtil.o build/PID.o build/pidtest.o
	@echo "Building pid"
	$(CC) $(CFLAGS) build/TimeUtil.o build/PID.o build/pidtest.o -o bin/pid $(LIB)

bin/time: build/TimeUtil.o build/timetest.o
	@echo "Building time"
	$(CC) $(CFLAGS) build/TimeUtil.o build/timetest.o -o bin/time $(LIB)

bin/kalman: build/Kalman.o build/kalmantest.o build/TimeUtil.o
	@echo "Building kalman"
	$(CC) $(CFLAGS) build/Kalman.o build/kalmantest.o build/TimeUtil.o -o bin/kalman $(LIB)

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

bin/gpiotest: build/gpiotest.o build/TimeUtil.o
	@echo "Building gpiotest"
	$(CC) $(CFLAGS) build/TimeUtil.o build/gpiotest.o -o bin/gpiotest $(LIB)

build/gpiotest.o: test/gpiotest.cpp
	@echo "Building gpiotest.o"
	$(CC) -c $(INC) test/gpiotest.cpp -o build/gpiotest.o

build/SerialIMU.o: src/SerialIMU.cpp include/SerialIMU.h include/Message.h
	@echo "Building SerialIMU.o"
	$(CC) -c $(INC) src/SerialIMU.cpp -o build/SerialIMU.o

build/serialtest.o: test/serialtest.cpp
	@echo "Building serialtest.o"
	$(CC) -c $(INC) test/serialtest.cpp -o build/serialtest.o

bin/serialtest: build/serialtest.o build/SerialIMU.o build/TimeUtil.o
	@echo "Building serialtest"
	$(CC) $(CFLAGS) build/serialtest.o build/SerialIMU.o build/TimeUtil.o -o bin/serialtest

clean:
	@echo "Cleaning..."
	rm build/* bin/*
