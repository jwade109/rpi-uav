CC = g++
CFLAGS = -g -std="c++11"
LDFLAGS = -g -c
LIB = -lm -lwiringPi -L/usr/local/lib -lncurses
INC = -I include
EIGEN = -I /usr/local/include/eigen3

all: pid time kalman gpiotest serialtest bmptest control

pid: bin/pid
time: bin/time
kalman: bin/kalman
gpiotest: bin/gpiotest
serialtest: bin/serialtest
bmptest: bin/bmptest
control: bin/control

bin/pid: build/timeutil.o build/PID.o build/pidtest.o
	@echo "Building pid"
	$(CC) $(CFLAGS) build/timeutil.o build/PID.o build/pidtest.o -o bin/pid $(LIB)

bin/time: build/timeutil.o build/timetest.o
	@echo "Building time"
	$(CC) $(CFLAGS) build/timeutil.o build/timetest.o -o bin/time $(LIB)

bin/kalman: build/Kalman.o build/kalmantest.o build/timeutil.o
	@echo "Building kalman"
	$(CC) $(CFLAGS) build/Kalman.o build/kalmantest.o build/timeutil.o -o bin/kalman $(LIB)

build/kalmantest.o: test/kalmantest.cpp
	@echo "Building kalmantest.o"
	$(CC) $(LDFLAGS) $(INC) $(EIGEN) test/kalmantest.cpp -o build/kalmantest.o

build/Kalman.o: src/Kalman.cpp include/Kalman.h
	@echo "Building Kalman.o"
	$(CC) $(LDFLAGS) $(INC) $(EIGEN) src/Kalman.cpp -o build/Kalman.o

build/timetest.o: test/timetest.cpp
	@echo "Building timetest.o"
	$(CC) $(LDFLAGS) $(INC) test/timetest.cpp -o build/timetest.o

build/timeutil.o: src/timeutil.cpp include/timeutil.h
	@echo "Building timeutil.o"
	$(CC) $(LDFLAGS) $(INC) src/timeutil.cpp -o build/timeutil.o

build/I2C.o: src/I2C.cpp include/I2C.h
	@echo "Building I2C.o"
	$(CC) $(LDFLAGS) $(INC) src/I2C.cpp -o build/I2C.o

build/pidtest.o: test/pidtest.cpp
	@echo "Building pidtest.o"
	$(CC) $(LDFLAGS) $(INC) test/pidtest.cpp -o build/pidtest.o

build/PID.o: src/PID.cpp include/PID.h
	@echo "Building PID.o"
	$(CC) $(LDFLAGS) $(INC) src/PID.cpp -o build/PID.o

bin/gpiotest: build/gpiotest.o build/timeutil.o
	@echo "Building gpiotest"
	$(CC) $(CFLAGS) build/timeutil.o build/gpiotest.o -o bin/gpiotest $(LIB)

build/gpiotest.o: test/gpiotest.cpp
	@echo "Building gpiotest.o"
	$(CC) $(LDFLAGS) $(INC) test/gpiotest.cpp -o build/gpiotest.o

build/SerialIMU.o: src/SerialIMU.cpp include/SerialIMU.h include/Message.h
	@echo "Building SerialIMU.o"
	$(CC) $(LDFLAGS) $(INC) src/SerialIMU.cpp -o build/SerialIMU.o

build/serialtest.o: test/serialtest.cpp
	@echo "Building serialtest.o"
	$(CC) $(LDFLAGS) $(INC) test/serialtest.cpp -o build/serialtest.o

bin/serialtest: build/serialtest.o build/SerialIMU.o build/timeutil.o
	@echo "Building serialtest"
	$(CC) $(CFLAGS) build/serialtest.o build/SerialIMU.o build/timeutil.o -o bin/serialtest

build/BMP085.o: src/BMP085.cpp include/BMP085.h
	@echo "Building BMP085.o"
	$(CC) $(LDFLAGS) $(INC) src/BMP085.cpp -o build/BMP085.o

build/bmptest.o: test/bmptest.cpp
	@echo "Building bmptest.o"
	$(CC) $(LDFLAGS) $(INC) test/bmptest.cpp -o build/bmptest.o

bin/bmptest: build/bmptest.o build/BMP085.o build/timeutil.o build/I2C.o
	@echo "Building bmptest"
	$(CC) $(CFLAGS) build/bmptest.o build/BMP085.o build/timeutil.o build/I2C.o -o bin/bmptest $(LIB)

build/control.o: test/control.cpp
	@echo "Building control.o"
	$(CC) $(LDFLAGS) $(INC) test/control.cpp -o build/control.o

bin/control: build/control.o build/BMP085.o build/timeutil.o build/I2C.o build/SerialIMU.o build/PID.o
	@echo "Building control"
	$(CC) $(CFLAGS) build/control.o build/BMP085.o build/timeutil.o build/I2C.o build/SerialIMU.o build/PID.o -o bin/control $(LIB)

clean:
	@echo "Cleaning..."
	rm build/* bin/*
